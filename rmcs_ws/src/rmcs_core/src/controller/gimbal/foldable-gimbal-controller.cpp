#include "controller/gimbal/eccentric_dual_yaw_solver.hpp"
#include "controller/pid/pid_calculator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>
#include <rclcpp/logging.hpp>
#include <utility>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/sentry_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::gimbal {
using namespace rmcs_description;

enum class FoldState : int {
    UnFold = 0,
    MovingToFoldPose = 1,
    Folding = 2,
    Folded = 3,
    UnFolding = 4,
    Fault = 5,
};

// 对 top 关节自瞄参考角做差分+低通滤波的目标角速度前馈，
// 补偿斜坡跟踪滞后；切板跳变时清零。
struct YawRateFeedforward {
    double gain = 1.0;
    double cutoff_hz = 15.0;
    double max_rate = 6.0;
    double jump_threshold = 0.05;

    auto update(double azimuth, std::chrono::steady_clock::time_point now) -> double {
        if (std::isfinite(prev_azimuth_)) {
            const auto dt = std::chrono::duration<double>(now - prev_timestamp_).count();
            const auto delta = limit_rad(azimuth - prev_azimuth_);
            if (std::abs(delta) > jump_threshold) {
                filtered_rate_ = 0.0;
            } else if (dt > kMinDt) {
                const auto raw = delta / dt;
                const auto alpha = dt / (dt + 1.0 / (2.0 * std::numbers::pi * cutoff_hz));
                filtered_rate_ += alpha * (raw - filtered_rate_);
            }
        }
        prev_azimuth_ = azimuth;
        prev_timestamp_ = now;
        return gain * std::clamp(filtered_rate_, -max_rate, max_rate);
    }

private:
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kMinDt = 1e-6;

    static auto limit_rad(double angle) -> double {
        constexpr double kPi = std::numbers::pi_v<double>;
        while (angle > kPi)
            angle -= 2.0 * kPi;
        while (angle <= -kPi)
            angle += 2.0 * kPi;
        return angle;
    }

    double prev_azimuth_ = kNaN;
    double filtered_rate_ = 0.0;
    std::chrono::steady_clock::time_point prev_timestamp_{};
};

class FoldableGimbalController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    FoldableGimbalController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        get_parameter_or("top_yaw_velocity_ff_gain", top_yaw_ff_.gain, 1.0);
        get_parameter_or("top_yaw_ff_cutoff_hz", top_yaw_ff_.cutoff_hz, 15.0);
        get_parameter_or("top_yaw_ff_max", top_yaw_ff_.max_rate, 6.0);
        get_parameter_or("top_yaw_ff_jump_threshold", top_yaw_ff_.jump_threshold, 0.05);
        fold_ready_time_ = std::max(get_parameter_or("fold_ready_time", 0.2), 1e-3);
        fold_timeout_ = std::max(get_parameter_or("fold_timeout", 5.0), 0.0);
        fold_velocity_tolerance_ =
            std::max(get_parameter_or("fold_velocity_tolerance", 0.05), 1e-6);
        fold_angle_tolerance_ = 
            std::max(get_parameter_or("fold_angle_tolerance", 0.05), 1e-6);

    }

    auto before_updating() -> void override {
        if (!input_.navigation_enable_control.ready()) {
            input_.navigation_enable_control.make_and_bind_directly(false);
            input_.navigation_toward.make_and_bind_directly(kVecNaN);
            RCLCPP_INFO(get_logger(), "Manual mode without navigation gimbal control");
        }

        enter_disabled_state();
        previous_actual_yaw_ = current_barrel_yaw_pitch().first;
        previous_yaw_timestamp_ = *input_.timestamp;
        last_update_timestamp_ = *input_.timestamp;
        last_rotary_knob_switch_ = *input_.rotary_knob_switch;
        fold_state_ = FoldState::Folded;
        fold_ready_elapsed_ = 0.0;
        fold_transition_elapsed_ = 0.0;
        locked_bottom_yaw_target_ = current_bottom_world_yaw();
        publish_fold_state();
    }

    auto update() -> void override {
        const auto actual_yaw_pitch = current_barrel_yaw_pitch();
        *output_.yaw_angle = *input_.bottom_yaw_angle;
        *output_.yaw_velocity = compute_actual_yaw_velocity(actual_yaw_pitch.first);

        if (!input_.enable_control()) {
            enter_disabled_state();
            last_rotary_knob_switch_ = *input_.rotary_knob_switch;
            fold_state_ = FoldState::Folded;
            locked_bottom_yaw_target_ = current_bottom_world_yaw();
            return;
        }

        const auto dt = update_dt();
        const bool fold_switch_up_edge =
            last_rotary_knob_switch_ == rmcs_msgs::Switch::MIDDLE
            && *input_.rotary_knob_switch == rmcs_msgs::Switch::UP;
        last_rotary_knob_switch_ = *input_.rotary_knob_switch;

        if (fold_switch_up_edge)
            switch_fold_state();

        if (!fold_parameters_ready()) {
            enter_fold_fault();
            reset_all_controls();
            publish_fold_state();
            return;
        }

        switch (fold_state_) {
        case FoldState::UnFold:
            update_normal_gimbal_control();
            apply_roll_control(roll_unfold_angle_);
            break;
        case FoldState::MovingToFoldPose:
            update_fold_pose_control(actual_yaw_pitch);
            apply_roll_control(roll_unfold_angle_);
            update_move_to_fold_pose(actual_yaw_pitch, dt);
            break;
        case FoldState::Folding:
            update_fold_pose_control(actual_yaw_pitch);
            apply_roll_control(roll_folded_angle_);
            update_folding(dt);
            break;
        case FoldState::Folded:
            update_fold_pose_control(actual_yaw_pitch);
            apply_roll_control(roll_folded_angle_);
            break;
        case FoldState::UnFolding:
            update_fold_pose_control(actual_yaw_pitch);
            apply_roll_control(roll_unfold_angle_);
            update_unfolding(dt);
            break;
        case FoldState::Fault:
            update_fault_control();
            apply_roll_control(*input_.roll_angle);
            break;
        }

        publish_fold_state();
    }

private:
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    static inline auto kVecNaN = Eigen::Vector2d{kNaN, kNaN};
    static constexpr double kEpsilon = 1e-9;
    static constexpr double kMinDt = 1e-6;
    static constexpr double kJoystickSensitivity = 0.006;
    static constexpr double kMouseSensitivity = 0.5;

    const double upper_limit_{get_parameter("upper_limit").as_double()};
    const double lower_limit_{get_parameter("lower_limit").as_double()};
    const double roll_folded_angle_{get_parameter("roll_folded_angle").as_double()};
    const double roll_unfold_angle_{get_parameter("roll_unfold_angle").as_double()};
    const double top_yaw_folded_angle_{get_parameter("top_yaw_folded_angle").as_double()};
    const double pitch_folded_angle_{get_parameter("pitch_folded_angle").as_double()};

    EccentricDualYawSolver solver_;

    YawRateFeedforward top_yaw_ff_;

    struct Input {
        explicit Input(rmcs_executor::Component& component) {
            component.register_input("/remote/joystick/left", joystick_left);
            component.register_input("/remote/switch/right", switch_right);
            component.register_input("/remote/switch/left", switch_left);
            component.register_input("/remote/rotary_knob_switch", rotary_knob_switch);

            component.register_input("/remote/mouse/velocity", mouse_velocity);
            component.register_input("/remote/mouse", mouse);

            component.register_input("/predefined/timestamp", timestamp);
            component.register_input("/tf", tf);

            component.register_input("/gimbal/top_yaw/angle", top_yaw_angle);
            component.register_input("/gimbal/top_yaw/velocity", top_yaw_velocity);
            component.register_input("/gimbal/bottom_yaw/angle", bottom_yaw_angle);
            component.register_input("/gimbal/bottom_yaw/velocity", bottom_yaw_velocity);
            component.register_input("/gimbal/pitch/angle", pitch_angle);
            component.register_input("/gimbal/pitch/velocity", pitch_velocity);
            component.register_input("/gimbal/roll/angle", roll_angle);
            component.register_input("/gimbal/roll/velocity", roll_velocity);
            component.register_input("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu);
            component.register_input(
                "/auto_aim/control_direction", auto_aim_control_direction, false);
            component.register_input("/auto_aim/robot_center", auto_aim_robot_center, false);
            component.register_input(
                "/rmcs_navigation/enable_control", navigation_enable_control, false);
            component.register_input("/rmcs_navigation/gimbal_toward", navigation_toward, false);
        }

        auto enable_control() const noexcept -> bool {
            using namespace rmcs_msgs;
            if ((*switch_left == Switch::UNKNOWN || *switch_right == Switch::UNKNOWN)
                || (*switch_left == Switch::DOWN && *switch_right == Switch::DOWN)) {
                return false;
            }
            return true;
        }

        auto enable_autoaim() const noexcept -> bool {
            using namespace rmcs_msgs;
            if (*switch_right != Switch::UP && !mouse->right)
                return false;
            if (!auto_aim_control_direction.ready())
                return false;
            const auto& dir = *auto_aim_control_direction;
            if (!auto_aim_robot_center.ready())
                return false;
            const auto& center = *auto_aim_robot_center;
            return !dir.isZero() && std::isfinite(dir.x()) && std::isfinite(dir.y())
                && std::isfinite(dir.z()) && !center.isZero() && std::isfinite(center.x())
                && std::isfinite(center.y()) && std::isfinite(center.z());
        }

        auto enable_navigation() const noexcept -> bool {
            if (!*navigation_enable_control || !navigation_toward.ready())
                return false;

            return true;
        }

        InputInterface<rmcs_msgs::Switch> rotary_knob_switch;
        InputInterface<Eigen::Vector2d> joystick_left;
        InputInterface<rmcs_msgs::Switch> switch_right;
        InputInterface<rmcs_msgs::Switch> switch_left;
        InputInterface<Eigen::Vector2d> mouse_velocity;
        InputInterface<rmcs_msgs::Mouse> mouse;

        InputInterface<std::chrono::steady_clock::time_point> timestamp;
        InputInterface<Tf> tf;

        InputInterface<double> top_yaw_angle;
        InputInterface<double> top_yaw_velocity;
        InputInterface<double> bottom_yaw_angle;
        InputInterface<double> bottom_yaw_velocity;
        InputInterface<double> pitch_angle;
        InputInterface<double> pitch_velocity;
        InputInterface<double> roll_angle;
        InputInterface<double> roll_velocity;
        InputInterface<double> chassis_yaw_velocity_imu;

        InputInterface<Eigen::Vector3d> auto_aim_control_direction;
        InputInterface<Eigen::Vector3d> auto_aim_robot_center;
        InputInterface<bool> navigation_enable_control;
        InputInterface<Eigen::Vector2d> navigation_toward;
    } input_{*this};

    struct Output {
        explicit Output(rmcs_executor::Component& component) {
            component.register_output(
                "/gimbal/top_yaw/control_torque", top_yaw_control_torque, kNaN);
            component.register_output(
                "/gimbal/bottom_yaw/control_torque", bottom_yaw_control_torque, kNaN);
            component.register_output("/gimbal/pitch/control_torque", pitch_control_torque, kNaN);
            component.register_output("/gimbal/roll/control_torque", roll_control_torque, kNaN);

            component.register_output(
                "/gimbal/yaw/control_angle_error", yaw_control_angle_error, kNaN);
            component.register_output("/gimbal/yaw/angle", yaw_angle, 0.0);
            component.register_output("/gimbal/yaw/velocity", yaw_velocity, 0.0);
            component.register_output("/gimbal/fold/state", fold_state, 0);
            component.register_output("/gimbal/fold/active", fold_active, false);
            component.register_output("/gimbal/fold/fault", fold_fault, false);
        }

        OutputInterface<double> top_yaw_control_torque;
        OutputInterface<double> bottom_yaw_control_torque;
        OutputInterface<double> pitch_control_torque;
        OutputInterface<double> roll_control_torque;

        OutputInterface<double> yaw_control_angle_error;
        OutputInterface<double> yaw_angle;
        OutputInterface<double> yaw_velocity;

        OutputInterface<int> fold_state;
        OutputInterface<bool> fold_active;
        OutputInterface<bool> fold_fault;
    } output_{*this};

    pid::PidCalculator top_yaw_angle_pid_{pid::make_pid_calculator(*this, "top_yaw_angle_")};
    pid::PidCalculator top_yaw_velocity_pid_{pid::make_pid_calculator(*this, "top_yaw_velocity_")};
    pid::PidCalculator bottom_yaw_angle_pid_{pid::make_pid_calculator(*this, "bottom_yaw_angle_")};
    pid::PidCalculator bottom_yaw_velocity_pid_{
        pid::make_pid_calculator(*this, "bottom_yaw_velocity_")};
    pid::PidCalculator pitch_angle_pid_{pid::make_pid_calculator(*this, "pitch_angle_")};
    pid::PidCalculator pitch_velocity_pid_{pid::make_pid_calculator(*this, "pitch_velocity_")};
    pid::PidCalculator roll_angle_pid_{pid::make_pid_calculator(*this, "roll_angle_")};
    pid::PidCalculator roll_velocity_pid_{pid::make_pid_calculator(*this, "roll_velocity_")};

    double stored_bottom_yaw_target_ = 0.0;
    double stored_pitch_target_ = 0.0;
    double previous_actual_yaw_ = 0.0;
    std::chrono::steady_clock::time_point previous_yaw_timestamp_{};

    rmcs_msgs::Switch last_rotary_knob_switch_ = rmcs_msgs::Switch::UNKNOWN;

    FoldState fold_state_ = FoldState::Folded;

    double locked_bottom_yaw_target_ = 0.0;

    double fold_ready_elapsed_ = 0.0;
    double fold_transition_elapsed_ = 0.0;

    double fold_ready_time_ = 0.2;
    double fold_timeout_ = 5.0;
    double fold_velocity_tolerance_ = 0.05;
    double fold_angle_tolerance_ = 0.05;

    std::chrono::steady_clock::time_point last_update_timestamp_{};

    static constexpr auto limit_rad(double angle) -> double {
        constexpr double kPi = std::numbers::pi_v<double>;
        while (angle > kPi)
            angle -= 2.0 * kPi;
        while (angle <= -kPi)
            angle += 2.0 * kPi;
        return angle;
    }

    auto update_dt() -> double {
        constexpr double kDefaultDt = 1e-3;

        const auto now = *input_.timestamp;
        if (last_update_timestamp_ == std::chrono::steady_clock::time_point{}) {
            last_update_timestamp_ = now;
            return kDefaultDt;
        }
        const auto dt = std::chrono::duration<double>(now - last_update_timestamp_).count();
        last_update_timestamp_ = now;

        return dt;
    }

    auto fold_parameters_ready() const -> bool {
        return std::isfinite(roll_folded_angle_) && std::isfinite(roll_unfold_angle_)
            && std::isfinite(top_yaw_folded_angle_) && std::isfinite(pitch_folded_angle_);
    }

    auto reset_all_controls() -> void {
        top_yaw_angle_pid_.reset();
        top_yaw_velocity_pid_.reset();
        bottom_yaw_angle_pid_.reset();
        bottom_yaw_velocity_pid_.reset();
        pitch_angle_pid_.reset();
        pitch_velocity_pid_.reset();
        roll_angle_pid_.reset();
        roll_velocity_pid_.reset();

        *output_.top_yaw_control_torque = kNaN;
        *output_.bottom_yaw_control_torque = kNaN;
        *output_.pitch_control_torque = kNaN;
        *output_.roll_control_torque = kNaN;
        *output_.yaw_control_angle_error = kNaN;
    }

    auto enter_disabled_state() -> void {
        reset_all_controls();

        solver_.update(EccentricDualYawSolver::SetDisabled{});
        stored_bottom_yaw_target_ = current_bottom_world_yaw();
        stored_pitch_target_ =
            std::clamp(limit_rad(*input_.pitch_angle), upper_limit_, lower_limit_);

        *output_.yaw_control_angle_error = kNaN;
    }

    auto compute_actual_yaw_velocity(double actual_yaw) -> double {
        const auto now = *input_.timestamp;
        const auto dt = std::chrono::duration<double>(now - previous_yaw_timestamp_).count();
        double velocity = 0.0;
        if (dt > kMinDt)
            velocity = limit_rad(actual_yaw - previous_actual_yaw_) / dt;
        previous_actual_yaw_ = actual_yaw;
        previous_yaw_timestamp_ = now;
        return velocity;
    }

    auto current_barrel_yaw_pitch() const -> std::pair<double, double> {
        auto direction = fast_tf::cast<OdomGimbalImu>(
            PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, *input_.tf);
        Eigen::Vector3d vector = *direction;
        if (vector.norm() > kEpsilon)
            vector.normalize();
        else
            vector = Eigen::Vector3d::UnitX();
        const auto xy_norm = std::hypot(vector.x(), vector.y());
        return {std::atan2(vector.y(), vector.x()), std::atan2(-vector.z(), xy_norm)};
    }

    auto current_bottom_world_yaw() const -> double {
        auto direction = fast_tf::cast<OdomGimbalImu>(
            BottomYawLink::DirectionVector{Eigen::Vector3d::UnitX()}, *input_.tf);
        Eigen::Vector3d vector = *direction;
        vector.z() = 0.0;
        if (vector.norm() > kEpsilon)
            vector.normalize();
        else
            vector = Eigen::Vector3d::UnitX();
        return std::atan2(vector.y(), vector.x());
    }

    auto switch_fold_state() -> void {
        switch (fold_state_) {
        case FoldState::UnFold:
            locked_bottom_yaw_target_ = current_bottom_world_yaw();
            fold_ready_elapsed_ = 0.0;
            fold_transition_elapsed_ = 0.0;
            fold_state_ = FoldState::MovingToFoldPose;
            break;
        case FoldState::Folded:
            fold_ready_elapsed_ = 0.0;
            fold_transition_elapsed_ = 0.0;
            fold_state_ = FoldState::UnFolding;
            break;
        case FoldState::Fault:
            RCLCPP_WARN(get_logger(), "Fold controller is in fault state; ignore fold switch.");
            break;
        default:
            RCLCPP_WARN(get_logger(), "Folding or unfolding, ignore trigger.");
            break;
        }
    }

    auto update_normal_gimbal_control() -> void {
        if (input_.enable_autoaim()) {
            const auto error = solver_.update(
                EccentricDualYawSolver::AutoAim{
                    *input_.tf,
                    *input_.top_yaw_angle,
                    *input_.auto_aim_control_direction,
                    *input_.auto_aim_robot_center,
                    upper_limit_,
                    lower_limit_,
                });
            const auto top_yaw_ff =
                top_yaw_ff_.update(solver_.top_target_azimuth(), *input_.timestamp);
            apply_control(error.bottom_yaw, error.top_yaw, error.pitch, top_yaw_ff);
            const auto [_, current_pitch] = current_barrel_yaw_pitch();
            stored_bottom_yaw_target_ = limit_rad(current_bottom_world_yaw() + error.bottom_yaw);
            stored_pitch_target_ =
                std::clamp(limit_rad(current_pitch + error.pitch), upper_limit_, lower_limit_);
            return;
        }

        const auto yaw_shift = +kJoystickSensitivity * input_.joystick_left->y()
                             + kMouseSensitivity * input_.mouse_velocity->y();
        const auto pitch_shift = -kJoystickSensitivity * input_.joystick_left->x()
                               - kMouseSensitivity * input_.mouse_velocity->x();
        double nav_yshift = 0.0;
        double nav_pshift = 0.0;
        if (input_.enable_navigation()) {
            constexpr auto kGimbalFree = std::numeric_limits<double>::min();
            const auto& toward = *input_.navigation_toward;
            if (toward.x() == kGimbalFree && toward.y() == kGimbalFree) {
                enter_disabled_state();
                return;
            }
            if (std::isfinite(toward.x()))
                nav_yshift = limit_rad(toward.x() - stored_bottom_yaw_target_);
            if (std::isfinite(toward.y()))
                nav_pshift = limit_rad(
                    std::clamp(toward.y(), upper_limit_, lower_limit_) - stored_pitch_target_);
        }
        stored_bottom_yaw_target_ = limit_rad(stored_bottom_yaw_target_ + nav_yshift + yaw_shift);
        stored_pitch_target_ =
            std::clamp(stored_pitch_target_ + nav_pshift + pitch_shift, upper_limit_, lower_limit_);
        const auto [_, current_pitch] = current_barrel_yaw_pitch();
        apply_control(
            limit_rad(stored_bottom_yaw_target_ - current_bottom_world_yaw()),
            limit_rad(-*input_.top_yaw_angle),
            limit_rad(stored_pitch_target_ - current_pitch));
    }

    auto update_fold_pose_control(const std::pair<double, double>& actual_yaw_pitch) -> void {
        apply_control(
            limit_rad(locked_bottom_yaw_target_ - current_bottom_world_yaw()),
            limit_rad(top_yaw_folded_angle_ - *input_.top_yaw_angle),
            limit_rad(pitch_folded_angle_ - actual_yaw_pitch.second));
    }

    auto fold_pose_reached(const std::pair<double, double>& actual_yaw_pitch) const -> bool {
        if (!input_.top_yaw_angle.ready() || !input_.top_yaw_velocity.ready()
            || !input_.bottom_yaw_velocity.ready() || !input_.pitch_velocity.ready()
            || !std::isfinite(*input_.top_yaw_angle)
            || !std::isfinite(*input_.top_yaw_velocity)
            || !std::isfinite(*input_.bottom_yaw_velocity)
            || !std::isfinite(*input_.pitch_velocity)
            || !std::isfinite(actual_yaw_pitch.second))
            return false;
        const auto top_yaw_error = limit_rad(top_yaw_folded_angle_ - *input_.top_yaw_angle);
        const auto bottom_error =
            limit_rad(locked_bottom_yaw_target_ - current_bottom_world_yaw());
        const auto pitch_error = limit_rad(pitch_folded_angle_ - actual_yaw_pitch.second);
        return std::abs(top_yaw_error) <= fold_angle_tolerance_
            && std::abs(bottom_error) <= fold_angle_tolerance_
            && std::abs(pitch_error) <= fold_angle_tolerance_
            && std::abs(*input_.top_yaw_velocity) <= fold_velocity_tolerance_
            && std::abs(*input_.bottom_yaw_velocity) <= fold_velocity_tolerance_
            && std::abs(*input_.pitch_velocity) <= fold_velocity_tolerance_;
    }

    auto update_move_to_fold_pose(
        const std::pair<double, double>& actual_yaw_pitch, double dt) -> void {
        fold_transition_elapsed_ += dt;
        fold_ready_elapsed_ = fold_pose_reached(actual_yaw_pitch)
                                 ? fold_ready_elapsed_ + dt
                                 : 0.0;
        if (fold_ready_elapsed_ >= fold_ready_time_) {
            fold_ready_elapsed_ = 0.0;
            fold_transition_elapsed_ = 0.0;
            fold_state_ = FoldState::Folding;
        } else if (fold_transition_elapsed_ > fold_timeout_) {
            enter_fold_fault();
        }
    }

    auto update_folding(double dt) -> void {
        fold_transition_elapsed_ += dt;
        if (!input_.roll_angle.ready() || !input_.roll_velocity.ready()
            || !std::isfinite(*input_.roll_angle) || !std::isfinite(*input_.roll_velocity)) {
            enter_fold_fault();
            return;
        }
        const auto roll_error = limit_rad(roll_folded_angle_ - *input_.roll_angle);
        const bool reached = std::abs(roll_error) <= fold_angle_tolerance_
                           && std::abs(*input_.roll_velocity) <= fold_velocity_tolerance_;
        fold_ready_elapsed_ = reached ? fold_ready_elapsed_ + dt : 0.0;
        if (fold_ready_elapsed_ >= fold_ready_time_) {
            fold_ready_elapsed_ = 0.0;
            fold_transition_elapsed_ = 0.0;
            fold_state_ = FoldState::Folded;
        } else if (fold_transition_elapsed_ > fold_timeout_) {
            enter_fold_fault();
        }
    }

    auto update_unfolding(double dt) -> void {
        fold_transition_elapsed_ += dt;
        if (!input_.roll_angle.ready() || !input_.roll_velocity.ready()
            || !std::isfinite(*input_.roll_angle) || !std::isfinite(*input_.roll_velocity)) {
            enter_fold_fault();
            return;
        }
        const auto roll_error = limit_rad(roll_unfold_angle_ - *input_.roll_angle);
        const bool reached = std::abs(roll_error) <= fold_angle_tolerance_
                           && std::abs(*input_.roll_velocity) <= fold_velocity_tolerance_;
        fold_ready_elapsed_ = reached ? fold_ready_elapsed_ + dt : 0.0;
        if (fold_ready_elapsed_ >= fold_ready_time_) {
            finish_unfolding();
        } else if (fold_transition_elapsed_ > fold_timeout_) {
            enter_fold_fault();
        }
    }

    auto finish_unfolding() -> void {
        const auto [current_yaw, current_pitch] = current_barrel_yaw_pitch();
        stored_bottom_yaw_target_ = current_bottom_world_yaw();
        stored_pitch_target_ = std::clamp(current_pitch, upper_limit_, lower_limit_);
        top_yaw_angle_pid_.reset();
        top_yaw_velocity_pid_.reset();
        bottom_yaw_angle_pid_.reset();
        bottom_yaw_velocity_pid_.reset();
        pitch_angle_pid_.reset();
        pitch_velocity_pid_.reset();
        solver_.update(EccentricDualYawSolver::SetDisabled{});
        fold_ready_elapsed_ = 0.0;
        fold_transition_elapsed_ = 0.0;
        fold_state_ = FoldState::UnFold;
    }

    auto enter_fold_fault() -> void {
        fold_state_ = FoldState::Fault;
        fold_ready_elapsed_ = 0.0;
        fold_transition_elapsed_ = 0.0;
    }

    auto apply_roll_control(double target_angle) -> void {
        if (!input_.roll_angle.ready() || !input_.roll_velocity.ready()
            || !std::isfinite(*input_.roll_angle) || !std::isfinite(*input_.roll_velocity)) {
            roll_angle_pid_.reset();
            roll_velocity_pid_.reset();
            *output_.roll_control_torque = kNaN;
            return;
        }
        const auto roll_error = limit_rad(target_angle - *input_.roll_angle);
        const auto velocity_reference = roll_angle_pid_.update(roll_error);
        const auto torque =
            roll_velocity_pid_.update(velocity_reference - *input_.roll_velocity);
        *output_.roll_control_torque = std::isfinite(torque) ? torque : kNaN;
    }

    auto update_fault_control() -> void {
        if (!std::isfinite(top_yaw_folded_angle_) || !std::isfinite(pitch_folded_angle_)) {
            *output_.top_yaw_control_torque = kNaN;
            *output_.bottom_yaw_control_torque = kNaN;
            *output_.pitch_control_torque = kNaN;
            return;
        }

        top_yaw_angle_pid_.reset();
        top_yaw_velocity_pid_.reset();
        bottom_yaw_angle_pid_.reset();
        bottom_yaw_velocity_pid_.reset();
        pitch_angle_pid_.reset();
        pitch_velocity_pid_.reset();
        roll_angle_pid_.reset();
        roll_velocity_pid_.reset();

        apply_control(
            limit_rad(0),
            limit_rad(0),
            limit_rad(0));
    }

    auto current_roll_target() const -> double {
        if (!input_.roll_angle.ready() || !std::isfinite(*input_.roll_angle))
            return roll_unfold_angle_;
        const auto to_unfold = std::abs(limit_rad(roll_unfold_angle_ - *input_.roll_angle));
        const auto to_fold = std::abs(limit_rad(roll_folded_angle_ - *input_.roll_angle));
        return to_unfold <= to_fold ? roll_unfold_angle_ : roll_folded_angle_;
    }

    auto publish_fold_state() -> void {
        *output_.fold_state = static_cast<int>(fold_state_);
        *output_.fold_active = fold_state_ != FoldState::UnFold
                            && fold_state_ != FoldState::Fault;
        *output_.fold_fault = fold_state_ == FoldState::Fault;
    }

    auto apply_control(
        double bottom_yaw_error, double top_yaw_error, double pitch_error,
        double top_yaw_feedforward = 0.0) -> void {
        const auto current_bottom_velocity =
            *input_.bottom_yaw_velocity + *input_.chassis_yaw_velocity_imu;

        const auto bottom_velocity_ref = bottom_yaw_angle_pid_.update(bottom_yaw_error);
        const auto top_velocity_ref =
            top_yaw_angle_pid_.update(top_yaw_error) + top_yaw_feedforward;
        const auto pitch_velocity_ref = pitch_angle_pid_.update(pitch_error);

        *output_.top_yaw_control_torque =
            top_yaw_velocity_pid_.update(top_velocity_ref - *input_.top_yaw_velocity);
        *output_.bottom_yaw_control_torque =
            bottom_yaw_velocity_pid_.update(bottom_velocity_ref - current_bottom_velocity);
        *output_.pitch_control_torque =
            pitch_velocity_pid_.update(pitch_velocity_ref - *input_.pitch_velocity);

        *output_.yaw_control_angle_error = bottom_yaw_error;
    }
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::FoldableGimbalController, rmcs_executor::Component)
