#include "controller/gimbal/eccentric_dual_yaw_solver.hpp"
#include "controller/pid/pid_calculator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/sentry_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::gimbal {
using namespace rmcs_description;

class EccentricDualYaw
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    EccentricDualYaw()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {}

    auto before_updating() -> void override {
        if (!input_.navigation_enable_control.ready()) {
            input_.navigation_enable_control.make_and_bind_directly(false);
            input_.navigation_toward.make_and_bind_directly(kVecNaN);
            RCLCPP_INFO(get_logger(), "Manual mode without navigation gimbal control");
        }

        enter_disabled_state();
        previous_actual_yaw_ = current_barrel_yaw_pitch().first;
        previous_yaw_timestamp_ = *input_.timestamp;
    }

    auto update() -> void override {
        const auto actual_yaw_pitch = current_barrel_yaw_pitch();
        *output_.yaw_angle = *input_.bottom_yaw_angle;
        *output_.yaw_velocity = compute_actual_yaw_velocity(actual_yaw_pitch.first);

        if (!input_.enable_control()) {
            enter_disabled_state();
            return;
        }

        // Autoaim Control
        if (input_.enable_autoaim()) {
            const auto error = solver_.update(
                EccentricDualYawSolver::AutoAim{
                    *input_.auto_aim_control_direction,
                    *input_.auto_aim_robot_center,
                    upper_limit_,
                    lower_limit_,
                });
            apply_control(error.bottom_yaw, error.top_yaw, error.pitch);

            const auto [_, cur_pitch] = current_barrel_yaw_pitch();
            stored_bottom_yaw_target_ = limit_rad(current_bottom_world_yaw() + error.bottom_yaw);
            stored_pitch_target_ =
                std::clamp(limit_rad(cur_pitch + error.pitch), upper_limit_, lower_limit_);
            return;
        }

        // Navigation Control
        if (input_.enable_navigation()) {
            const auto error = solver_.update(
                EccentricDualYawSolver::Navigation{
                    *input_.navigation_toward,
                    current_bottom_world_yaw(),
                    actual_yaw_pitch.second,
                    stored_bottom_yaw_target_,
                    stored_pitch_target_,
                    upper_limit_,
                    lower_limit_,
                });
            apply_control(error.bottom_yaw, error.top_yaw, error.pitch);

            stored_bottom_yaw_target_ = limit_rad(current_bottom_world_yaw() + error.bottom_yaw);
            stored_pitch_target_ =
                std::clamp(limit_rad(actual_yaw_pitch.second + error.pitch), upper_limit_, lower_limit_);
            return;
        }

        // Manual Control
        {
            const double yaw_shift = kJoystickSensitivity * input_.joystick_left->y()
                                   + kMouseSensitivity * input_.mouse_velocity->y();
            const double pitch_shift = -kJoystickSensitivity * input_.joystick_left->x()
                                     - kMouseSensitivity * input_.mouse_velocity->x();

            stored_bottom_yaw_target_ = limit_rad(stored_bottom_yaw_target_ + yaw_shift);
            stored_pitch_target_ =
                std::clamp(stored_pitch_target_ + pitch_shift, upper_limit_, lower_limit_);

            const double bottom_yaw_error =
                limit_rad(stored_bottom_yaw_target_ - current_bottom_world_yaw());
            const double pitch_error = limit_rad(stored_pitch_target_ - actual_yaw_pitch.second);

            apply_control(
                bottom_yaw_error, limit_rad(-*input_.top_yaw_angle), pitch_error);
        }
    }

private:
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    static inline auto kVecNaN = Eigen::Vector2d{kNaN, kNaN};
    static constexpr double kUpdateRate = 1000.0;
    static constexpr double kEpsilon = 1e-9;
    static constexpr double kMinDt = 1e-6;
    static constexpr double kJoystickSensitivity = 0.006;
    static constexpr double kMouseSensitivity = 0.5;

    const double upper_limit_{get_parameter("upper_limit").as_double()};
    const double lower_limit_{get_parameter("lower_limit").as_double()};

    EccentricDualYawSolver solver_{*this};

    const double bottom_yaw_viscous_ff_gain_{get_parameter_or("bottom_yaw_viscous_ff_gain", 0.0)};
    const double bottom_yaw_coulomb_ff_gain_{get_parameter_or("bottom_yaw_coulomb_ff_gain", 0.0)};
    const double bottom_yaw_coulomb_ff_tanh_gain_{
        get_parameter_or("bottom_yaw_coulomb_ff_tanh_gain", 100.0)};
    const double bottom_yaw_ext_friction_gain_{
        get_parameter_or("bottom_yaw_ext_friction_gain", 0.0)};
    const double k_top_to_bottom_{get_parameter_or("k_top_to_bottom", 0.0)};

    const double top_yaw_viscous_ff_gain_{get_parameter_or("top_yaw_viscous_ff_gain", 0.0)};
    const double top_yaw_coulomb_ff_gain_{get_parameter_or("top_yaw_coulomb_ff_gain", 0.0)};
    const double top_yaw_coulomb_ff_tanh_gain_{
        get_parameter_or("top_yaw_coulomb_ff_tanh_gain", 100.0)};
    const double pitch_viscous_ff_gain_{get_parameter_or("pitch_viscous_ff_gain", 0.0)};
    const double pitch_coulomb_ff_gain_{get_parameter_or("pitch_coulomb_ff_gain", 0.0)};
    const double pitch_coulomb_ff_tanh_gain_{get_parameter_or("pitch_coulomb_ff_tanh_gain", 100.0)};
    const double pitch_gravity_ff_gain_{get_parameter_or("pitch_gravity_ff_gain", 0.0)};
    const double pitch_gravity_ff_phase_{get_parameter_or("pitch_gravity_ff_phase", 0.0)};

    struct Input {
        explicit Input(rmcs_executor::Component& component) {
            component.register_input("/remote/joystick/left", joystick_left);
            component.register_input("/remote/switch/right", switch_right);
            component.register_input("/remote/switch/left", switch_left);
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
            component.register_input("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu);

            component.register_input(
                "/auto_aim/control_direction", auto_aim_control_direction, false);
            component.register_input("/auto_aim/yaw_rate", auto_aim_yaw_rate, false);
            component.register_input("/auto_aim/pitch_rate", auto_aim_pitch_rate, false);
            component.register_input("/auto_aim/yaw_acc", auto_aim_yaw_acc, false);
            component.register_input("/auto_aim/pitch_acc", auto_aim_pitch_acc, false);
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
        InputInterface<double> chassis_yaw_velocity_imu;

        InputInterface<Eigen::Vector3d> auto_aim_control_direction;
        InputInterface<Eigen::Vector3d> auto_aim_robot_center;
        InputInterface<double> auto_aim_yaw_rate;
        InputInterface<double> auto_aim_pitch_rate;
        InputInterface<double> auto_aim_yaw_acc;
        InputInterface<double> auto_aim_pitch_acc;

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

            component.register_output(
                "/gimbal/yaw/control_angle_error", yaw_control_angle_error, kNaN);
            component.register_output("/gimbal/yaw/angle", yaw_angle, 0.0);
            component.register_output("/gimbal/yaw/velocity", yaw_velocity, 0.0);
        }

        OutputInterface<double> top_yaw_control_torque;
        OutputInterface<double> bottom_yaw_control_torque;
        OutputInterface<double> pitch_control_torque;

        OutputInterface<double> yaw_control_angle_error;
        OutputInterface<double> yaw_angle;
        OutputInterface<double> yaw_velocity;
    } output_{*this};

    pid::PidCalculator top_yaw_angle_pid_{pid::make_pid_calculator(*this, "top_yaw_angle_")};
    pid::PidCalculator top_yaw_velocity_pid_{pid::make_pid_calculator(*this, "top_yaw_velocity_")};
    pid::PidCalculator bottom_yaw_angle_pid_{pid::make_pid_calculator(*this, "bottom_yaw_angle_")};
    pid::PidCalculator bottom_yaw_velocity_pid_{
        pid::make_pid_calculator(*this, "bottom_yaw_velocity_")};
    pid::PidCalculator pitch_angle_pid_{pid::make_pid_calculator(*this, "pitch_angle_")};
    pid::PidCalculator pitch_velocity_pid_{pid::make_pid_calculator(*this, "pitch_velocity_")};

    double stored_bottom_yaw_target_ = 0.0;
    double stored_pitch_target_ = 0.0;
    double previous_actual_yaw_ = 0.0;
    std::chrono::steady_clock::time_point previous_yaw_timestamp_{};

    static constexpr auto limit_rad(double angle) -> double {
        constexpr double kPi = std::numbers::pi_v<double>;
        while (angle > kPi)
            angle -= 2.0 * kPi;
        while (angle <= -kPi)
            angle += 2.0 * kPi;
        return angle;
    }

    auto reset_all_controls() -> void {
        top_yaw_angle_pid_.reset();
        top_yaw_velocity_pid_.reset();
        bottom_yaw_angle_pid_.reset();
        bottom_yaw_velocity_pid_.reset();
        pitch_angle_pid_.reset();
        pitch_velocity_pid_.reset();

        *output_.top_yaw_control_torque = kNaN;
        *output_.bottom_yaw_control_torque = kNaN;
        *output_.pitch_control_torque = kNaN;
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

    auto apply_control(double bottom_yaw_error, double top_yaw_error, double pitch_error) -> void {

        constexpr auto friction_feedforward = [](double viscous_gain, double coulomb_gain,
                                                 double tanh_gain, double velocity) -> double {
            return (viscous_gain * velocity) + (coulomb_gain * std::tanh(tanh_gain * velocity));
        };

        const auto current_bottom_velocity =
            *input_.bottom_yaw_velocity + *input_.chassis_yaw_velocity_imu;
        const auto current_pitch_angle = limit_rad(*input_.pitch_angle);

        const auto bottom_velocity_ref = bottom_yaw_angle_pid_.update(bottom_yaw_error);
        const auto top_velocity_ref = top_yaw_angle_pid_.update(top_yaw_error);
        const auto pitch_velocity_ref = pitch_angle_pid_.update(pitch_error);

        const auto top_yaw_continuous_torque_ff = friction_feedforward(
            top_yaw_viscous_ff_gain_, top_yaw_coulomb_ff_gain_, top_yaw_coulomb_ff_tanh_gain_,
            top_velocity_ref);

        const auto bottom_world_velocity_ff = *input_.chassis_yaw_velocity_imu;
        const auto bottom_yaw_torque_ff =
            friction_feedforward(
                bottom_yaw_viscous_ff_gain_, bottom_yaw_coulomb_ff_gain_,
                bottom_yaw_coulomb_ff_tanh_gain_, bottom_world_velocity_ff)
            - k_top_to_bottom_ * top_yaw_continuous_torque_ff;

        const auto top_yaw_torque_ff = top_yaw_continuous_torque_ff;
        const auto pitch_torque_ff =
            friction_feedforward(
                pitch_viscous_ff_gain_, pitch_coulomb_ff_gain_, pitch_coulomb_ff_tanh_gain_,
                pitch_velocity_ref)
            + pitch_gravity_ff_gain_ * std::sin(current_pitch_angle - pitch_gravity_ff_phase_);

        *output_.top_yaw_control_torque =
            top_yaw_velocity_pid_.update(top_velocity_ref - *input_.top_yaw_velocity)
            + top_yaw_torque_ff;
        const auto bottom_velocity_pid_output =
            bottom_yaw_velocity_pid_.update(bottom_velocity_ref - current_bottom_velocity);

        // External sliding friction: constant offset in the direction of pre-compensation torque
        const auto bottom_yaw_pre_friction_torque =
            bottom_velocity_pid_output + bottom_yaw_torque_ff;
        const auto bottom_ext_friction =
            (bottom_yaw_pre_friction_torque > 0.0)    ? bottom_yaw_ext_friction_gain_
            : (bottom_yaw_pre_friction_torque < -0.0) ? -bottom_yaw_ext_friction_gain_
                                                      : 0.0;
        *output_.bottom_yaw_control_torque = bottom_yaw_pre_friction_torque + bottom_ext_friction;
        *output_.pitch_control_torque =
            pitch_velocity_pid_.update(pitch_velocity_ref - *input_.pitch_velocity)
            + pitch_torque_ff;

        *output_.yaw_control_angle_error = bottom_yaw_error;
    }
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::EccentricDualYaw, rmcs_executor::Component)
