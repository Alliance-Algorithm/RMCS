#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_msgs/target_snapshot.hpp>

#include "controller/gimbal/planner.hpp"
#include "controller/gimbal/two_axis_gimbal_solver.hpp"
#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::gimbal {

using namespace rmcs_description;

namespace {

constexpr double kDegToRad = 1.0 / 57.3;
constexpr double kControlDt = Planner::kDt;

double limit_rad(double angle) {
    constexpr double kPi = std::numbers::pi_v<double>;
    while (angle > kPi)
        angle -= 2.0 * kPi;
    while (angle <= -kPi)
        angle += 2.0 * kPi;
    return angle;
}

double wrap_relative_angle(double angle) { return limit_rad(angle); }

double clamp_pitch(double pitch, double upper_limit, double lower_limit) {
    return std::clamp(pitch, upper_limit, lower_limit);
}

std::pair<double, double> yaw_pitch_from_direction(const Eigen::Vector3d& direction) {
    const double xy_norm = std::hypot(direction.x(), direction.y());
    return {std::atan2(direction.y(), direction.x()), std::atan2(-direction.z(), xy_norm)};
}

Eigen::Vector3d direction_from_yaw_pitch(double yaw, double pitch) {
    Eigen::Vector3d direction{
        std::cos(pitch) * std::cos(yaw),
        std::cos(pitch) * std::sin(yaw),
        -std::sin(pitch),
    };
    if (direction.norm() > 1e-9)
        direction.normalize();
    else
        direction = Eigen::Vector3d::UnitX();
    return direction;
}

double parameter_or_declare(rclcpp::Node& node, const std::string& name, double default_value) {
    if (!node.has_parameter(name))
        node.declare_parameter<double>(name, default_value);
    return node.get_parameter(name).as_double();
}

double friction_feedforward(
    double viscous_gain, double coulomb_gain, double tanh_gain, double velocity) {
    return (viscous_gain * velocity) + (coulomb_gain * std::tanh(tanh_gain * velocity));
}

PlannerConfig load_planner_config(rclcpp::Node& node) {
    PlannerConfig config;
    config.yaw_offset = parameter_or_declare(node, "yaw_offset", 0.0) * kDegToRad;
    config.pitch_offset = parameter_or_declare(node, "pitch_offset", 0.0) * kDegToRad;
    config.fire_thresh = parameter_or_declare(node, "fire_thresh", 0.0035);
    config.low_speed_delay_time = parameter_or_declare(node, "low_speed_delay_time", 0.0);
    config.high_speed_delay_time = parameter_or_declare(node, "high_speed_delay_time", 0.0);
    config.decision_speed = parameter_or_declare(node, "decision_speed", 7.0);
    config.max_yaw_acc = parameter_or_declare(node, "max_yaw_acc", 400.0);
    config.max_pitch_acc = parameter_or_declare(node, "max_pitch_acc", 100.0);
    return config;
}

struct AimResult {
    bool valid = false;
    double yaw = 0.0;
    double pitch = 0.0;
    double fly_time = 0.0;
};

struct AxisCommand {
    double target = 0.0;
    double velocity_ff = 0.0;
    double acceleration_ff = 0.0;
};

struct AutoAimPlan {
    bool fire = false;
    AxisCommand bottom_yaw;
    AxisCommand top_yaw;
    AxisCommand pitch;
    Eigen::Vector3d upper_control_direction = Eigen::Vector3d::Zero();
};

struct ControlTarget {
    AxisCommand bottom_yaw;
    AxisCommand top_yaw;
    AxisCommand pitch;
    std::optional<Eigen::Vector3d> upper_control_direction = std::nullopt;
};

} // namespace

class SentryGimbalController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    enum class Mode {
        kDisabled,
        kManual,
        kAutoAim,
    };

    SentryGimbalController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , upper_limit_(get_parameter("upper_limit").as_double())
        , lower_limit_(get_parameter("lower_limit").as_double())
        , planner_config_(load_planner_config(*this))
        , rotation_planner_(planner_config_)
        , upper_gimbal_solver_(*this, upper_limit_, lower_limit_)
        , top_yaw_angle_pid_(pid::make_pid_calculator(*this, "top_yaw_angle_"))
        , top_yaw_velocity_pid_(pid::make_pid_calculator(*this, "top_yaw_velocity_"))
        , bottom_yaw_angle_pid_(pid::make_pid_calculator(*this, "bottom_yaw_angle_"))
        , bottom_yaw_velocity_pid_(pid::make_pid_calculator(*this, "bottom_yaw_velocity_"))
        , pitch_angle_pid_(pid::make_pid_calculator(*this, "pitch_angle_"))
        , pitch_velocity_pid_(pid::make_pid_calculator(*this, "pitch_velocity_"))
        , bottom_yaw_vel_ff_gain_(parameter_or_declare(*this, "bottom_yaw_vel_ff_gain", 1.0))
        , bottom_yaw_acc_ff_gain_(parameter_or_declare(*this, "bottom_yaw_acc_ff_gain", 0.0))
        , bottom_yaw_viscous_ff_gain_(
              parameter_or_declare(*this, "bottom_yaw_viscous_ff_gain", 0.0))
        , bottom_yaw_coulomb_ff_gain_(
              parameter_or_declare(*this, "bottom_yaw_coulomb_ff_gain", 0.0))
        , bottom_yaw_coulomb_ff_tanh_gain_(
              parameter_or_declare(*this, "bottom_yaw_coulomb_ff_tanh_gain", 100.0))
        , k_top_to_bottom_(parameter_or_declare(*this, "k_top_to_bottom", 0.0))
        , top_yaw_vel_ff_gain_(parameter_or_declare(*this, "top_yaw_vel_ff_gain", 1.0))
        , top_yaw_acc_ff_gain_(parameter_or_declare(*this, "top_yaw_acc_ff_gain", 0.0))
        , pitch_vel_ff_gain_(parameter_or_declare(*this, "pitch_vel_ff_gain", 0.0))
        , top_yaw_viscous_ff_gain_(parameter_or_declare(*this, "top_yaw_viscous_ff_gain", 0.0))
        , top_yaw_coulomb_ff_gain_(parameter_or_declare(*this, "top_yaw_coulomb_ff_gain", 0.0))
        , top_yaw_coulomb_ff_tanh_gain_(
              parameter_or_declare(*this, "top_yaw_coulomb_ff_tanh_gain", 100.0))
        , pitch_acc_ff_gain_(parameter_or_declare(*this, "pitch_acc_ff_gain", 0.0))
        , pitch_viscous_ff_gain_(parameter_or_declare(*this, "pitch_viscous_ff_gain", 0.0))
        , pitch_coulomb_ff_gain_(parameter_or_declare(*this, "pitch_coulomb_ff_gain", 0.0))
        , pitch_coulomb_ff_tanh_gain_(
              parameter_or_declare(*this, "pitch_coulomb_ff_tanh_gain", 100.0))
        , pitch_gravity_ff_gain_(parameter_or_declare(*this, "pitch_gravity_ff_gain", 0.0))
        , pitch_gravity_ff_phase_(parameter_or_declare(*this, "pitch_gravity_ff_phase", 0.0))
        , result_timeout_(
              std::chrono::duration<double>(parameter_or_declare(*this, "result_timeout", 0.2)))
        , bullet_speed_fallback_storage_(
              static_cast<float>(parameter_or_declare(*this, "bullet_speed_fallback", 23.0))) {

        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);

        register_input("/predefined/timestamp", timestamp_);
        register_input("/tf", tf_);

        register_input("/gimbal/top_yaw/angle", top_yaw_angle_);
        register_input("/gimbal/top_yaw/velocity", top_yaw_velocity_);
        register_input("/gimbal/bottom_yaw/angle", bottom_yaw_angle_);
        register_input("/gimbal/bottom_yaw/velocity", bottom_yaw_velocity_);
        register_input("/gimbal/pitch/angle", pitch_angle_);
        register_input("/gimbal/pitch/velocity", pitch_velocity_);
        register_input("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_);

        register_input("/referee/shooter/initial_speed", bullet_speed_, false);
        register_input("/gimbal/auto_aim/target_snapshot", target_snapshot_, false);

        register_output("/gimbal/top_yaw/control_torque", top_yaw_control_torque_, nan_);
        register_output("/gimbal/bottom_yaw/control_torque", bottom_yaw_control_torque_, nan_);
        register_output("/gimbal/pitch/control_torque", pitch_control_torque_, nan_);

        register_output("/gimbal/yaw/control_angle_error", yaw_control_angle_error_, nan_);
        register_output("/gimbal/yaw/angle", yaw_angle_, 0.0);
        register_output("/gimbal/yaw/velocity", yaw_velocity_, 0.0);

        register_output(
            "/gimbal/auto_aim/control_direction", control_direction_, Eigen::Vector3d::Zero());
        register_output("/gimbal/auto_aim/fire_control", fire_control_, false);
    }

    void before_updating() override {
        if (!bullet_speed_.ready())
            bullet_speed_.bind_directly(bullet_speed_fallback_storage_);
        if (!target_snapshot_.ready())
            target_snapshot_.make_and_bind_directly(rmcs_msgs::TargetSnapshot{});

        enter_disabled_state();
        previous_actual_yaw_ = current_barrel_yaw_pitch().first;
        previous_yaw_timestamp_ = *timestamp_;
        mode_ = Mode::kDisabled;
    }

    void update() override {
        const auto actual_yaw_pitch = current_barrel_yaw_pitch();
        *yaw_angle_ = *bottom_yaw_angle_;
        *yaw_velocity_ = compute_actual_yaw_velocity(actual_yaw_pitch.first);

        const Mode requested_mode = select_requested_mode();
        std::optional<AutoAimPlan> auto_plan;
        Mode next_mode = requested_mode;
        if (requested_mode == Mode::kAutoAim) {
            auto_plan = make_auto_aim_plan();
            if (!auto_plan.has_value())
                next_mode = Mode::kManual;
        }

        if (next_mode != mode_) {
            on_mode_changed(next_mode);
            mode_ = next_mode;
        }

        if (mode_ == Mode::kDisabled) {
            enter_disabled_state();
            return;
        }

        if (mode_ == Mode::kAutoAim) {
            *control_direction_ = auto_plan->upper_control_direction;
            *fire_control_ = auto_plan->fire;
            apply_control(
                ControlTarget{
                    .bottom_yaw = auto_plan->bottom_yaw,
                    .top_yaw = auto_plan->top_yaw,
                    .pitch = auto_plan->pitch,
                    .upper_control_direction = auto_plan->upper_control_direction,
                });
            return;
        }

        clear_auto_outputs();
        apply_control(update_manual_target());
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double joystick_sensitivity_ = 0.006;
    static constexpr double mouse_sensitivity_ = 0.5;

    bool auto_aim_requested() const {
        return mouse_->right || *switch_right_ == rmcs_msgs::Switch::UP;
    }

    Mode select_requested_mode() const {
        using rmcs_msgs::Switch;
        if ((*switch_left_ == Switch::UNKNOWN || *switch_right_ == Switch::UNKNOWN)
            || (*switch_left_ == Switch::DOWN && *switch_right_ == Switch::DOWN)) {
            return Mode::kDisabled;
        }
        return auto_aim_requested() ? Mode::kAutoAim : Mode::kManual;
    }

    void on_mode_changed(Mode next_mode) {
        reset_all_controls();
        if (next_mode == Mode::kManual)
            sync_manual_targets_to_feedback();
    }

    void sync_manual_targets_to_feedback() {
        manual_bottom_yaw_target_ = current_bottom_world_yaw();
        manual_pitch_target_ =
            clamp_pitch(wrap_relative_angle(*pitch_angle_), upper_limit_, lower_limit_);
    }

    void reset_all_controls() {
        upper_gimbal_solver_.update(TwoAxisGimbalSolver::SetDisabled{});
        top_yaw_angle_pid_.reset();
        top_yaw_velocity_pid_.reset();
        bottom_yaw_angle_pid_.reset();
        bottom_yaw_velocity_pid_.reset();
        pitch_angle_pid_.reset();
        pitch_velocity_pid_.reset();

        *top_yaw_control_torque_ = nan_;
        *bottom_yaw_control_torque_ = nan_;
        *pitch_control_torque_ = nan_;
    }

    void clear_auto_outputs() {
        *control_direction_ = Eigen::Vector3d::Zero();
        *fire_control_ = false;
    }

    void enter_disabled_state() {
        clear_auto_outputs();
        reset_all_controls();
        sync_manual_targets_to_feedback();
        *yaw_control_angle_error_ = nan_;
    }

    double compute_actual_yaw_velocity(double actual_yaw) {
        const auto now = *timestamp_;
        const double dt = std::chrono::duration<double>(now - previous_yaw_timestamp_).count();
        double velocity = 0.0;
        if (dt > 1e-6)
            velocity = limit_rad(actual_yaw - previous_actual_yaw_) / dt;
        previous_actual_yaw_ = actual_yaw;
        previous_yaw_timestamp_ = now;
        return velocity;
    }

    std::pair<double, double> current_barrel_yaw_pitch() const {
        auto direction = fast_tf::cast<OdomGimbalImu>(
            PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, *tf_);
        Eigen::Vector3d vector = *direction;
        if (vector.norm() > 1e-9)
            vector.normalize();
        else
            vector = Eigen::Vector3d::UnitX();
        return yaw_pitch_from_direction(vector);
    }

    double current_bottom_world_yaw() const {
        auto direction = fast_tf::cast<OdomGimbalImu>(
            BottomYawLink::DirectionVector{Eigen::Vector3d::UnitX()}, *tf_);
        Eigen::Vector3d vector = *direction;
        vector.z() = 0.0;
        if (vector.norm() > 1e-9)
            vector.normalize();
        else
            vector = Eigen::Vector3d::UnitX();
        return std::atan2(vector.y(), vector.x());
    }

    double current_bottom_motor_yaw_velocity() const { return *bottom_yaw_velocity_; }

    double current_bottom_world_yaw_velocity() const {
        // Reconstruct bottom yaw world rate from the collocated motor speed and chassis yaw rate
        // instead of the delayed top IMU yaw rate.
        return current_bottom_motor_yaw_velocity() + *chassis_yaw_velocity_imu_;
    }

    Eigen::Vector3d bottom_to_top_offset() const {
        return fast_tf::lookup_transform<BottomYawLink, TopYawLink>(*tf_).translation();
    }

    TwoAxisGimbalSolver::AngleError
        upper_angle_error(const Eigen::Vector3d& control_direction) {
        return upper_gimbal_solver_.update(TwoAxisGimbalSolver::SetControlDirection{
            fast_tf::cast<OdomImu>(OdomGimbalImu::DirectionVector{control_direction}, *tf_)});
    }

    ControlTarget update_manual_target() {
        const double yaw_shift =
            joystick_sensitivity_ * joystick_left_->y() + mouse_sensitivity_ * mouse_velocity_->y();
        const double pitch_shift = -joystick_sensitivity_ * joystick_left_->x()
                                 - mouse_sensitivity_ * mouse_velocity_->x();

        manual_bottom_yaw_target_ = limit_rad(manual_bottom_yaw_target_ + yaw_shift);
        manual_pitch_target_ =
            clamp_pitch(manual_pitch_target_ + pitch_shift, upper_limit_, lower_limit_);

        return {
            .bottom_yaw =
                AxisCommand{
                    .target = manual_bottom_yaw_target_,
                },
            .top_yaw =
                AxisCommand{
                    .target = 0.0,
                },
            .pitch =
                AxisCommand{
                    .target = manual_pitch_target_,
                },
        };
    }

    std::optional<AutoAimPlan> make_auto_aim_plan() {
        const auto snapshot = *target_snapshot_;
        const auto now = *timestamp_;
        if (!snapshot.valid
            || std::chrono::duration<double>(now - snapshot.timestamp) > result_timeout_)
            return std::nullopt;

        auto rotation_snapshot = snapshot;
        rotation_snapshot.state[1] = 0.0;
        rotation_snapshot.state[3] = 0.0;
        rotation_snapshot.state[5] = 0.0;

        double bullet_speed = static_cast<double>(*bullet_speed_);
        if (bullet_speed < 10.0 || bullet_speed > 25.0)
            bullet_speed = bullet_speed_fallback_storage_;

        const PlannerResult rotation_plan = rotation_planner_.plan(
            std::optional<rmcs_msgs::TargetSnapshot>{rotation_snapshot}, now, bullet_speed);
        if (!rotation_plan.control)
            return std::nullopt;

        const double planning_delay = std::abs(snapshot.state[7]) > planner_config_.decision_speed
                                        ? planner_config_.high_speed_delay_time
                                        : planner_config_.low_speed_delay_time;
        const auto bullet_traj = detail::solve_ballistic(
            bullet_speed, rotation_plan.control_xyza.head<2>().norm(),
            rotation_plan.control_xyza.z());
        if (bullet_traj.unsolvable)
            return std::nullopt;

        const double relative_horizon = planning_delay + bullet_traj.fly_time;
        const Eigen::Vector3d center_now{
            snapshot.state[0],
            snapshot.state[2],
            snapshot.state[4],
        };
        const Eigen::Vector3d center_velocity{
            snapshot.state[1],
            snapshot.state[3],
            snapshot.state[5],
        };
        // RCLCPP_INFO(
        //     get_logger(), "%f %f %f, %f %f %f", center_now.x(), center_now.y(), center_now.z(),
        //     center_velocity.x(), center_velocity.y(), center_velocity.z());
        const double snapshot_age = std::chrono::duration<double>(now - snapshot.timestamp).count();
        const Eigen::Vector3d dynamic_center =
            center_now + center_velocity * (snapshot_age + relative_horizon);
        const Eigen::Vector3d residual_point = rotation_plan.control_xyza.head<3>() - center_now;
        const Eigen::Vector3d total_point = dynamic_center + residual_point;

        const auto half_step = std::chrono::microseconds(static_cast<int64_t>(kControlDt * 5e5));
        const auto center_aim_prev = aim_center(snapshot, now - half_step, relative_horizon);
        const auto center_aim_now = aim_center(snapshot, now, relative_horizon);
        const auto center_aim_next = aim_center(snapshot, now + half_step, relative_horizon);
        const auto total_aim_prev = aim_point(
            dynamic_center_at(snapshot, now - half_step, relative_horizon) + residual_point,
            bullet_speed);
        const auto total_aim_now = aim_point(total_point, bullet_speed);
        const auto total_aim_next = aim_point(
            dynamic_center_at(snapshot, now + half_step, relative_horizon) + residual_point,
            bullet_speed);
        if (!center_aim_prev.valid || !center_aim_now.valid || !center_aim_next.valid
            || !total_aim_prev.valid || !total_aim_now.valid || !total_aim_next.valid) {
            return std::nullopt;
        }

        const double bottom_yaw_target = center_aim_now.yaw;
        const double bottom_yaw_velocity_ff =
            limit_rad(center_aim_next.yaw - center_aim_prev.yaw) / kControlDt;
        const double bottom_yaw_acceleration_ff =
            (limit_rad(center_aim_next.yaw - center_aim_now.yaw)
             - limit_rad(center_aim_now.yaw - center_aim_prev.yaw))
            / (0.25 * kControlDt * kControlDt);

        const Eigen::Vector3d total_point_in_bottom_frame =
            Eigen::AngleAxisd{-bottom_yaw_target, Eigen::Vector3d::UnitZ()} * total_point
            - bottom_to_top_offset();
        const double top_yaw_target =
            std::atan2(total_point_in_bottom_frame.y(), total_point_in_bottom_frame.x());

        AutoAimPlan plan;
        plan.fire = rotation_plan.fire;
        const double desired_pitch = clamp_pitch(total_aim_now.pitch, upper_limit_, lower_limit_);
        plan.bottom_yaw = AxisCommand{
            .target = bottom_yaw_target,
            .velocity_ff = bottom_yaw_vel_ff_gain_ * bottom_yaw_velocity_ff,
            .acceleration_ff = bottom_yaw_acc_ff_gain_ * bottom_yaw_acceleration_ff,
        };
        plan.top_yaw = AxisCommand{
            .target = top_yaw_target,
            .velocity_ff = top_yaw_vel_ff_gain_ * rotation_plan.yaw_velocity,
            .acceleration_ff = top_yaw_acc_ff_gain_ * rotation_plan.yaw_acceleration,
        };
        plan.pitch = AxisCommand{
            .target = desired_pitch,
            .velocity_ff =
                pitch_vel_ff_gain_ * ((total_aim_next.pitch - total_aim_prev.pitch) / kControlDt),
            .acceleration_ff = pitch_acc_ff_gain_
                             * ((total_aim_next.pitch - total_aim_now.pitch)
                                - (total_aim_now.pitch - total_aim_prev.pitch))
                             / (0.25 * kControlDt * kControlDt),
        };
        plan.upper_control_direction = direction_from_yaw_pitch(
            limit_rad(plan.bottom_yaw.target + plan.top_yaw.target), plan.pitch.target);
        return std::optional<AutoAimPlan>{plan};
    }

    AimResult aim_center(
        const rmcs_msgs::TargetSnapshot& snapshot, const std::chrono::steady_clock::time_point& now,
        double relative_horizon) const {
        return aim_point(
            dynamic_center_at(snapshot, now, relative_horizon),
            static_cast<double>(*bullet_speed_));
    }

    Eigen::Vector3d dynamic_center_at(
        const rmcs_msgs::TargetSnapshot& snapshot, const std::chrono::steady_clock::time_point& now,
        double relative_horizon) const {
        const double total_dt =
            std::chrono::duration<double>(now - snapshot.timestamp).count() + relative_horizon;
        return {
            snapshot.state[0] + total_dt * snapshot.state[1],
            snapshot.state[2] + total_dt * snapshot.state[3],
            snapshot.state[4] + total_dt * snapshot.state[5],
        };
    }

    AimResult aim_point(const Eigen::Vector3d& point, double bullet_speed) const {
        AimResult result;
        if (bullet_speed < 10.0 || bullet_speed > 25.0)
            bullet_speed = static_cast<double>(bullet_speed_fallback_storage_);

        const auto bullet_traj =
            detail::solve_ballistic(bullet_speed, point.head<2>().norm(), point.z());
        if (bullet_traj.unsolvable)
            return result;

        result.valid = true;
        result.yaw = limit_rad(std::atan2(point.y(), point.x()) + planner_config_.yaw_offset);
        result.pitch = -bullet_traj.pitch - planner_config_.pitch_offset;
        result.fly_time = bullet_traj.fly_time;
        return result;
    }

    void apply_control(const ControlTarget& target) {
        const double current_bottom_angle = current_bottom_world_yaw();
        const double current_bottom_velocity = current_bottom_world_yaw_velocity();
        const double current_top_angle = wrap_relative_angle(*top_yaw_angle_);
        const double current_pitch_angle = wrap_relative_angle(*pitch_angle_);

        const double bottom_yaw_error = limit_rad(target.bottom_yaw.target - current_bottom_angle);
        double top_yaw_error = limit_rad(target.top_yaw.target - current_top_angle);
        double pitch_error = limit_rad(target.pitch.target - current_pitch_angle);
        if (target.upper_control_direction.has_value()) {
            const auto angle_error = upper_angle_error(*target.upper_control_direction);
            if (std::isfinite(angle_error.yaw_angle_error)
                && std::isfinite(angle_error.pitch_angle_error)) {
                top_yaw_error = angle_error.yaw_angle_error;
                pitch_error = angle_error.pitch_angle_error;
            } else {
                upper_gimbal_solver_.update(TwoAxisGimbalSolver::SetDisabled{});
            }
        } else {
            upper_gimbal_solver_.update(TwoAxisGimbalSolver::SetDisabled{});
        }

        const double bottom_velocity_ref =
            bottom_yaw_angle_pid_.update(bottom_yaw_error) + target.bottom_yaw.velocity_ff;
        const double top_velocity_ref =
            top_yaw_angle_pid_.update(top_yaw_error) + target.top_yaw.velocity_ff;
        const double pitch_velocity_ref =
            pitch_angle_pid_.update(pitch_error) + target.pitch.velocity_ff;
        const double bottom_world_velocity_ff =
            target.bottom_yaw.velocity_ff + *chassis_yaw_velocity_imu_;
        const double top_yaw_continuous_torque_ff =
            target.top_yaw.acceleration_ff + top_yaw_viscous_ff_gain_ * target.top_yaw.velocity_ff;
        const double bottom_yaw_torque_ff =
            target.bottom_yaw.acceleration_ff
            + friction_feedforward(
                bottom_yaw_viscous_ff_gain_, bottom_yaw_coulomb_ff_gain_,
                bottom_yaw_coulomb_ff_tanh_gain_, bottom_world_velocity_ff)
            - k_top_to_bottom_ * top_yaw_continuous_torque_ff;

        const double top_yaw_torque_ff = target.top_yaw.acceleration_ff
                                       + friction_feedforward(
                                             top_yaw_viscous_ff_gain_, top_yaw_coulomb_ff_gain_,
                                             top_yaw_coulomb_ff_tanh_gain_, top_velocity_ref);
        const double pitch_torque_ff =
            target.pitch.acceleration_ff
            + friction_feedforward(
                pitch_viscous_ff_gain_, pitch_coulomb_ff_gain_, pitch_coulomb_ff_tanh_gain_,
                pitch_velocity_ref)
            + pitch_gravity_ff_gain_ * std::sin(current_pitch_angle - pitch_gravity_ff_phase_);

        *bottom_yaw_control_torque_ =
            bottom_yaw_velocity_pid_.update(bottom_velocity_ref - current_bottom_velocity)
            + bottom_yaw_torque_ff;
        *top_yaw_control_torque_ =
            top_yaw_velocity_pid_.update(top_velocity_ref - *top_yaw_velocity_) + top_yaw_torque_ff;
        *pitch_control_torque_ =
            pitch_velocity_pid_.update(pitch_velocity_ref - *pitch_velocity_) + pitch_torque_ff;
        *yaw_control_angle_error_ = bottom_yaw_error;
    }

    const double upper_limit_;
    const double lower_limit_;
    const PlannerConfig planner_config_;
    Planner rotation_planner_;
    TwoAxisGimbalSolver upper_gimbal_solver_;

    pid::PidCalculator top_yaw_angle_pid_;
    pid::PidCalculator top_yaw_velocity_pid_;
    pid::PidCalculator bottom_yaw_angle_pid_;
    pid::PidCalculator bottom_yaw_velocity_pid_;
    pid::PidCalculator pitch_angle_pid_;
    pid::PidCalculator pitch_velocity_pid_;

    const double bottom_yaw_vel_ff_gain_;
    const double bottom_yaw_acc_ff_gain_;
    const double bottom_yaw_viscous_ff_gain_;
    const double bottom_yaw_coulomb_ff_gain_;
    const double bottom_yaw_coulomb_ff_tanh_gain_;
    const double k_top_to_bottom_;
    const double top_yaw_vel_ff_gain_;
    const double top_yaw_acc_ff_gain_;
    const double pitch_vel_ff_gain_;
    const double top_yaw_viscous_ff_gain_;
    const double top_yaw_coulomb_ff_gain_;
    const double top_yaw_coulomb_ff_tanh_gain_;
    const double pitch_acc_ff_gain_;
    const double pitch_viscous_ff_gain_;
    const double pitch_coulomb_ff_gain_;
    const double pitch_coulomb_ff_tanh_gain_;
    const double pitch_gravity_ff_gain_;
    const double pitch_gravity_ff_phase_;

    std::chrono::duration<double> result_timeout_;
    float bullet_speed_fallback_storage_;

    Mode mode_ = Mode::kDisabled;
    double manual_bottom_yaw_target_ = 0.0;
    double manual_pitch_target_ = 0.0;
    double previous_actual_yaw_ = 0.0;
    std::chrono::steady_clock::time_point previous_yaw_timestamp_{};

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;

    InputInterface<std::chrono::steady_clock::time_point> timestamp_;
    InputInterface<Tf> tf_;

    InputInterface<double> top_yaw_angle_;
    InputInterface<double> top_yaw_velocity_;
    InputInterface<double> bottom_yaw_angle_;
    InputInterface<double> bottom_yaw_velocity_;
    InputInterface<double> pitch_angle_;
    InputInterface<double> pitch_velocity_;
    InputInterface<double> chassis_yaw_velocity_imu_;

    InputInterface<float> bullet_speed_;
    InputInterface<rmcs_msgs::TargetSnapshot> target_snapshot_;

    OutputInterface<double> top_yaw_control_torque_;
    OutputInterface<double> bottom_yaw_control_torque_;
    OutputInterface<double> pitch_control_torque_;

    OutputInterface<double> yaw_control_angle_error_;
    OutputInterface<double> yaw_angle_;
    OutputInterface<double> yaw_velocity_;

    OutputInterface<Eigen::Vector3d> control_direction_;
    OutputInterface<bool> fire_control_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::SentryGimbalController, rmcs_executor::Component)
