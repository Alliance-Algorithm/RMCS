#include "controller/gimbal/two_axis_gimbal_solver.hpp"
#include "controller/pid/pid_calculator.hpp"

#include <cmath>
#include <limits>
#include <string>

#include <eigen3/Eigen/Geometry>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::gimbal {

using namespace rmcs_description;

class DeformableInfantryGimbalController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DeformableInfantryGimbalController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        configure_pid("yaw_angle", yaw_angle_pid_);
        configure_pid("yaw_velocity", yaw_velocity_pid_);
        configure_pid("pitch_angle", pitch_angle_pid_);
        configure_pid("pitch_velocity", pitch_velocity_pid_);
        get_parameter("pitch_torque_control", pitch_torque_control_enabled_);
        get_parameter("manual_joystick_sensitivity", joystick_sensitivity_);
        get_parameter("manual_mouse_sensitivity", mouse_sensitivity_);
        get_parameter("yaw_vel_ff_gain", yaw_vel_ff_gain_);
        get_parameter("yaw_acc_ff_gain", yaw_acc_ff_gain_);
        get_parameter("pitch_acc_ff_gain", pitch_acc_ff_gain_);
        get_parameter_or("ctrl_hold_pitch_target_angle", ctrl_hold_pitch_target_angle_, 0.0);
        get_parameter_or(
            "ctrl_hold_chassis_yaw_velocity_max", ctrl_hold_chassis_yaw_velocity_max_, 30.0);
    }

    auto update() -> void override {
        const auto switch_right = *input_.switch_right;
        const auto switch_left = *input_.switch_left;

        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            suspension_on_by_switch_ = false;
            last_switch_right_ = switch_right;
            reset_all_controls();
            return;
        }

        update_ctrl_hold_request_state(switch_left, switch_right);

        if (ctrl_hold_requested()) {
            update_ctrl_hold_control();
            return;
        }

        deactivate_ctrl_hold();

        const auto auto_aim_active = auto_aim_requested() && input_.auto_aim_should_control.ready()
                                  && *input_.auto_aim_should_control
                                  && input_.auto_aim_control_direction.ready()
                                  && input_.auto_aim_control_direction->allFinite()
                                  && !input_.auto_aim_control_direction->isZero();
        const auto angle_error =
            auto_aim_active ? update_auto_aim_control() : update_manual_control();

        *output_.yaw_angle_error = angle_error.yaw_angle_error;
        *output_.pitch_angle_error = angle_error.pitch_angle_error;

        if (!std::isfinite(angle_error.yaw_angle_error)
            || !std::isfinite(angle_error.pitch_angle_error)) {
            reset_control_outputs();
            return;
        }

        const auto feedforward_enabled = auto_aim_active && input_.auto_aim_feedforward_valid.ready()
                                      && *input_.auto_aim_feedforward_valid;
        const auto yaw_velocity_ff = feedforward_enabled && input_.auto_aim_yaw_rate.ready()
                                          && std::isfinite(*input_.auto_aim_yaw_rate)
                                      ? yaw_vel_ff_gain_ * *input_.auto_aim_yaw_rate
                                      : 0.0;
        const auto yaw_acc_ff = feedforward_enabled && input_.auto_aim_yaw_acc.ready()
                                     && std::isfinite(*input_.auto_aim_yaw_acc)
                                 ? yaw_acc_ff_gain_ * *input_.auto_aim_yaw_acc
                                 : 0.0;
        const auto pitch_acc_ff = feedforward_enabled && input_.auto_aim_pitch_acc.ready()
                                       && std::isfinite(*input_.auto_aim_pitch_acc)
                                   ? pitch_acc_ff_gain_ * *input_.auto_aim_pitch_acc
                                   : 0.0;

        const auto yaw_velocity_ref =
            yaw_angle_pid_.update(angle_error.yaw_angle_error) + yaw_velocity_ff;
        const auto pitch_velocity_ref = pitch_angle_pid_.update(angle_error.pitch_angle_error);

        *output_.yaw_control_torque =
            yaw_velocity_pid_.update(yaw_velocity_ref - *input_.yaw_velocity_imu) + yaw_acc_ff;
        if (pitch_torque_control_enabled_) {
            *output_.pitch_control_velocity = kNaN;
            *output_.pitch_control_torque =
                pitch_velocity_pid_.update(pitch_velocity_ref - *input_.pitch_velocity_imu)
              + pitch_acc_ff;
        } else {
            pitch_velocity_pid_.reset();
            *output_.pitch_control_velocity = pitch_velocity_ref;
            *output_.pitch_control_torque = kNaN;
        }
    }

private:
    static constexpr auto kNaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr auto kDefaultDt = 1e-3;

    auto configure_pid(const std::string& prefix, pid::PidCalculator& calculator) -> void {
        get_parameter(prefix + "_integral_min", calculator.integral_min);
        get_parameter(prefix + "_integral_max", calculator.integral_max);
        get_parameter(prefix + "_integral_split_min", calculator.integral_split_min);
        get_parameter(prefix + "_integral_split_max", calculator.integral_split_max);
        get_parameter(prefix + "_output_min", calculator.output_min);
        get_parameter(prefix + "_output_max", calculator.output_max);
    }

    struct Input {
        explicit Input(rmcs_executor::Component& component) {
            component.register_input("/remote/joystick/left", joystick_left);
            component.register_input("/remote/keyboard", keyboard);
            component.register_input("/remote/switch/right", switch_right);
            component.register_input("/remote/switch/left", switch_left);
            component.register_input("/remote/mouse/velocity", mouse_velocity);
            component.register_input("/remote/mouse", mouse);
            component.register_input("/predefined/update_rate", update_rate, false);

            component.register_input("/tf", tf);
            component.register_input("/gimbal/yaw/angle", yaw_angle);
            component.register_input("/gimbal/yaw/velocity", yaw_velocity);
            component.register_input("/gimbal/pitch/angle", pitch_angle);
            component.register_input("/gimbal/pitch/velocity", pitch_velocity);
            component.register_input("/gimbal/yaw/velocity_imu", yaw_velocity_imu);
            component.register_input("/gimbal/pitch/velocity_imu", pitch_velocity_imu);

            component.register_input("/auto_aim/should_control", auto_aim_should_control, false);
            component.register_input(
                "/auto_aim/control_direction", auto_aim_control_direction, false);
            component.register_input(
                "/auto_aim/feedforward_valid", auto_aim_feedforward_valid, false);
            component.register_input("/auto_aim/yaw_rate", auto_aim_yaw_rate, false);
            component.register_input("/auto_aim/yaw_acc", auto_aim_yaw_acc, false);
            component.register_input("/auto_aim/pitch_acc", auto_aim_pitch_acc, false);
        }

        InputInterface<Eigen::Vector2d> joystick_left;
        InputInterface<rmcs_msgs::Keyboard> keyboard;
        InputInterface<rmcs_msgs::Switch> switch_right;
        InputInterface<rmcs_msgs::Switch> switch_left;
        InputInterface<Eigen::Vector2d> mouse_velocity;
        InputInterface<rmcs_msgs::Mouse> mouse;
        InputInterface<double> update_rate;

        InputInterface<Tf> tf;
        InputInterface<double> yaw_angle;
        InputInterface<double> yaw_velocity;
        InputInterface<double> pitch_angle;
        InputInterface<double> pitch_velocity;
        InputInterface<double> yaw_velocity_imu;
        InputInterface<double> pitch_velocity_imu;

        InputInterface<bool> auto_aim_should_control;
        InputInterface<Eigen::Vector3d> auto_aim_control_direction;
        InputInterface<bool> auto_aim_feedforward_valid;
        InputInterface<double> auto_aim_yaw_rate;
        InputInterface<double> auto_aim_yaw_acc;
        InputInterface<double> auto_aim_pitch_acc;
    } input_{*this};

    struct Output {
        explicit Output(rmcs_executor::Component& component) {
            component.register_output("/gimbal/yaw/control_torque", yaw_control_torque, kNaN);
            component.register_output("/gimbal/yaw/control_angle", yaw_control_angle, kNaN);
            component.register_output(
                "/gimbal/pitch/control_velocity", pitch_control_velocity, kNaN);
            component.register_output("/gimbal/pitch/control_torque", pitch_control_torque, kNaN);
            component.register_output("/gimbal/pitch/control_angle", pitch_control_angle, kNaN);
            component.register_output("/gimbal/yaw/control_angle_error", yaw_angle_error, kNaN);
            component.register_output("/gimbal/pitch/control_angle_error", pitch_angle_error, kNaN);
            component.register_output(
                "/chassis/manual_yaw_velocity_override", chassis_manual_yaw_velocity_override,
                kNaN);
        }

        OutputInterface<double> yaw_control_torque;
        OutputInterface<double> yaw_control_angle;
        OutputInterface<double> pitch_control_velocity;
        OutputInterface<double> pitch_control_torque;
        OutputInterface<double> pitch_control_angle;
        OutputInterface<double> yaw_angle_error;
        OutputInterface<double> pitch_angle_error;
        OutputInterface<double> chassis_manual_yaw_velocity_override;
    } output_{*this};

    auto ctrl_hold_requested() const -> bool {
        return (input_.keyboard.ready() && input_.keyboard->ctrl) || suspension_on_by_switch_;
    }

    auto update_ctrl_hold_request_state(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right) -> void {
        if (switch_left == rmcs_msgs::Switch::DOWN && switch_right == rmcs_msgs::Switch::UP
            && last_switch_right_ == rmcs_msgs::Switch::MIDDLE) {
            suspension_on_by_switch_ = !suspension_on_by_switch_;
        }
        last_switch_right_ = switch_right;
    }

    auto update_dt() const -> double {
        if (input_.update_rate.ready() && std::isfinite(*input_.update_rate) && *input_.update_rate > 1e-6)
            return 1.0 / *input_.update_rate;
        return kDefaultDt;
    }

    auto manual_yaw_shift() const -> double {
        return joystick_sensitivity_ * input_.joystick_left->y()
             + mouse_sensitivity_ * input_.mouse_velocity->y();
    }

    auto auto_aim_requested() const -> bool {
        return input_.mouse->right || *input_.switch_right == rmcs_msgs::Switch::UP;
    }

    auto update_auto_aim_control() -> TwoAxisGimbalSolver::AngleError {
        return gimbal_solver_.update(
            TwoAxisGimbalSolver::SetControlDirection{
                OdomImu::DirectionVector{*input_.auto_aim_control_direction}});
    }

    auto update_manual_control() -> TwoAxisGimbalSolver::AngleError {
        if (!gimbal_solver_.enabled())
            return gimbal_solver_.update(TwoAxisGimbalSolver::SetToLevel{});

        const auto yaw_shift = manual_yaw_shift();

        const auto pitch_shift = -joystick_sensitivity_ * input_.joystick_left->x()
                               + mouse_sensitivity_ * input_.mouse_velocity->x();
        return gimbal_solver_.update(TwoAxisGimbalSolver::SetControlShift{yaw_shift, pitch_shift});
    }

    auto activate_ctrl_hold() -> void {
        ctrl_hold_active_ = true;
        locked_yaw_angle_ = (input_.yaw_angle.ready() && std::isfinite(*input_.yaw_angle))
                              ? *input_.yaw_angle
                              : 0.0;
        yaw_angle_pid_.reset();
        yaw_velocity_pid_.reset();
        pitch_angle_pid_.reset();
        pitch_velocity_pid_.reset();
        gimbal_solver_.update(TwoAxisGimbalSolver::SetDisabled{});
    }

    auto deactivate_ctrl_hold() -> void {
        if (!ctrl_hold_active_)
            return;

        ctrl_hold_active_ = false;
        locked_yaw_angle_ = kNaN;
        yaw_angle_pid_.reset();
        yaw_velocity_pid_.reset();
        *output_.yaw_control_angle = kNaN;
        *output_.pitch_control_angle = kNaN;
        *output_.chassis_manual_yaw_velocity_override = kNaN;
        gimbal_solver_.update(TwoAxisGimbalSolver::SetDisabled{});
    }

    auto update_ctrl_hold_control() -> void {
        if (!ctrl_hold_active_)
            activate_ctrl_hold();

        *output_.yaw_control_torque = kNaN;
        *output_.yaw_control_angle = kNaN;
        *output_.pitch_control_velocity = kNaN;
        *output_.pitch_control_torque = kNaN;
        *output_.pitch_control_angle = kNaN;
        *output_.chassis_manual_yaw_velocity_override = kNaN;
        *output_.yaw_angle_error = kNaN;
        *output_.pitch_angle_error = kNaN;
        *output_.pitch_control_angle = kNaN;

        if (input_.yaw_angle.ready() && std::isfinite(*input_.yaw_angle) && input_.yaw_velocity.ready()
            && std::isfinite(*input_.yaw_velocity)) {
            auto yaw_target_error = locked_yaw_angle_ - *input_.yaw_angle;
            if (yaw_target_error > std::numbers::pi)
                yaw_target_error -= 2 * std::numbers::pi;
            else if (yaw_target_error < -std::numbers::pi)
                yaw_target_error += 2 * std::numbers::pi;

            *output_.yaw_angle_error = yaw_target_error;
            const auto yaw_velocity_ref = yaw_angle_pid_.update(yaw_target_error);
            *output_.yaw_control_torque =
                yaw_velocity_pid_.update(yaw_velocity_ref - *input_.yaw_velocity);
        }

        if (input_.pitch_angle.ready() && std::isfinite(*input_.pitch_angle)) {
            auto pitch_target_error = ctrl_hold_pitch_target_angle_ - *input_.pitch_angle;
            if (pitch_target_error > std::numbers::pi)
                pitch_target_error -= 2 * std::numbers::pi;
            else if (pitch_target_error < -std::numbers::pi)
                pitch_target_error += 2 * std::numbers::pi;

            *output_.pitch_angle_error = pitch_target_error;
            const auto pitch_velocity_ref = pitch_angle_pid_.update(pitch_target_error);
            if (pitch_torque_control_enabled_) {
                *output_.pitch_control_velocity = kNaN;
                *output_.pitch_control_torque =
                    pitch_velocity_pid_.update(pitch_velocity_ref - *input_.pitch_velocity_imu);
            } else {
                pitch_velocity_pid_.reset();
                *output_.pitch_control_velocity = pitch_velocity_ref;
                *output_.pitch_control_torque = kNaN;
            }
        }

        const auto yaw_velocity_override = std::clamp(
            manual_yaw_shift() / update_dt(), -ctrl_hold_chassis_yaw_velocity_max_,
            ctrl_hold_chassis_yaw_velocity_max_);
        *output_.chassis_manual_yaw_velocity_override = yaw_velocity_override;
    }

    auto reset_control_outputs() -> void {
        yaw_angle_pid_.reset();
        yaw_velocity_pid_.reset();
        pitch_angle_pid_.reset();
        pitch_velocity_pid_.reset();
        *output_.yaw_control_torque = kNaN;
        *output_.yaw_control_angle = kNaN;
        *output_.pitch_control_velocity = kNaN;
        *output_.pitch_control_torque = kNaN;
        *output_.pitch_control_angle = kNaN;
        *output_.chassis_manual_yaw_velocity_override = kNaN;
    }

    auto reset_all_controls() -> void {
        deactivate_ctrl_hold();
        gimbal_solver_.update(TwoAxisGimbalSolver::SetDisabled{});
        *output_.yaw_angle_error = kNaN;
        *output_.pitch_angle_error = kNaN;
        reset_control_outputs();
    }

    TwoAxisGimbalSolver gimbal_solver_{
        *this, get_parameter("upper_limit").as_double(), get_parameter("lower_limit").as_double()};

    pid::PidCalculator yaw_angle_pid_{
        get_parameter("yaw_angle_kp").as_double(),
        get_parameter("yaw_angle_ki").as_double(),
        get_parameter("yaw_angle_kd").as_double(),
    };
    pid::PidCalculator yaw_velocity_pid_{
        get_parameter("yaw_velocity_kp").as_double(),
        get_parameter("yaw_velocity_ki").as_double(),
        get_parameter("yaw_velocity_kd").as_double(),
    };
    pid::PidCalculator pitch_angle_pid_{
        get_parameter("pitch_angle_kp").as_double(),
        get_parameter("pitch_angle_ki").as_double(),
        get_parameter("pitch_angle_kd").as_double(),
    };
    pid::PidCalculator pitch_velocity_pid_{
        get_parameter("pitch_velocity_kp").as_double(),
        get_parameter("pitch_velocity_ki").as_double(),
        get_parameter("pitch_velocity_kd").as_double(),
    };

    double joystick_sensitivity_ = 0.003;
    double mouse_sensitivity_ = 0.5;
    bool pitch_torque_control_enabled_ = false;
    double yaw_vel_ff_gain_ = 0.0;
    double yaw_acc_ff_gain_ = 0.0;
    double pitch_acc_ff_gain_ = 0.0;
    double ctrl_hold_pitch_target_angle_ = 0.0;
    double ctrl_hold_chassis_yaw_velocity_max_ = 30.0;
    bool suspension_on_by_switch_ = false;
    bool ctrl_hold_active_ = false;
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    double locked_yaw_angle_ = kNaN;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::DeformableInfantryGimbalController, rmcs_executor::Component)
