#include "controller/gimbal/two_axis_gimbal_solver.hpp"
#include "controller/pid/pid_calculator.hpp"

#include <cmath>
#include <limits>
#include <numbers>
#include <string>

#include <eigen3/Eigen/Geometry>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
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

        // Feedforward parameters
        yaw_inertia_ = get_parameter("inertia").as_double();
        yaw_friction_ = get_parameter("friction").as_double();
    }

    auto update() -> void override {
        const auto switch_right = *input_.switch_right;
        const auto switch_left = *input_.switch_left;

        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
            return;
        }

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
            *output_.yaw_target_angle = kNaN;
            *output_.pitch_target_angle = kNaN;
            reset_control_outputs();
            return;
        }

        *output_.yaw_target_angle = wrap_to_2pi(*input_.yaw_angle + angle_error.yaw_angle_error);
        *output_.pitch_target_angle =
            wrap_to_2pi(*input_.pitch_angle + angle_error.pitch_angle_error);

        const auto yaw_velocity_ref = yaw_angle_pid_.update(angle_error.yaw_angle_error);
        const auto pitch_velocity_ref = pitch_angle_pid_.update(angle_error.pitch_angle_error);

        // Calculate acceleration from velocity derivative with low-pass filter
        double yaw_accel_raw = 0.0;
        if (std::isfinite(last_yaw_velocity_ref_)) {
            yaw_accel_raw = (yaw_velocity_ref - last_yaw_velocity_ref_) * 1000.0;  // dt = 1ms
        }
        last_yaw_velocity_ref_ = yaw_velocity_ref;

        // Low-pass filter for acceleration (alpha = 0.1 for smoothing)
        const double alpha = 0.1;
        if (std::isfinite(filtered_yaw_accel_)) {
            filtered_yaw_accel_ = alpha * yaw_accel_raw + (1.0 - alpha) * filtered_yaw_accel_;
        } else {
            filtered_yaw_accel_ = 0.0;
        }

        // Feedforward control: torque = J * accel + b * velocity
        const auto yaw_velocity_error = yaw_velocity_ref - *input_.yaw_velocity_imu;
        const auto yaw_feedforward = yaw_inertia_ * filtered_yaw_accel_ + yaw_friction_ * yaw_velocity_ref;
        const auto yaw_feedback = yaw_velocity_pid_.update(yaw_velocity_error);

        *output_.yaw_control_torque = yaw_feedforward + yaw_feedback;
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

private:
    static constexpr auto kNaN = std::numeric_limits<double>::quiet_NaN();

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
            component.register_input("/remote/switch/right", switch_right);
            component.register_input("/remote/switch/left", switch_left);
            component.register_input("/remote/mouse/velocity", mouse_velocity);
            component.register_input("/remote/mouse", mouse);

            component.register_input("/tf", tf);
            component.register_input("/gimbal/yaw/angle", yaw_angle);
            component.register_input("/gimbal/pitch/angle", pitch_angle);
            component.register_input("/gimbal/yaw/velocity", yaw_velocity);
            component.register_input("/gimbal/pitch/velocity", pitch_velocity);
            component.register_input("/gimbal/yaw/velocity_imu", yaw_velocity_imu);
            component.register_input("/gimbal/pitch/velocity_imu", pitch_velocity_imu);

            component.register_input("/auto_aim/should_control", auto_aim_should_control, false);
            component.register_input(
                "/auto_aim/control_direction", auto_aim_control_direction, false);
        }

        InputInterface<Eigen::Vector2d> joystick_left;
        InputInterface<rmcs_msgs::Switch> switch_right;
        InputInterface<rmcs_msgs::Switch> switch_left;
        InputInterface<Eigen::Vector2d> mouse_velocity;
        InputInterface<rmcs_msgs::Mouse> mouse;

        InputInterface<Tf> tf;
        InputInterface<double> yaw_angle;
        InputInterface<double> pitch_angle;
        InputInterface<double> yaw_velocity;
        InputInterface<double> pitch_velocity;
        InputInterface<double> yaw_velocity_imu;
        InputInterface<double> pitch_velocity_imu;

        InputInterface<bool> auto_aim_should_control;
        InputInterface<Eigen::Vector3d> auto_aim_control_direction;
    } input_{*this};

    struct Output {
        explicit Output(rmcs_executor::Component& component) {
            component.register_output("/gimbal/yaw/control_torque", yaw_control_torque, kNaN);
            component.register_output(
                "/gimbal/pitch/control_velocity", pitch_control_velocity, kNaN);
            component.register_output("/gimbal/pitch/control_torque", pitch_control_torque, kNaN);
            component.register_output("/gimbal/yaw/control_angle_error", yaw_angle_error, kNaN);
            component.register_output("/gimbal/pitch/control_angle_error", pitch_angle_error, kNaN);
            component.register_output("/gimbal/yaw/target_angle", yaw_target_angle, kNaN);
            component.register_output("/gimbal/pitch/target_angle", pitch_target_angle, kNaN);
        }

        OutputInterface<double> yaw_control_torque;
        OutputInterface<double> pitch_control_velocity;
        OutputInterface<double> pitch_control_torque;
        OutputInterface<double> yaw_angle_error;
        OutputInterface<double> pitch_angle_error;
        OutputInterface<double> yaw_target_angle;
        OutputInterface<double> pitch_target_angle;
    } output_{*this};

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

        const auto yaw_shift = joystick_sensitivity_ * input_.joystick_left->y()
                             + mouse_sensitivity_ * input_.mouse_velocity->y();
        const auto pitch_shift = -joystick_sensitivity_ * input_.joystick_left->x()
                               + mouse_sensitivity_ * input_.mouse_velocity->x();
        return gimbal_solver_.update(TwoAxisGimbalSolver::SetControlShift{yaw_shift, pitch_shift});
    }

    auto reset_control_outputs() -> void {
        yaw_angle_pid_.reset();
        yaw_velocity_pid_.reset();
        pitch_angle_pid_.reset();
        pitch_velocity_pid_.reset();
        last_yaw_velocity_ref_ = kNaN;
        filtered_yaw_accel_ = kNaN;
        *output_.yaw_control_torque = kNaN;
        *output_.pitch_control_velocity = kNaN;
        *output_.pitch_control_torque = kNaN;
    }

    auto reset_all_controls() -> void {
        gimbal_solver_.update(TwoAxisGimbalSolver::SetDisabled{});
        *output_.yaw_angle_error = kNaN;
        *output_.pitch_angle_error = kNaN;
        *output_.yaw_target_angle = kNaN;
        *output_.pitch_target_angle = kNaN;
        reset_control_outputs();
    }

    static auto wrap_to_2pi(double angle) -> double {
        angle = std::fmod(angle, 2 * std::numbers::pi);
        if (angle < 0.0)
            angle += 2 * std::numbers::pi;
        return angle;
    }

    TwoAxisGimbalSolver gimbal_solver_{
        *this, get_parameter("upper_limit").as_double(), get_parameter("lower_limit").as_double(),
        get_parameter("use_encoder_pitch").as_bool()};

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

    // Feedforward parameters
    double yaw_inertia_;
    double yaw_friction_;
    double last_yaw_velocity_ref_ = kNaN;
    double filtered_yaw_accel_ = kNaN;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::DeformableInfantryGimbalController, rmcs_executor::Component)
