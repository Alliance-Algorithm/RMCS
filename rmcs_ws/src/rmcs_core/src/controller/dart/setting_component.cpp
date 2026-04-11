#include <algorithm>
#include <cmath>
#include <cstdint>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::dart {

// Consumes manager-published angle/force errors.
// angle_error_vector uses [0]=yaw, [1]=pitch.
class DartSettingController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartSettingController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , yaw_error_to_velocity_gain_(get_parameter("yaw_error_to_velocity_gain").as_double())
        , pitch_error_to_velocity_gain_(get_parameter("pitch_error_to_velocity_gain").as_double())
        , force_error_velocity_pid_(
              get_parameter("force_error_kp").as_double(),
              get_parameter("force_error_ki").as_double(),
              get_parameter("force_error_kd").as_double())
        , yaw_max_velocity_(get_parameter("yaw_max_velocity").as_double())
        , pitch_max_velocity_(get_parameter("pitch_max_velocity").as_double())
        , force_max_velocity_(get_parameter("force_max_velocity").as_double()) {

        register_input("/dart_manager/angle/error_vector", angle_error_vector_);
        register_input("/dart_manager/force/error", force_error_);

        register_output("/dart/yaw_motor/control_velocity", yaw_control_velocity_, 0.0);
        register_output("/dart/pitch_motor/control_velocity", pitch_control_velocity_, 0.0);
        register_output("/dart/force_screw_motor/control_velocity", force_control_velocity_, 0.0);

        const double default_force_output_max = sanitize_max_velocity(force_max_velocity_);
        force_error_velocity_pid_.output_min = -default_force_output_max;
        force_error_velocity_pid_.output_max = default_force_output_max;

        get_parameter("force_error_integral_min", force_error_velocity_pid_.integral_min);
        get_parameter("force_error_integral_max", force_error_velocity_pid_.integral_max);
        get_parameter(
            "force_error_integral_split_min", force_error_velocity_pid_.integral_split_min);
        get_parameter(
            "force_error_integral_split_max", force_error_velocity_pid_.integral_split_max);
        get_parameter("force_error_output_min", force_error_velocity_pid_.output_min);
        get_parameter("force_error_output_max", force_error_velocity_pid_.output_max);
    }

    void update() override {
        const Eigen::Vector2d& angle_error = *angle_error_vector_;
        *yaw_control_velocity_ =
            limit_velocity(angle_error[0], yaw_error_to_velocity_gain_, yaw_max_velocity_);
        *pitch_control_velocity_ =
            limit_velocity(angle_error[1], pitch_error_to_velocity_gain_, pitch_max_velocity_);
        *force_control_velocity_ = update_force_control_velocity(*force_error_);
    }

private:
    static double sanitize_max_velocity(const double max_velocity) {
        if (!std::isfinite(max_velocity)) {
            return 0.0;
        }
        return std::max(0.0, max_velocity);
    }

    static double clamp_velocity(const double velocity, const double max_velocity) {
        if (!std::isfinite(velocity)) {
            return 0.0;
        }

        const double limited_max_velocity = sanitize_max_velocity(max_velocity);
        return std::clamp(velocity, -limited_max_velocity, limited_max_velocity);
    }

    static double limit_velocity(const double error, const double gain, const double max_velocity) {
        if (!std::isfinite(error) || !std::isfinite(gain)) {
            return 0.0;
        }

        return clamp_velocity(error * gain, max_velocity);
    }

    static double
        limit_velocity(const int32_t error, const double gain, const double max_velocity) {
        return limit_velocity(static_cast<double>(error), gain, max_velocity);
    }

    double update_force_control_velocity(const int32_t force_error) {
        if (force_error == 0 || sanitize_max_velocity(force_max_velocity_) == 0.0) {
            force_error_velocity_pid_.reset();
            return 0.0;
        }

        const double control_velocity =
            force_error_velocity_pid_.update(static_cast<double>(force_error));
        if (!std::isfinite(control_velocity)) {
            force_error_velocity_pid_.reset();
            return 0.0;
        }

        return clamp_velocity(control_velocity, force_max_velocity_);
    }

    double yaw_error_to_velocity_gain_;
    double pitch_error_to_velocity_gain_;
    pid::PidCalculator force_error_velocity_pid_;

    double yaw_max_velocity_;
    double pitch_max_velocity_;
    double force_max_velocity_;

    InputInterface<Eigen::Vector2d> angle_error_vector_;
    InputInterface<int32_t> force_error_;

    OutputInterface<double> yaw_control_velocity_;
    OutputInterface<double> pitch_control_velocity_;
    OutputInterface<double> force_control_velocity_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartSettingController, rmcs_executor::Component)
