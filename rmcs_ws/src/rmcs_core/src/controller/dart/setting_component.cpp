#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

#include <eigen3/Eigen/Dense>
#include <numbers>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/dart_mechanism_command.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::dart {

// Consumes manager-published angle/force errors and carriage velocity commands.
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
        register_input("/dart_manager/carriage/command", carriage_command_, false);
        register_input("/dart_manager/carriage/target_velocity", carriage_target_velocity_, false);

        register_input("/force_sensor/channel_1/weight", force_sensor_ch1_);
        register_input("/force_sensor/channel_2/weight", force_sensor_ch2_);

        register_input("/imu/catapult_pitch_angle", pitch_angle_);
        register_input(
            "/dart_manager/force/max_velocity_override", force_max_velocity_override_, false);

        register_input("/dart/pitch_motor/velocity", pitch_velocity_);
        register_input("/dart/yaw_motor/velocity", yaw_velocity_);
        register_input("/dart/pitch_motor/torque", pitch_torque_);
        register_input("/dart/yaw_motor/torque", yaw_torque_);

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

        if (*yaw_torque_ > 0.5 && *yaw_velocity_ < 1.0) {
            *yaw_control_velocity_ =
                limit_velocity(angle_error[0], yaw_error_to_velocity_gain_, yaw_max_velocity_);
        } else {
            *yaw_control_velocity_ = 0.0;
            RCLCPP_WARN(get_logger(), "yaw motor stalled!");
        }

        if (*pitch_torque_ > 0.5 && *pitch_velocity_ < 1.0) {
            *pitch_control_velocity_ =
                limit_velocity(angle_error[1], pitch_error_to_velocity_gain_, pitch_max_velocity_);
        } else {
            *pitch_control_velocity_ = 0.0;
            RCLCPP_WARN(get_logger(), "pitch motor stalled!");
        }

        if (const auto carriage_control_velocity = resolve_carriage_control_velocity()) {
            *force_control_velocity_ = *carriage_control_velocity;
            force_error_velocity_pid_.reset();
        } else {
            *force_control_velocity_ = update_force_control_velocity(*force_error_);
        }

        if (count++ == 1000) {
            auto pitch = *pitch_angle_ / std::numbers::pi * 180 + 90;

            RCLCPP_INFO(
                get_logger(), "[ForSensor]: (%5d,%5d),[Pitch]: %5f,x: %5f| y:%5f",
                *force_sensor_ch1_, *force_sensor_ch2_, -pitch, angle_error.x(), angle_error.y());

            count = 0;
        }
    }

    int count = 0;

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

    static double sanitize_velocity_magnitude(const double velocity) {
        if (!std::isfinite(velocity)) {
            return 0.0;
        }
        return std::abs(velocity);
    }

    rmcs_msgs::DartMechanismCommand active_carriage_command() const {
        if (carriage_command_.ready()) {
            return *carriage_command_;
        }
        return rmcs_msgs::DartMechanismCommand::WAIT;
    }

    double requested_carriage_velocity() const {
        if (carriage_target_velocity_.ready()) {
            return sanitize_velocity_magnitude(*carriage_target_velocity_);
        }
        return 0.0;
    }

    std::optional<double> resolve_carriage_control_velocity() const {
        const double requested_velocity = requested_carriage_velocity();
        switch (active_carriage_command()) {
        case rmcs_msgs::DartMechanismCommand::DOWN: return requested_velocity;
        case rmcs_msgs::DartMechanismCommand::UP: return -requested_velocity;
        default: return std::nullopt;
        }
    }

    double force_max_velocity_limit() const {
        if (!force_max_velocity_override_.ready()) {
            return sanitize_max_velocity(force_max_velocity_);
        }

        const double override_velocity = *force_max_velocity_override_;
        if (!std::isfinite(override_velocity)) {
            return sanitize_max_velocity(force_max_velocity_);
        }

        return sanitize_max_velocity(override_velocity);
    }

    double update_force_control_velocity(const int32_t force_error) {
        const double velocity_limit = force_max_velocity_limit();
        if (force_error == 0 || velocity_limit == 0.0) {
            force_error_velocity_pid_.reset();
            return 0.0;
        }

        const double control_velocity =
            force_error_velocity_pid_.update(static_cast<double>(force_error));
        if (!std::isfinite(control_velocity)) {
            force_error_velocity_pid_.reset();
            return 0.0;
        }

        return clamp_velocity(control_velocity, velocity_limit);
    }

    double yaw_error_to_velocity_gain_;
    double pitch_error_to_velocity_gain_;
    pid::PidCalculator force_error_velocity_pid_;

    double yaw_max_velocity_;
    double pitch_max_velocity_;
    double force_max_velocity_;

    InputInterface<Eigen::Vector2d> angle_error_vector_;
    InputInterface<int32_t> force_error_;
    InputInterface<rmcs_msgs::DartMechanismCommand> carriage_command_;
    InputInterface<double> carriage_target_velocity_;

    InputInterface<int32_t> force_sensor_ch1_;
    InputInterface<int32_t> force_sensor_ch2_;

    InputInterface<double> pitch_angle_;
    InputInterface<double> force_max_velocity_override_;

    InputInterface<double> pitch_velocity_;
    InputInterface<double> yaw_velocity_;
    InputInterface<double> pitch_torque_;
    InputInterface<double> yaw_torque_;

    OutputInterface<double> yaw_control_velocity_;
    OutputInterface<double> pitch_control_velocity_;
    OutputInterface<double> force_control_velocity_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartSettingController, rmcs_executor::Component)
