#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/dart_mechanism_command.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::dart {

namespace {

constexpr double kYawPitchStallCommandVelocityThreshold = 0.5;
constexpr double kYawPitchStallVelocityThreshold = 1.0;
constexpr double kYawPitchStallTorqueThreshold = 0.5;
constexpr uint64_t kYawPitchStallConfirmTicks = 100;

} // namespace

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
        , carriage_angle_velocity_pid_(
              get_parameter("carriage_angle_kp").as_double(),
              get_parameter("carriage_angle_ki").as_double(),
              get_parameter("carriage_angle_kd").as_double())
        , yaw_max_velocity_(get_parameter("yaw_max_velocity").as_double())
        , pitch_max_velocity_(get_parameter("pitch_max_velocity").as_double())
        , force_max_velocity_(get_parameter("force_max_velocity").as_double()) {

        register_input("/dart_manager/angle/error_vector", angle_error_vector_);
        register_input("/dart_manager/force/error", force_error_);
        register_input("/dart_manager/carriage/command", carriage_command_, false);
        register_input("/dart_manager/carriage/target_velocity", carriage_target_velocity_, false);
        register_input("/dart_manager/carriage/target_angle", carriage_target_angle_, false);

        register_input("/force_sensor/channel_1/weight", force_sensor_ch1_);
        register_input("/force_sensor/channel_2/weight", force_sensor_ch2_);
        register_input("/dart/force_screw_motor/encoder_angle", force_screw_angle_);

        register_input("/imu/catapult_pitch_angle", pitch_angle_);
        register_input(
            "/dart_manager/force/max_velocity_override", force_max_velocity_override_, false);

        register_input("/dart/pitch_motor/velocity", pitch_velocity_);
        register_input("/dart/yaw_motor/velocity", yaw_velocity_);
        register_input("/dart/pitch_motor/torque", pitch_torque_);
        register_input("/dart/yaw_motor/torque", yaw_torque_);
        register_input("/imu/catapult_yaw_angle", yaw_angle_);
        register_input("/imu/catapult_roll_angle", roll_angle_);

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

        get_parameter("carriage_angle_integral_min", carriage_angle_velocity_pid_.integral_min);
        get_parameter("carriage_angle_integral_max", carriage_angle_velocity_pid_.integral_max);
        get_parameter(
            "carriage_angle_integral_split_min", carriage_angle_velocity_pid_.integral_split_min);
        get_parameter(
            "carriage_angle_integral_split_max", carriage_angle_velocity_pid_.integral_split_max);
        get_parameter("carriage_angle_output_min", carriage_angle_velocity_pid_.output_min);
        get_parameter("carriage_angle_output_max", carriage_angle_velocity_pid_.output_max);
    }

    void update() override {
        const Eigen::Vector2d& angle_error = *angle_error_vector_;

        const double requested_yaw_velocity =
            limit_velocity(angle_error[0], yaw_error_to_velocity_gain_, yaw_max_velocity_);
        *yaw_control_velocity_ = apply_yaw_pitch_stall_protection(
            "yaw", requested_yaw_velocity, *yaw_velocity_, *yaw_torque_, yaw_stall_counter_,
            yaw_stall_latched_);

        const double requested_pitch_velocity =
            limit_velocity(angle_error[1], pitch_error_to_velocity_gain_, pitch_max_velocity_);
        *pitch_control_velocity_ = apply_yaw_pitch_stall_protection(
            "pitch", requested_pitch_velocity, *pitch_velocity_, *pitch_torque_,
            pitch_stall_counter_, pitch_stall_latched_);

        if (const auto carriage_control_velocity = resolve_carriage_angle_control_velocity()) {
            *force_control_velocity_ = *carriage_control_velocity;
            force_error_velocity_pid_.reset();
        } else if (const auto carriage_control_velocity = resolve_carriage_control_velocity()) {
            *force_control_velocity_ = *carriage_control_velocity;
            carriage_angle_velocity_pid_.reset();
            force_error_velocity_pid_.reset();
        } else {
            carriage_angle_velocity_pid_.reset();
            *force_control_velocity_ = update_force_control_velocity(*force_error_);
        }

        if (count++ == 1000) {
            // RCLCPP_INFO(
            //     get_logger(), "[ForSensor]: (%5d,%5d),[PYR]]: (%5f,%5f,%5f)", *force_sensor_ch1_,
            //     *force_sensor_ch2_, *pitch_angle_, *yaw_angle_, *roll_angle_);

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

    double apply_yaw_pitch_stall_protection(
        const char* motor_name, const double requested_velocity, const double measured_velocity,
        const double measured_torque, uint64_t& stall_counter, bool& stall_latched) {
        if (sanitize_velocity_magnitude(requested_velocity)
            < kYawPitchStallCommandVelocityThreshold) {
            stall_counter = 0;
            stall_latched = false;
            return 0.0;
        }

        if (stall_latched) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s motor stalled!", motor_name);
            return 0.0;
        }

        const bool stall_detected =
            sanitize_velocity_magnitude(measured_velocity) < kYawPitchStallVelocityThreshold
            && std::abs(measured_torque) > kYawPitchStallTorqueThreshold;
        if (stall_detected) {
            ++stall_counter;
            if (stall_counter >= kYawPitchStallConfirmTicks) {
                stall_latched = true;
                RCLCPP_WARN(get_logger(), "%s motor stalled!", motor_name);
                return 0.0;
            }
        } else {
            stall_counter = 0;
        }

        return requested_velocity;
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

    std::optional<double> requested_carriage_target_angle() const {
        if (!carriage_target_angle_.ready()) {
            return std::nullopt;
        }

        const double target_angle = *carriage_target_angle_;
        if (!std::isfinite(target_angle)) {
            return std::nullopt;
        }

        return target_angle;
    }

    std::optional<double> resolve_carriage_angle_control_velocity() {
        const auto target_angle = requested_carriage_target_angle();
        const double velocity_limit = requested_carriage_velocity();
        if (!target_angle || velocity_limit == 0.0) {
            carriage_angle_velocity_pid_.reset();
            return std::nullopt;
        }

        const double control_velocity =
            carriage_angle_velocity_pid_.update(*target_angle - *force_screw_angle_);
        if (!std::isfinite(control_velocity)) {
            carriage_angle_velocity_pid_.reset();
            return 0.0;
        }

        return clamp_velocity(control_velocity, velocity_limit);
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
    pid::PidCalculator carriage_angle_velocity_pid_;

    double yaw_max_velocity_;
    double pitch_max_velocity_;
    double force_max_velocity_;

    InputInterface<Eigen::Vector2d> angle_error_vector_;
    InputInterface<int32_t> force_error_;
    InputInterface<rmcs_msgs::DartMechanismCommand> carriage_command_;
    InputInterface<double> carriage_target_velocity_;
    InputInterface<double> carriage_target_angle_;

    InputInterface<int32_t> force_sensor_ch1_;
    InputInterface<int32_t> force_sensor_ch2_;
    InputInterface<double> force_screw_angle_;

    InputInterface<double> pitch_angle_;
    InputInterface<double> force_max_velocity_override_;

    InputInterface<double> pitch_velocity_;
    InputInterface<double> yaw_velocity_;
    InputInterface<double> pitch_torque_;
    InputInterface<double> yaw_torque_;
    InputInterface<double> yaw_angle_;
    InputInterface<double> roll_angle_;

    OutputInterface<double> yaw_control_velocity_;
    OutputInterface<double> pitch_control_velocity_;
    OutputInterface<double> force_control_velocity_;

    uint64_t yaw_stall_counter_{0};
    uint64_t pitch_stall_counter_{0};
    bool yaw_stall_latched_{false};
    bool pitch_stall_latched_{false};
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartSettingController, rmcs_executor::Component)
