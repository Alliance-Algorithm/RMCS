#include <cmath>
#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::dart {

class DartChassisLeveling
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartChassisLeveling()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , pitch_angle_pid_(
              get_parameter("pitch_angle_kp").as_double(),
              get_parameter("pitch_angle_ki").as_double(),
              get_parameter("pitch_angle_kd").as_double())
        , roll_angle_pid_(
              get_parameter("roll_angle_kp").as_double(),
              get_parameter("roll_angle_ki").as_double(),
              get_parameter("roll_angle_kd").as_double()) {
        register_input("/imu/catapult_pitch_angle", pitch_angle_);
        register_input("/imu/catapult_roll_angle", roll_angle_);
        register_input(
            "/dart_manager/chassis_leveling/pitch/flag", chassis_pitch_leveling_flag_, false);
        register_input(
            "/dart_manager/chassis_leveling/roll/flag", chassis_roll_leveling_flag_, false);

        register_input("/dart/leveling_feet/front_left/encoder_angle", front_left_encoder_angle_);
        register_input("/dart/leveling_feet/front_right/encoder_angle", front_right_encoder_angle_);
        register_input("/dart/leveling_feet/rear_left/encoder_angle", rear_left_encoder_angle_);
        register_input("/dart/leveling_feet/rear_right/encoder_angle", rear_right_encoder_angle_);

        register_output(
            "/dart/leveling_feet/front_left/control_angle", front_left_control_angle_, 0.0);
        register_output(
            "/dart/leveling_feet/front_right/control_angle", front_right_control_angle_, 0.0);
        register_output(
            "/dart/leveling_feet/rear_left/control_angle", rear_left_control_angle_, 0.0);
        register_output(
            "/dart/leveling_feet/rear_right/control_angle", rear_right_control_angle_, 0.0);

        configure_pid_limits("pitch_angle", pitch_angle_pid_);
        configure_pid_limits("roll_angle", roll_angle_pid_);
    }

    void update() override {
        const double pitch_increment = update_axis_increment(
            chassis_pitch_leveling_enabled(), *pitch_angle_, pitch_angle_pid_);
        const double roll_increment =
            update_axis_increment(chassis_roll_leveling_enabled(), *roll_angle_, roll_angle_pid_);

        // User-defined mapping:
        // pitch -> front_left + front_right
        // roll  -> front_left + rear_left
        *front_left_control_angle_ = *front_left_encoder_angle_ + pitch_increment + roll_increment;
        *front_right_control_angle_ = *front_right_encoder_angle_ + pitch_increment;
        *rear_left_control_angle_ = *rear_left_encoder_angle_ + roll_increment;
        *rear_right_control_angle_ = *rear_right_encoder_angle_;
    }

private:
    void configure_pid_limits(const std::string& prefix, pid::PidCalculator& pid) {
        const auto parameter_name = [&prefix](const char* suffix) { return prefix + suffix; };

        // Optional YAML overrides follow the same pattern as other controllers in this package.
        (void)get_parameter(parameter_name("_integral_min"), pid.integral_min);
        (void)get_parameter(parameter_name("_integral_max"), pid.integral_max);
        (void)get_parameter(parameter_name("_integral_split_min"), pid.integral_split_min);
        (void)get_parameter(parameter_name("_integral_split_max"), pid.integral_split_max);
        (void)get_parameter(parameter_name("_output_min"), pid.output_min);
        (void)get_parameter(parameter_name("_output_max"), pid.output_max);
    }

    bool chassis_pitch_leveling_enabled() const {
        return chassis_pitch_leveling_flag_.ready() && *chassis_pitch_leveling_flag_;
    }

    bool chassis_roll_leveling_enabled() const {
        return chassis_roll_leveling_flag_.ready() && *chassis_roll_leveling_flag_;
    }

    static double
        update_axis_increment(const bool enabled, const double angle, pid::PidCalculator& pid) {
        if (!enabled || !std::isfinite(angle)) {
            pid.reset();
            return 0.0;
        }

        const double increment = pid.update(-angle);
        if (!std::isfinite(increment)) {
            pid.reset();
            return 0.0;
        }

        return increment;
    }

    pid::PidCalculator pitch_angle_pid_;
    pid::PidCalculator roll_angle_pid_;

    InputInterface<double> pitch_angle_;
    InputInterface<double> roll_angle_;
    InputInterface<bool> chassis_pitch_leveling_flag_;
    InputInterface<bool> chassis_roll_leveling_flag_;

    InputInterface<double> front_left_encoder_angle_;
    InputInterface<double> front_right_encoder_angle_;
    InputInterface<double> rear_left_encoder_angle_;
    InputInterface<double> rear_right_encoder_angle_;

    OutputInterface<double> front_left_control_angle_;
    OutputInterface<double> front_right_control_angle_;
    OutputInterface<double> rear_left_control_angle_;
    OutputInterface<double> rear_right_control_angle_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartChassisLeveling, rmcs_executor::Component)
