#include <limits>
#include <string>
#include <string_view>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::gimbal {

class FlightYawController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    FlightYawController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , cascade_mode_(get_parameter("cascade_mode").as_bool()) {

        register_input("/gimbal/yaw/control_angle_error", angle_error_);
        register_input("/gimbal/yaw/velocity_imu", velocity_imu_);
        register_input("/gimbal/yaw/hold_feedforward", hold_feedforward_, false);
        register_output("/gimbal/yaw/control_velocity", control_velocity_, kNan);
        register_output("/gimbal/yaw/control_torque", control_torque_, kNan);

        if (cascade_mode_) {
            outer_pid_ = pid::PidCalculator{
                get_parameter("outer_kp").as_double(),
                get_parameter("outer_ki").as_double(),
                get_parameter("outer_kd").as_double()};
            inner_pid_ = pid::PidCalculator{
                get_parameter("inner_kp").as_double(),
                get_parameter("inner_ki").as_double(),
                get_parameter("inner_kd").as_double()};
            load_limits(outer_pid_, "outer_");
            load_limits(inner_pid_, "inner_");
        } else {
            single_pid_ = pid::PidCalculator{
                get_parameter("single_kp").as_double(),
                get_parameter("single_ki").as_double(),
                get_parameter("single_kd").as_double()};
            load_limits(single_pid_, "single_");
        }
    }

    void update() override {
        const auto err = *angle_error_;
        const double ff = hold_feedforward_.ready() ? *hold_feedforward_ : 0.0;
        if (cascade_mode_) {
            const auto desired_velocity = outer_pid_.update(err);
            *control_velocity_          = desired_velocity;
            *control_torque_            = inner_pid_.update(desired_velocity - *velocity_imu_) + ff;
        } else {
            *control_velocity_ = kNan;
            *control_torque_   = single_pid_.update(err) + ff;
        }
    }

private:
    void load_limits(pid::PidCalculator& pid, std::string_view prefix) {
        const auto get = [this, prefix](std::string_view name, double& target) {
            get_parameter(std::string{prefix} + std::string{name}, target);
        };
        get("integral_min", pid.integral_min);
        get("integral_max", pid.integral_max);
        get("integral_split_min", pid.integral_split_min);
        get("integral_split_max", pid.integral_split_max);
        get("output_min", pid.output_min);
        get("output_max", pid.output_max);
    }

    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

    const bool cascade_mode_;
    pid::PidCalculator outer_pid_, inner_pid_, single_pid_;

    InputInterface<double> angle_error_;
    InputInterface<double> velocity_imu_;
    InputInterface<double> hold_feedforward_;
    OutputInterface<double> control_velocity_;
    OutputInterface<double> control_torque_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::FlightYawController, rmcs_executor::Component)
