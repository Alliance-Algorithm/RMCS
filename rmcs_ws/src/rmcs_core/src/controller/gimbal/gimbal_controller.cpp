
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::gimbal {

class SimpleYawController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SimpleYawController()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        auto set_pid_parameter = [this](pid::PidCalculator& pid, const std::string& name) {
            pid.kp = get_parameter(name + "_kp").as_double();
            pid.ki = get_parameter(name + "_ki").as_double();
            pid.kd = get_parameter(name + "_kd").as_double();
            get_parameter(name + "_integral_min", pid.integral_min);
            get_parameter(name + "_integral_max", pid.integral_max);
            get_parameter(name + "_output_min", pid.output_min);
            get_parameter(name + "_output_max", pid.output_max);
        };
        set_pid_parameter(yaw_angle_pid, "yaw_angle_pid");
        set_pid_parameter(yaw_velocity_pid, "yaw_velocity_pid");
        set_pid_parameter(pitch_angle_pid, "pitch_angle_pid");

        register_input("/gimbal/yaw/angle", yaw_angle_);
        register_input("/gimbal/yaw/velocity", yaw_velocity_);
        register_input("/gimbal/yaw/control_angle_error", control_angle_error_);
        register_input("/gimbal/pitch/control_angle_error", pitch_angle_error_);

        register_output("/gimbal/yaw/control_torque", yaw_control_torque_, 0.0);
        register_output("/gimbal/pitch/control_torque", pitch_control_torque_, 0.0);

    }

    void update() override {
        if (std::isnan(*control_angle_error_)) {
            *yaw_control_torque_ = nan_;
        } else {
            *yaw_control_torque_ = yaw_velocity_pid.update(
                yaw_angle_pid.update(*control_angle_error_) - *yaw_velocity_);
        }

        if(std::isnan(*pitch_angle_error_)) {
            *pitch_control_torque_ = nan_;
        } else {
            *pitch_control_torque_ = pitch_angle_pid.update(*pitch_angle_error_);
        }

    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();


    pid::PidCalculator yaw_angle_pid;
    pid::PidCalculator yaw_velocity_pid;
    pid::PidCalculator pitch_angle_pid;

    InputInterface<double> yaw_angle_;
    InputInterface<double> pitch_angle_error_;
    InputInterface<double> yaw_velocity_;
    InputInterface<double> control_angle_error_;
    InputInterface<double> control_angle_shift_;
    OutputInterface<double> yaw_control_torque_;
    OutputInterface<double> pitch_control_torque_;
};
} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::SimpleYawController, rmcs_executor::Component)