#include <algorithm>
#include <cmath>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/dart_slider_status.hpp>

#include "controller/pid/matrix_pid_calculator.hpp"

namespace rmcs_core::controller::dart {

// Subscribes to DartManager belt/trigger commands and outputs motor torques + trigger servo value.
// Belt velocity control uses MatrixPidCalculator<2> with sync compensation.
class DartLaunchSetting
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartLaunchSetting()
        : Node{get_component_name(),
               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
        , belt_pid_{
              get_parameter("b_kp").as_double(),
              get_parameter("b_ki").as_double(),
              get_parameter("b_kd").as_double()} {

        belt_velocity_      = get_parameter("belt_velocity").as_double();
        sync_coefficient_   = get_parameter("sync_coefficient").as_double();
        max_control_torque_ = get_parameter("max_control_torque").as_double();
        trigger_free_value_ = get_parameter("trigger_free_value").as_double();
        trigger_lock_value_ = get_parameter("trigger_lock_value").as_double();

        register_input("/dart/manager/belt/command",        belt_command_);
        register_input("/dart/manager/trigger/lock_enable", trigger_lock_enable_);
        register_input("/dart/drive_belt/left/velocity",    left_belt_velocity_);
        register_input("/dart/drive_belt/right/velocity",   right_belt_velocity_);

        register_output(
            "/dart/drive_belt/left/control_torque",  left_belt_torque_,  0.0);
        register_output(
            "/dart/drive_belt/right/control_torque", right_belt_torque_, 0.0);
        register_output("/dart/trigger_servo/value", trigger_value_, trigger_lock_value_);
    }

    void update() override {
        double control_velocity = 0.0;
        switch (*belt_command_) {
        case rmcs_msgs::DartSliderStatus::DOWN:
            control_velocity = +belt_velocity_;
            break;
        case rmcs_msgs::DartSliderStatus::UP:
            control_velocity = -belt_velocity_;
            break;
        case rmcs_msgs::DartSliderStatus::WAIT:
        default:
            control_velocity = 0.0;
            break;
        }
        drive_belt_sync_control(control_velocity);

        *trigger_value_ = *trigger_lock_enable_ ? trigger_lock_value_ : trigger_free_value_;
    }

private:
    void drive_belt_sync_control(double set_velocity) {
        if (set_velocity == 0.0) {
            *left_belt_torque_  = 0.0;
            *right_belt_torque_ = 0.0;
            belt_pid_.reset();
            return;
        }

        Eigen::Vector2d setpoint_error{
            set_velocity - *left_belt_velocity_,
            set_velocity - *right_belt_velocity_};
        Eigen::Vector2d relative_velocity{
            *left_belt_velocity_  - *right_belt_velocity_,
            *right_belt_velocity_ - *left_belt_velocity_};

        Eigen::Vector2d control_torques =
            belt_pid_.update(setpoint_error) - sync_coefficient_ * relative_velocity;

        *left_belt_torque_ =
            std::clamp(control_torques[0], -max_control_torque_, max_control_torque_);
        *right_belt_torque_ =
            std::clamp(control_torques[1], -max_control_torque_, max_control_torque_);
    }

    rclcpp::Logger logger_;

    pid::MatrixPidCalculator<2> belt_pid_;

    double belt_velocity_;
    double sync_coefficient_;
    double max_control_torque_;
    double trigger_free_value_;
    double trigger_lock_value_;

    InputInterface<rmcs_msgs::DartSliderStatus> belt_command_;
    InputInterface<bool>   trigger_lock_enable_;
    InputInterface<double> left_belt_velocity_;
    InputInterface<double> right_belt_velocity_;

    OutputInterface<double> left_belt_torque_;
    OutputInterface<double> right_belt_torque_;
    OutputInterface<double> trigger_value_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::dart::DartLaunchSetting, rmcs_executor::Component)
