#include <cstdint>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include <rmcs_msgs/dart_mechanism_command.hpp>
#include <rmcs_msgs/dart_servo_command.hpp>

namespace rmcs_core::controller::dart {

class DartFilling
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartFilling()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , lifting_velocity_(get_parameter("lifting_velocity").as_double())
        , limiting_free_angle_(static_cast<uint16_t>(get_parameter("limiting_free_angle").as_int()))
        , limiting_lock_angle_(
              static_cast<uint16_t>(get_parameter("limiting_lock_angle").as_int())) {
        register_input("/dart/manager/lifting/command", lifting_command_);
        register_input("/dart/manager/limiting/command", limiting_command_);

        register_output("/dart/lifting_left/control_velocity", lifting_left_vel_, 0.0);
        register_output("/dart/lifting_right/control_velocity", lifting_right_vel_, 0.0);
        register_output(
            "/dart/limiting_servo/control_angle", limiting_servo_angle_, limiting_lock_angle_);
    }

    void update() override {
        switch (*lifting_command_) {
        case rmcs_msgs::DartMechanismCommand::UP:
            *lifting_left_vel_ = -lifting_velocity_;
            *lifting_right_vel_ = +lifting_velocity_;
            break;
        case rmcs_msgs::DartMechanismCommand::DOWN:
            *lifting_left_vel_ = +lifting_velocity_;
            *lifting_right_vel_ = -lifting_velocity_;
            break;
        default:
            *lifting_left_vel_ = 0.0;
            *lifting_right_vel_ = 0.0;
            break;
        }

        switch (*limiting_command_) {
        case rmcs_msgs::DartServoCommand::FREE:
            *limiting_servo_angle_ = limiting_free_angle_;
            break;
        case rmcs_msgs::DartServoCommand::LOCK:
            *limiting_servo_angle_ = limiting_lock_angle_;
            break;
        case rmcs_msgs::DartServoCommand::WAIT: break;
        }
    }

private:
    double lifting_velocity_;
    uint16_t limiting_free_angle_;
    uint16_t limiting_lock_angle_;

    InputInterface<rmcs_msgs::DartMechanismCommand> lifting_command_;
    InputInterface<rmcs_msgs::DartServoCommand> limiting_command_;

    OutputInterface<double> lifting_left_vel_;
    OutputInterface<double> lifting_right_vel_;
    OutputInterface<uint16_t> limiting_servo_angle_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartFilling, rmcs_executor::Component)
