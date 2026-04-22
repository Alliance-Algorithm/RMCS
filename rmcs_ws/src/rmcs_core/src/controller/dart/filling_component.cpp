#include <cmath>
#include <cstdint>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include <rmcs_msgs/dart_mechanism_command.hpp>
#include <rmcs_msgs/dart_motor_exit_mode.hpp>
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
        , default_lifting_velocity_(get_parameter("lifting_velocity").as_double())
        , limiting_free_angle_(static_cast<uint16_t>(get_parameter("limiting_free_angle").as_int()))
        , limiting_lock_angle_(
              static_cast<uint16_t>(get_parameter("limiting_lock_angle").as_int())) {
        register_input("/dart_manager/lift/command", lift_command_);
        register_input("/dart_manager/lift/target_velocity", lift_target_velocity_, false);
        register_input("/dart_manager/lift/exit_mode", lift_exit_mode_, false);
        register_input("/dart_manager/limit_servo/command", limiting_command_);

        register_output("/dart/lifting_left/control_velocity", lifting_left_vel_, 0.0);
        register_output("/dart/lifting_right/control_velocity", lifting_right_vel_, 0.0);
        register_output(
            "/dart/limiting_servo/control_angle", limiting_servo_angle_, limiting_lock_angle_);
    }

    void update() override {
        const double requested_velocity = requested_lift_velocity();
        switch (*lift_command_) {
        case rmcs_msgs::DartMechanismCommand::UP:
            *lifting_left_vel_ = -requested_velocity;
            *lifting_right_vel_ = +requested_velocity;
            break;
        case rmcs_msgs::DartMechanismCommand::DOWN:
            *lifting_left_vel_ = +requested_velocity;
            *lifting_right_vel_ = -requested_velocity;
            break;
        default:
            if (active_lift_exit_mode() == rmcs_msgs::ExitMode::KEEP) {
                break;
            }
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
    rmcs_msgs::ExitMode active_lift_exit_mode() const {
        if (lift_exit_mode_.ready()) {
            return *lift_exit_mode_;
        }
        return rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY;
    }

    double requested_lift_velocity() const {
        if (lift_target_velocity_.ready()) {
            return std::abs(*lift_target_velocity_);
        }
        return default_lifting_velocity_;
    }

    double default_lifting_velocity_;
    uint16_t limiting_free_angle_;
    uint16_t limiting_lock_angle_;

    InputInterface<rmcs_msgs::DartMechanismCommand> lift_command_;
    InputInterface<double> lift_target_velocity_;
    InputInterface<rmcs_msgs::ExitMode> lift_exit_mode_;
    InputInterface<rmcs_msgs::DartServoCommand> limiting_command_;

    OutputInterface<double> lifting_left_vel_;
    OutputInterface<double> lifting_right_vel_;
    OutputInterface<uint16_t> limiting_servo_angle_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartFilling, rmcs_executor::Component)
