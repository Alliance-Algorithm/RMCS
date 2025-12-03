#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::dart {
class Test
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Test()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {
        register_input("/dart/pitch_motor/control_torque",   pitch_control_torque_);
        register_input("/dart/yaw_motor/control_torque",     yaw_control_torque_);
        register_input("/dart/pitch_motor/velocity",   pitch_speed_);
        register_input("/dart/yaw_motor/velocity",     yaw_speed_);
        }
    void update() override {
        // RCLCPP_INFO(this->get_logger(), "pitch_control_torque = %f, yaw_control_torque = %f", *pitch_control_torque_, *yaw_control_torque_);
        // RCLCPP_INFO(this->get_logger(), "pitch_speed = %f, yaw_speed = %f", *pitch_speed_, *yaw_speed_);
    }
private:
    rclcpp::Logger logger_;
    InputInterface<double> pitch_control_torque_;
    InputInterface<double> yaw_control_torque_;
    InputInterface<double> pitch_speed_;
    InputInterface<double> yaw_speed_;
};
} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::Test, rmcs_executor::Component)