#include "std_msgs/msg/float64.hpp"
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rmcs_executor/component.hpp>
namespace rmcs_core::simulator {

class MotorControlBridge : public rmcs_executor::Component,
                           public rclcpp::Node {
public:
  MotorControlBridge()
      : Node(get_component_name(),
             rclcpp::NodeOptions{}
                 .automatically_declare_parameters_from_overrides(true)) {

    register_input(get_parameter("simulated_motor_control_torque").as_string(),
                   simulated_motor_control_torque_);

    publisher_ = this->create_publisher<std_msgs::msg::Float64>(
        "motor_simulator_control_input", 10);
  }

  void update() override {
    std_msgs::msg::Float64 msg;
    msg.set__data(*simulated_motor_control_torque_);
    publisher_->publish(msg);
  }

private:
  InputInterface<double> simulated_motor_control_torque_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
};
} // namespace rmcs_core::simulator

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::simulator::MotorControlBridge,
                       rmcs_executor::Component)