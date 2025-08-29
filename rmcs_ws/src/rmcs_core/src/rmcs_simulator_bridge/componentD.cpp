#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include "std_msgs/msg/float64.hpp" 
#include "rclcpp/rclcpp.hpp"



namespace rmcs_core::rmcs_simulator_bridge::angle_test {

class ComponentD
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ComponentD()
        : rclcpp::Node(get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)){

        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "Waiting_angle", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                angle_input = msg->data;  // 将消息数据赋给输入接口
            });

        
        register_output(get_parameter("output").as_string(), aim_angle_, 30);
    }

    void update() override { *aim_angle_ = angle_input; RCLCPP_INFO(get_logger(), "%f", *aim_angle_);}

private:
    double angle_input;
    OutputInterface<double> aim_angle_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
};

} 

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::rmcs_simulator_bridge::angle_test::ComponentD, 
    rmcs_executor::Component
)
