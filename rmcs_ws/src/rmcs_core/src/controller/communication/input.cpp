// 秉持最小引入的原则，尽量不要使用 <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rclcpp/logging.hpp>

namespace rmcs_core::controller::communication {

class InputTest
    // 继承 Component 获得接口
    : public rmcs_executor::Component
    // 继承 Node 获得 ROS2 工具
    , public rclcpp::Node {
public:
    InputTest()
        : Node(get_component_name(),rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)){
        register_input(get_parameter("name").as_string(), value_);

    }

    void update() override {
        RCLCPP_INFO(get_logger(), "Value: %f", *value_);
    }

private:
    InputInterface<double> value_;
    
};

} // namespace rmcs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::communication::InputTest, rmcs_executor::Component)