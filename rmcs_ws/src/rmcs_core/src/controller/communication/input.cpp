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
        register_input(get_parameter("name_sin").as_string(), value_sin);
        register_input(get_parameter("name_cos").as_string(), value_cos);
        register_output(get_parameter("name_sum").as_string(), value_sum);
        
    }

    void update() override {
        RCLCPP_INFO(get_logger(), "Value: %f", *value_sin);
        RCLCPP_INFO(get_logger(), "Value: %f", *value_cos);
        *value_sum = *value_sin + *value_cos;
    }

private:
    InputInterface<double> value_sin;
    InputInterface<double> value_cos;
    OutputInterface<double> value_sum;
};

} // namespace rmcs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::communication::InputTest, rmcs_executor::Component)