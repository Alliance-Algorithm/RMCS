// 秉持最小引入的原则，尽量不要使用 <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::communication {

class OutputTest
    // 继承 Component 获得接口
    : public rmcs_executor::Component
    // 继承 Node 获得 ROS2 工具
    , public rclcpp::Node {
public:
    OutputTest()
        : Node(get_component_name(),rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)){
        register_output(get_parameter("name").as_string(), value_);
        input_ = get_parameter("value").as_double();
    }

    void update() override {
        *value_ = input_;
    }

private:
    OutputInterface<double> value_;
    
    double input_;
};

} // namespace rmcs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::communication::OutputTest, rmcs_executor::Component)