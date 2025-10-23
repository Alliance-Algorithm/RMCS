// 秉持最小引入的原则，尽量不要使用 <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <cmath>

namespace rmcs_core::controller::communication {

class OutputTest
    // 继承 Component 获得接口
    : public rmcs_executor::Component
    // 继承 Node 获得 ROS2 工具
    , public rclcpp::Node {
public:
    OutputTest()
        : Node(get_component_name(),rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)){
        register_output(get_parameter("name_sin").as_string(), value_sin);
        register_output(get_parameter("name_cos").as_string(), value_cos);
        w = get_parameter("w").as_double();
    }

    void update() override {
        sum += 0.001*w;
        *value_sin = sin(sum);
        *value_cos = cos(sum);

    }

private:
    OutputInterface<double> value_sin;
    OutputInterface<double> value_cos;
    
    double w;
    double sum;
};

} // namespace rmcs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::communication::OutputTest, rmcs_executor::Component)