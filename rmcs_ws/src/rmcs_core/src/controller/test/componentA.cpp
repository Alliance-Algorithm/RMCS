#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rclcpp/parameter.hpp>
#include <any>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>

namespace rmcs_core::controller::test_a {

class Component final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit Component()
        : Node{get_component_name(), options} {
        // 注册输出接口
        register_output<double>("sin", sin_output_, 0);
        register_output<double>("cos", cos_output_, 0);
        // 从配置文件中获取角频率参数
        omega_ = get_parameter_or<double>("omega", 1);
    }

    void update() override {
        // 计算当前时间的正弦和余弦值
        double sin_value = std::sin(omega_ * time_);
        double cos_value = std::cos(omega_ * time_);
        *sin_output_ = sin_value;
        *cos_output_ = cos_value; 
        // 更新时间
        time_ += 0.001;

    }

private:
    // 输出接口
    rmcs_executor::Component::OutputInterface<double> sin_output_;
    rmcs_executor::Component::OutputInterface<double> cos_output_;
    static inline auto options =
        rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true);
    double omega_;
    double time_ = 0.0;
};

} // namespace rmcs_core::controller::test_a

// 插件导出
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::test_a::Component, 
    rmcs_executor::Component
)