#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rclcpp/parameter.hpp>
#include <any>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>

namespace rmcs_core::controller::test_b {
 
class Component final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit Component()
        : Node{get_component_name(), options} {
        // 从输入中获取正弦和余弦值
        register_input("sin" , sin_input_);
        register_input("cos" , cos_input_);
        register_output("sum", sum_output_, 0);
        RCLCPP_INFO(get_logger(), "SignalProcessor initialized");
        
    }

    void update() override {
        if (!*sin_input_|| !*cos_input_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), 
                *get_clock(), 
                1000, 
                "Missing input values"
            );
            return;
        }
        try {
            double sin_value = *sin_input_;
            double cos_value = *cos_input_;
            // 计算相加结果
            double sum = sin_value + cos_value;
            *sum_output_ = sum;
            RCLCPP_INFO(get_logger(), "%f", sin_value);
            RCLCPP_INFO(get_logger(), "%f", cos_value);
            RCLCPP_INFO(get_logger(), "%f", sum);
        } catch (const std::bad_any_cast& e) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), 
                *get_clock(), 
                1000, 
                "Input type error: %s", 
                e.what()
            );
        }
    }

private:
    // 输入接口
    rmcs_executor::Component::InputInterface<double> sin_input_;
    rmcs_executor::Component::InputInterface<double> cos_input_;
    
    // 输出接口
    rmcs_executor::Component::OutputInterface<double> sum_output_;

    static inline auto options =
        rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true);
};

} // namespace rmcs_core::controller::test_b

// 插件导出
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::test_b::Component, 
    rmcs_executor::Component
)