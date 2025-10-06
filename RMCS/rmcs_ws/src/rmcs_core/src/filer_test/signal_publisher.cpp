#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rclcpp/parameter.hpp>
#include <any>
#include <cmath>
#include <random>
#include <pluginlib/class_list_macros.hpp>

namespace rmcs_core::filter_test::signal_publisher {

class Signal final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit Signal()
        : Node{get_component_name(), options} {

        // 注册输出接口
        register_output<double>(get_parameter("sin_test").as_string(), sin_output_, 0);
        register_output<double>(get_parameter("noisy_sin_test").as_string(), noisy_sin_output_, 0);
    }

    void update() override {
        // 计算当前时间的正弦值
        double sin_value = std::sin(omega_ * time_);
        *sin_output_ = sin_value;
        *noisy_sin_output_ = sin_value + noise_level_ * ((std::rand() % 2000) / 1000.0 - 1.0);
        // 更新时间
        time_ += 0.001;
 
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "生成信号: 纯净=%.3f, 噪声后=%.3f", 
                 sin_value, *noisy_sin_output_);

    }

private:
    // 输出接口
    rmcs_executor::Component::OutputInterface<double> sin_output_;
    rmcs_executor::Component::OutputInterface<double> noisy_sin_output_;
    static inline auto options =
        rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true);
    double omega_ = M_PI;
    double noise_level_ = 0.01;
    double time_ = 0.0;
};

}

// 插件导出
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::filter_test::signal_publisher::Signal, 
    rmcs_executor::Component
)