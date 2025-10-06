#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rclcpp/parameter.hpp>
#include <any>
#include <cmath>
#include <memory>
#include <random>
#include "low_pass_filter.hpp"
#include "middle_value_filter.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace rmcs_core::filter_test::signal_transformer {

class Transformation final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit Transformation()
        : Node{get_component_name(), options} {

        // 注册输入输出接口
        register_input<double>(get_parameter("input").as_string(), in_value_);
        register_output<double>(get_parameter("output").as_string(), out_value_, 0);

        const auto name =  get_parameter("name").as_string();
        if (name == "low_pass") {
            double alpha = get_parameter("alpha").as_double();
            filter_ = std::make_shared<filter_test::LowPassFilter<1>>(alpha);
            RCLCPP_INFO(get_logger(), "使用低通滤波器, alpha: %.3f", 
                        std::dynamic_pointer_cast<filter_test::LowPassFilter<1>>(filter_)->get_alpha());
        } else if (name == "middle_value") {
            size_t window_size = get_parameter("window_size").as_int();
            filter_ = std::make_shared<filter_test::MedianFilter<1>>(window_size);
            RCLCPP_INFO(get_logger(), "使用中值滤波器, 窗口大小: %zu", 
                       std::dynamic_pointer_cast<filter_test::MedianFilter<1>>(filter_)->get_window_size());
        } else {
            RCLCPP_ERROR(get_logger(), "Unknown filter name: %s", name.c_str());
            throw std::runtime_error("Unknown filter name");
        }
    }

    void update() override {
         // 检查输入是否有效
        if (! *in_value_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "输入数据不可用");
            return;
        }
        
        double input_value = *in_value_;
        
        // 检查输入是否为NaN
        if (std::isnan(input_value)) {
            RCLCPP_WARN(get_logger(), "输入数据为NaN,重置滤波器");
            filter_->reset();
            *out_value_ = 0.0;
            return;
        }
        
        try {
            // 调用滤波器处理数据
            double filtered_value = filter_->update(input_value);
            *out_value_ = filtered_value;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
                           "滤波处理: 输入=%.3f, 输出=%.3f", input_value, filtered_value);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "滤波器处理错误: %s", e.what());
            filter_->reset();
            *out_value_ = input_value; // 出错时直接输出原始值
        }

    }

private:
    // 输入输出接口
    rmcs_executor::Component::InputInterface<double> in_value_;
    rmcs_executor::Component::OutputInterface<double> out_value_;
    std::shared_ptr<filter_test::FilterBase<1>> filter_;
    static inline auto options =
        rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true);

};

}

// 插件导出
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::filter_test::signal_transformer::Transformation, 
    rmcs_executor::Component
)