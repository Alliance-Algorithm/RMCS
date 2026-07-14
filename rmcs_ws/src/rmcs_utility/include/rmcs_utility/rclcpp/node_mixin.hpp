#pragma once
#include <format>

namespace rmcs_utility {

struct NodeMixin {
    using node = NodeMixin;

    template <typename Self, typename... Args>
    auto info(this const Self& self, std::format_string<Args...> fmt, Args&&... args) -> void {
        auto text = std::format(fmt, std::forward<Args>(args)...);
        RCLCPP_INFO(self.get_logger(), "%s", text.c_str());
    }

    template <typename Self, typename... Args>
    auto warn(this const Self& self, std::format_string<Args...> fmt, Args&&... args) -> void {
        auto text = std::format(fmt, std::forward<Args>(args)...);
        RCLCPP_WARN(self.get_logger(), "%s", text.c_str());
    }

    template <typename Self, typename... Args>
    auto error(this const Self& self, std::format_string<Args...> fmt, Args&&... args) -> void {
        auto text = std::format(fmt, std::forward<Args>(args)...);
        RCLCPP_ERROR(self.get_logger(), "%s", text.c_str());
    }

    template <typename T>
    auto param(this const auto& self, const std::string& name, T& dst) {
        if (self.has_parameter(name)) {
            dst = self.template get_parameter_or<T>(name, T{});
            return;
        }

        self.error("param [ {} ] for {} is needed", name, self.get_name());
        throw std::runtime_error{"lack of param"};
    }
    template <typename T1, typename T2>
    auto param_or(this const auto& self, const std::string& name, T1& dst, const T2& fallback)
        requires std::convertible_to<T2, T1> {
        dst = self.template get_parameter_or<T1>(name, fallback);
    }
};

} // namespace rmcs_utility
