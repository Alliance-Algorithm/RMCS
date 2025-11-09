#pragma once
#include <rclcpp/node.hpp>

namespace rmcs::util {

struct Node : public rclcpp::Node {
    using rclcpp::Node::Node;

    template <typename... Args>
    [[gnu::always_inline]]
    inline auto rclcpp_info(std::format_string<Args...> fmt, Args&&... args) const noexcept
        -> void {
        RCLCPP_INFO(get_logger(), "%s", std::format(fmt, std::forward<Args>(args)...).c_str());
    }
    template <typename... Args>
    [[gnu::always_inline]]
    inline auto rclcpp_warn(std::format_string<Args...> fmt, Args&&... args) const noexcept
        -> void {
        RCLCPP_WARN(get_logger(), "%s", std::format(fmt, std::forward<Args>(args)...).c_str());
    }
    template <typename... Args>
    [[gnu::always_inline]]
    inline auto rclcpp_error(std::format_string<Args...> fmt, Args&&... args) const noexcept
        -> void {
        RCLCPP_ERROR(get_logger(), "%s", std::format(fmt, std::forward<Args>(args)...).c_str());
    }

    [[gnu::always_inline]]
    inline auto rclcpp_info(std::format_string<> fmt) const noexcept -> void {
        RCLCPP_INFO(get_logger(), "%s", std::format(fmt).c_str());
    }
    [[gnu::always_inline]]
    inline auto rclcpp_warn(std::format_string<> fmt) const noexcept -> void {
        RCLCPP_WARN(get_logger(), "%s", std::format(fmt).c_str());
    }
    [[gnu::always_inline]]
    inline auto rclcpp_error(std::format_string<> fmt) const noexcept -> void {
        RCLCPP_ERROR(get_logger(), "%s", std::format(fmt).c_str());
    }

    template <typename... Args>
    [[gnu::always_inline]]
    inline auto info(std::format_string<Args...> fmt, Args&&... args) const noexcept -> void {
        RCLCPP_INFO(get_logger(), "%s", std::format(fmt, std::forward<Args>(args)...).c_str());
    }
    template <typename... Args>
    [[gnu::always_inline]]
    inline auto warn(std::format_string<Args...> fmt, Args&&... args) const noexcept -> void {
        RCLCPP_WARN(get_logger(), "%s", std::format(fmt, std::forward<Args>(args)...).c_str());
    }
    template <typename... Args>
    [[gnu::always_inline]]
    inline auto error(std::format_string<Args...> fmt, Args&&... args) const noexcept -> void {
        RCLCPP_ERROR(get_logger(), "%s", std::format(fmt, std::forward<Args>(args)...).c_str());
    }

    [[gnu::always_inline]]
    inline auto info(std::format_string<> fmt) const noexcept -> void {
        RCLCPP_INFO(get_logger(), "%s", std::format(fmt).c_str());
    }
    [[gnu::always_inline]]
    inline auto warn(std::format_string<> fmt) const noexcept -> void {
        RCLCPP_WARN(get_logger(), "%s", std::format(fmt).c_str());
    }
    [[gnu::always_inline]]
    inline auto error(std::format_string<> fmt) const noexcept -> void {
        RCLCPP_ERROR(get_logger(), "%s", std::format(fmt).c_str());
    }
};

inline auto options = rclcpp::NodeOptions {}.automatically_declare_parameters_from_overrides(true);

} // namespace rmcs::utility
