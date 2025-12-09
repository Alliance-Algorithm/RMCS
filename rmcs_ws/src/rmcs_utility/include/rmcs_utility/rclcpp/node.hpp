#pragma once
#include "rmcs_utility/pimpl.hpp"
#include "rmcs_utility/rclcpp/param.hpp"

#include <format>
#include <string>

namespace rmcs_util {

class RclcppNode {
    RMCS_PIMPL_DEFINITION(RclcppNode)

public:
    explicit RclcppNode(const std::string& name) noexcept;

    auto spin_once() const noexcept -> void;

    auto set_pub_topic_prefix(const std::string&) noexcept -> void;
    auto get_pub_topic_prefix() const noexcept -> std::string;

    auto params() const noexcept -> const std::unique_ptr<IParams>&;

    template <typename... Args>
    auto info(std::format_string<Args...> fmt, Args&&... args) const noexcept {
        impl_info_(std::format(fmt, std::forward<Args>(args)...));
    }
    template <typename... Args>
    auto warn(std::format_string<Args...> fmt, Args&&... args) const noexcept {
        impl_warn_(std::format(fmt, std::forward<Args>(args)...));
    }
    template <typename... Args>
    auto error(std::format_string<Args...> fmt, Args&&... args) const noexcept {
        impl_error(std::format(fmt, std::forward<Args>(args)...));
    }

    static auto shutdown() noexcept -> void;

public:
    struct Details;
    std::unique_ptr<Details> details;

private:
    auto impl_info_(const std::string&) const noexcept -> void;
    auto impl_warn_(const std::string&) const noexcept -> void;
    auto impl_error(const std::string&) const noexcept -> void;
};

} // namespace rmcs_util
