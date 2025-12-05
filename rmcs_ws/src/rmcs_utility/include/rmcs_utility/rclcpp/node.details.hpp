#pragma once
#include "node.hpp"
#include "rclcpp/node.hpp"
#include <rclcpp/executors.hpp>

namespace rmcs_util {

namespace qos {
inline const auto debug = rclcpp::QoS{10}.reliable().keep_last(10);
inline const auto real_time = rclcpp::QoS{5}.best_effort().keep_last(5);
} // namespace qos
namespace prefix {
constexpr auto check_naming(std::string_view name) noexcept -> bool {
    return std::ranges::all_of(
               name,
               [](char c) {
                   return (c >= 'a' && c <= 'z') || (c >= '0' && c < '9') || (c == '_')
                       || (c == '/');
               })
        && !std::ranges::empty(name);
}
constexpr auto naming_standard = "Names must match pattern: ^[a-z0-9_/]+$";
} // namespace prefix

struct RclcppNode::Details {

    std::shared_ptr<rclcpp::Node> rclcpp;

    explicit Details(const std::string& name) noexcept
        : rclcpp{std::make_shared<rclcpp::Node>(name)} {}

    auto spin_once() const noexcept { rclcpp::spin_some(rclcpp); }

    template <class T>
    auto make_pub(const std::string& topic_name, const rclcpp::QoS& qos) noexcept {
        return rclcpp->create_publisher<T>(topic_name, qos);
    }
    template <class T, typename F>
    auto make_sub(const std::string& topic_name, const rclcpp::QoS& qos, F&& callback) {
        return rclcpp->create_subscription<T>(topic_name, qos, std::forward<F>(callback));
    }
};

} // namespace rmcs_util
