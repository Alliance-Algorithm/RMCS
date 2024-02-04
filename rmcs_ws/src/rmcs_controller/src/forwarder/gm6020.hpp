#pragma once

#include <atomic>
#include <cmath>
#include <memory>

#include <rclcpp/node.hpp>
#include <std_msgs/msg/float64.hpp>

#include "forwarder/package.hpp"
#include "rmcs_controller/qos.hpp"

namespace forwarder {

template <bool reverse>
class GM6020 {
public:
    GM6020(rclcpp::Node* node, const std::string& wheel_name)
        : node_(node) {
        angle_publisher_ =
            node_->create_publisher<std_msgs::msg::Float64>(wheel_name + "/angle", kCoreQoS);
        velocity_publisher_ =
            node_->create_publisher<std_msgs::msg::Float64>(wheel_name + "/velocity", kCoreQoS);
        control_current_subscription_ = node_->create_subscription<std_msgs::msg::Float64>(
            wheel_name + "/control_current", kCoreQoS,
            std::bind(&GM6020::control_current_subscription_callback, this, std::placeholders::_1));
    }
    GM6020(const GM6020&)            = delete;
    GM6020& operator=(const GM6020&) = delete;

    void publish_status(std::unique_ptr<Package> package) {
        auto& static_part = package->static_part();

        if (package->dymatic_part_size() != sizeof(PackageC620FeedbackPart)) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Package size does not match (6020): [0x%02X 0x%02X] (size = %d)", static_part.type,
                static_part.index, static_part.data_size);
            return;
        }

        auto& dymatic_part = package->dymatic_part<PackageC620FeedbackPart>();

        // angle unit: rad [0, 2pi)
        auto angle  = std::make_unique<std_msgs::msg::Float64>();
        angle->data = static_cast<double>(dymatic_part.angle) / 8192.0 * 2.0 * std::numbers::pi;
        if constexpr (reverse)
            angle->data = 2.0 * std::numbers::pi - angle->data;
        angle_publisher_->publish(std::move(angle));

        // velocity unit: rad/s
        auto velocity  = std::make_unique<std_msgs::msg::Float64>();
        velocity->data = static_cast<double>(dymatic_part.velocity) * 2.0 * std::numbers::pi / 60.0;
        if constexpr (reverse)
            velocity->data = -velocity->data;
        velocity_publisher_->publish(std::move(velocity));
    }

    void control_current_subscription_callback(std_msgs::msg::Float64::UniquePtr msg) {
        control_current_.store(msg->data, std::memory_order_relaxed);
    }

    void write_control_current_to_package(PackageC620ControlPart& dymatic_part, size_t index) {
        double current = control_current_.load(std::memory_order_relaxed);

        if (std::isnan(current)) {
            dymatic_part.current[index] = 0;
            return;
        }

        if constexpr (reverse)
            current = -current;
        current = std::round(current / 3.0 * 25000);
        if (current > 25000) {
            current = 25000;
        } else if (current < -25000) {
            current = -25000;
        }
        dymatic_part.current[index] = static_cast<int16_t>(current);
    }

private:
    rclcpp::Node* node_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_publisher_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr control_current_subscription_;
    std::atomic<double> control_current_ = 0;
};

} // namespace forwarder