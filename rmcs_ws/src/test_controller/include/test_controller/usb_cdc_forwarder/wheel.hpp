#pragma once

#include <memory>

#include <rclcpp/node.hpp>
#include <std_msgs/msg/float64.hpp>

#include "test_controller/usb_cdc_forwarder/package.hpp"
#include "test_controller/usb_cdc_forwarder/qos.hpp"

namespace usb_cdc_forwarder {

template <bool reverse>
class Wheel {
public:
    Wheel(rclcpp::Node* node, const std::string& wheel_name)
        : node_(node) {
        angle_publisher =
            node_->create_publisher<std_msgs::msg::Float64>(wheel_name + "/angle", kSensorQoS);
        velocity_publisher =
            node_->create_publisher<std_msgs::msg::Float64>(wheel_name + "/velocity", kSensorQoS);
        control_current_subscription = node_->create_subscription<std_msgs::msg::Float64>(
            wheel_name + "/control_current", kControlQoS,
            std::bind(&Wheel::control_current_subscription_callback, this, std::placeholders::_1));
    }
    Wheel(const Wheel&)            = delete;
    Wheel& operator=(const Wheel&) = delete;

    void publish_status(std::unique_ptr<Package> package) {
        auto& static_part = package->static_part();

        if (package->dymatic_part_size() != sizeof(PackageC620FeedbackPart)) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Package size does not match (c620): [0x%02X 0x%02X] (size = %d)", static_part.type,
                static_part.index, static_part.data_size);
            return;
        }

        auto& dymatic_part = package->dymatic_part<PackageC620FeedbackPart>();

        // angle unit: rad [0, 2pi)
        auto angle  = std::make_unique<std_msgs::msg::Float64>();
        angle->data = static_cast<double>(dymatic_part.angle) / 8192.0 * 2.0 * std::numbers::pi;
        if constexpr (reverse)
            angle->data = 2.0 * std::numbers::pi - angle->data;
        angle_publisher->publish(std::move(angle));

        // velocity unit: rad/s
        auto velocity  = std::make_unique<std_msgs::msg::Float64>();
        velocity->data = static_cast<double>(dymatic_part.velocity) * 2.0 * std::numbers::pi / 60.0;
        if constexpr (reverse)
            velocity->data = -velocity->data;
        velocity_publisher->publish(std::move(velocity));
    }

    void control_current_subscription_callback(std_msgs::msg::Float64::UniquePtr msg) {
        control_current = msg->data;
    }

    void write_control_current_to_package(PackageC620ControlPart& dymatic_part, size_t index) {
        double current = control_current;
        if (!std::isfinite(current)) {
            RCLCPP_ERROR(
                node_->get_logger(), "Send wheel motor control: current[%zu] isn't a number",
                index);
            current = 0;
        }
        if constexpr (reverse)
            current = -current;
        current = std::round(current / 20.0 * 16384);
        if (current > 16384) {
            current = 16384;
        } else if (current < -16384) {
            current = -16384;
        }
        dymatic_part.current[index] = static_cast<int16_t>(current);
    }

private:
    rclcpp::Node* node_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_publisher;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr control_current_subscription;
    std::atomic<double> control_current = 0;
};

template <bool reverse>
struct WheelCollection {
    using wheel_t = Wheel<reverse>;

    template <size_t n>
    WheelCollection(rclcpp::Node* node, const std::string (&wheel_names)[n])
        : node_(node) {
        for (const auto& wheel_name : wheel_names) {
            wheel_list_.push_back(std::make_unique<wheel_t>(node, wheel_name));
        }
    }

    wheel_t& operator[](size_t n) { return *wheel_list_[n]; }

    void write_control_package(Package& package) {
        auto& static_part     = package.static_part();
        static_part.index     = 0;
        static_part.data_size = sizeof(PackageC620ControlPart);

        auto& dymatic_part  = package.dymatic_part<PackageC620ControlPart>();
        dymatic_part.can_id = 0x200;

        int i = 0;
        for (auto& wheel : wheel_list_) {
            if (wheel == nullptr) {
                RCLCPP_ERROR(node_->get_logger(), "Wheel controller: wheel is null");
                continue;
            }
            wheel->write_control_current_to_package(dymatic_part, i);
            i++;
        }
    }

    rclcpp::Node* node_;
    std::vector<std::unique_ptr<wheel_t>> wheel_list_;
};

} // namespace usb_cdc_forwarder