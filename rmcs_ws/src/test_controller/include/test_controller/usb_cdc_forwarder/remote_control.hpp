#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rm_msgs/msg/remote_control.hpp>

#include "test_controller/usb_cdc_forwarder/package.hpp"
#include "test_controller/usb_cdc_forwarder/qos.hpp"

namespace usb_cdc_forwarder {

class RemoteControl {
public:
    RemoteControl(rclcpp::Node* node)
        : node_(node) {
        remote_control_publisher_ =
            node_->create_publisher<rm_msgs::msg::RemoteControl>("/remote_control", kSensorQoS);
    }

    void publish_status(std::unique_ptr<Package> package) {
        auto& static_part = package->static_part();
        if (package->dymatic_part_size() != sizeof(PackageDR16FeedbackPart)) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Package size does not match (dr16): [0x%02X 0x%02X] (size = %d)", static_part.type,
                static_part.index, static_part.data_size);
            return;
        }

        auto& dymatic_part = package->dymatic_part<PackageDR16FeedbackPart>();
        auto msg           = std::make_unique<rm_msgs::msg::RemoteControl>();

        uint64_t rockers_and_switches(
            *reinterpret_cast<uint64_t*>(dymatic_part.rockers_and_switches));
        uint64_t channel_mask = 0b11111111111;

        uint16_t channel0     = (rockers_and_switches & (channel_mask << 0)) >> 0;
        uint16_t channel1     = (rockers_and_switches & (channel_mask << 11)) >> 11;
        uint16_t channel2     = (rockers_and_switches & (channel_mask << 22)) >> 22;
        uint16_t channel3     = (rockers_and_switches & (channel_mask << 33)) >> 33;
        auto channel_to_float = [](uint16_t value) {
            return static_cast<float>(static_cast<int32_t>(value) - 1024) / 660.0f;
        };

        msg->channel_right_x = channel_to_float(channel0);
        msg->channel_right_y = channel_to_float(channel1);
        msg->channel_left_x  = channel_to_float(channel2);
        msg->channel_left_y  = channel_to_float(channel3);

        uint64_t switch_mask = 0b11;
        msg->switch1         = (rockers_and_switches & (switch_mask << 44)) >> 44;
        msg->switch2         = (rockers_and_switches & (switch_mask << 46)) >> 46;

        msg->mouse_x_velocity = dymatic_part.mouse_x_velocity;
        msg->mouse_y_velocity = dymatic_part.mouse_y_velocity;
        msg->mouse_z_velocity = dymatic_part.mouse_z_velocity;

        msg->mouse_left_button  = dymatic_part.mouse_left_button;
        msg->mouse_right_button = dymatic_part.mouse_right_button;

        msg->keyboard = dymatic_part.keyboard;

        remote_control_publisher_->publish(std::move(msg));
    }

private:
    rclcpp::Node* node_;

    rclcpp::Publisher<rm_msgs::msg::RemoteControl>::SharedPtr remote_control_publisher_;
};

} // namespace usb_cdc_forwarder