#pragma once

#include <cstring>

#include <atomic>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::hardware::device {
using rmcs_executor::Component;

class Dr16 {
public:
    explicit Dr16(Component& component) {
        component.register_output(
            "/remote/joystick/right", joystick_right_, Eigen::Vector2d::Zero());
        component.register_output("/remote/joystick/left", joystick_left_, Eigen::Vector2d::Zero());

        component.register_output(
            "/remote/switch/right", switch_right_, rmcs_msgs::Switch::UNKNOWN);
        component.register_output("/remote/switch/left", switch_left_, rmcs_msgs::Switch::UNKNOWN);

        component.register_output(
            "/remote/mouse/velocity", mouse_velocity_, Eigen::Vector2d::Zero());

        component.register_output("/remote/mouse", mouse_);
        memset(&*mouse_, 0, sizeof(*mouse_));
        component.register_output("/remote/keyboard", keyboard_);
        memset(&*keyboard_, 0, sizeof(*keyboard_));
    }

    void store_status(const std::byte* uart_data, size_t uart_data_length) {
        if (uart_data_length != 6 + 8 + 4)
            return;

        Dr16DataPart1 part1;
        std::memcpy(&part1, uart_data, 6);
        uart_data += 6;
        data_part1_.store(part1, std::memory_order::relaxed);

        Dr16DataPart2 part2;
        std::memcpy(&part2, uart_data, 8);
        uart_data += 8;
        data_part2_.store(part2, std::memory_order::relaxed);

        Dr16DataPart3 part3;
        std::memcpy(&part3, uart_data, 4);
        uart_data += 4;
        data_part3_.store(part3, std::memory_order::relaxed);
    }

    void update() {
        auto part1 = data_part1_.load(std::memory_order::relaxed);

        auto channel_to_double = [](uint16_t value) {
            return (static_cast<int32_t>(value) - 1024) / 660.0;
        };
        joystick_right_->y() = -channel_to_double(part1.joystick_channel0);
        joystick_right_->x() = channel_to_double(part1.joystick_channel1);
        joystick_left_->y()  = -channel_to_double(part1.joystick_channel2);
        joystick_left_->x()  = channel_to_double(part1.joystick_channel3);

        *switch_right_ = part1.switch_right;
        *switch_left_  = part1.switch_left;

        auto part2 = data_part2_.load(std::memory_order::relaxed);

        mouse_velocity_->x() = -part2.mouse_velocity_y / 32768.0;
        mouse_velocity_->y() = -part2.mouse_velocity_x / 32768.0;

        mouse_->left  = part2.mouse_left;
        mouse_->right = part2.mouse_right;

        auto part3 = data_part3_.load(std::memory_order::relaxed);

        *keyboard_ = part3.keyboard;
    }

private:
    struct __attribute__((packed, aligned(8))) Dr16DataPart1 {
        uint16_t joystick_channel0 : 11;
        uint16_t joystick_channel1 : 11;
        uint16_t joystick_channel2 : 11;
        uint16_t joystick_channel3 : 11;

        rmcs_msgs::Switch switch_right : 2;
        rmcs_msgs::Switch switch_left  : 2;
    };
    std::atomic<Dr16DataPart1> data_part1_;
    static_assert(decltype(data_part1_)::is_always_lock_free);

    struct __attribute__((packed, aligned(8))) Dr16DataPart2 {
        int16_t mouse_velocity_x;
        int16_t mouse_velocity_y;
        int16_t mouse_velocity_z;

        bool mouse_left;
        bool mouse_right;
    };
    std::atomic<Dr16DataPart2> data_part2_;
    static_assert(decltype(data_part2_)::is_always_lock_free);

    struct __attribute__((packed, aligned(4))) Dr16DataPart3 {
        rmcs_msgs::Keyboard keyboard;
        uint16_t unused;
    };
    std::atomic<Dr16DataPart3> data_part3_;
    static_assert(decltype(data_part3_)::is_always_lock_free);

    Component::OutputInterface<Eigen::Vector2d> joystick_right_;
    Component::OutputInterface<Eigen::Vector2d> joystick_left_;

    Component::OutputInterface<rmcs_msgs::Switch> switch_right_;
    Component::OutputInterface<rmcs_msgs::Switch> switch_left_;

    Component::OutputInterface<Eigen::Vector2d> mouse_velocity_;

    Component::OutputInterface<rmcs_msgs::Mouse> mouse_;
    Component::OutputInterface<rmcs_msgs::Keyboard> keyboard_;
};

} // namespace rmcs_core::hardware::device