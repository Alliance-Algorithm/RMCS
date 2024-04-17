#pragma once

#include <cstring>

#include <memory>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

#include "forwarder/package.hpp"
#include "rmcs_core/msgs.hpp"

namespace rmcs_core::forwarder {

using rmcs_executor::Component;

class Dr16Publisher {
public:
    explicit Dr16Publisher(Component* component) {
        component->register_output(
            "/remote/joystick/right", joystick_right_, Eigen::Vector2d::Zero());
        component->register_output(
            "/remote/joystick/left", joystick_left_, Eigen::Vector2d::Zero());

        component->register_output(
            "/remote/switch/right", switch_right_, rmcs_core::msgs::Switch::UNKNOWN);
        component->register_output(
            "/remote/switch/left", switch_left_, rmcs_core::msgs::Switch::UNKNOWN);

        component->register_output(
            "/remote/mouse/velocity", mouse_velocity_, Eigen::Vector2d::Zero());

        component->register_output("/remote/mouse", mouse_);
        memset(&*mouse_, 0, sizeof(*mouse_));
        component->register_output("/remote/keyboard", keyboard_);
        memset(&*keyboard_, 0, sizeof(*keyboard_));
    }

    void update_status(std::unique_ptr<Package> package, const rclcpp::Logger& logger) {
        auto& static_part = package->static_part();
        if (package->dynamic_part_size() != sizeof(PackageDr16FeedbackPart)) {
            RCLCPP_ERROR(
                logger, "Package size does not match (dr16): [0x%02X 0x%02X] (size = %d)",
                static_part.type, static_part.index, static_part.data_size);
            return;
        }

        auto& dynamic_part = package->dynamic_part<PackageDr16FeedbackPart>();

        auto channel_to_double = [](uint16_t value) {
            return (static_cast<int32_t>(value) - 1024) / 660.0;
        };
        joystick_right_->y() = -channel_to_double(dynamic_part.joystick_channel0);
        joystick_right_->x() = channel_to_double(dynamic_part.joystick_channel1);
        joystick_left_->y()  = -channel_to_double(dynamic_part.joystick_channel2);
        joystick_left_->x()  = channel_to_double(dynamic_part.joystick_channel3);

        *switch_right_ = dynamic_part.switch_right;
        *switch_left_  = dynamic_part.switch_left;

        mouse_velocity_->x() = dynamic_part.mouse_velocity_x / 32768.0;
        mouse_velocity_->y() = dynamic_part.mouse_velocity_y / 32768.0;

        mouse_->left  = dynamic_part.mouse_left;
        mouse_->right = dynamic_part.mouse_right;

        *keyboard_ = dynamic_part.keyboard;
    }

    Component::OutputInterface<Eigen::Vector2d> joystick_right_;
    Component::OutputInterface<Eigen::Vector2d> joystick_left_;

    Component::OutputInterface<rmcs_core::msgs::Switch> switch_right_;
    Component::OutputInterface<rmcs_core::msgs::Switch> switch_left_;

    Component::OutputInterface<Eigen::Vector2d> mouse_velocity_;

    Component::OutputInterface<rmcs_core::msgs::Mouse> mouse_;
    Component::OutputInterface<rmcs_core::msgs::Keyboard> keyboard_;
};

} // namespace rmcs_core::forwarder