#pragma once

#include <cstring>

#include <memory>

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
        component->register_output("/remote/joystick/right/x", joystick_right_x_, 0.0);
        component->register_output("/remote/joystick/right/y", joystick_right_y_, 0.0);
        component->register_output("/remote/joystick/left/x", joystick_left_x_, 0.0);
        component->register_output("/remote/joystick/left/y", joystick_left_y_, 0.0);

        component->register_output(
            "/remote/switch/right", switch_right_, rmcs_core::msgs::Switch::UNKNOWN);
        component->register_output(
            "/remote/switch/left", switch_left_, rmcs_core::msgs::Switch::UNKNOWN);

        component->register_output("/remote/mouse/velocity/x", mouse_velocity_x_, 0.0);
        component->register_output("/remote/mouse/velocity/y", mouse_velocity_y_, 0.0);

        component->register_output("/remote/mouse/left", mouse_left_, false);
        component->register_output("/remote/mouse/right", mouse_right_, false);

        component->register_output("/remote/keyboard", keyboard_);
        memset(&*keyboard_, 0, sizeof(rmcs_core::msgs::Keyboard));
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
            return static_cast<float>(static_cast<int32_t>(value) - 1024) / 660.0;
        };
        *joystick_right_x_ = channel_to_double(dynamic_part.joystick_right_x);
        *joystick_right_y_ = channel_to_double(dynamic_part.joystick_right_y);
        *joystick_left_x_  = channel_to_double(dynamic_part.joystick_left_x);
        *joystick_left_y_  = channel_to_double(dynamic_part.joystick_left_y);

        *switch_right_ = dynamic_part.switch_right;
        *switch_left_  = dynamic_part.switch_left;

        *mouse_velocity_x_ = dynamic_part.mouse_velocity_x / 32768.0;
        *mouse_velocity_y_ = dynamic_part.mouse_velocity_y / 32768.0;

        *mouse_left_  = dynamic_part.mouse_left;
        *mouse_right_ = dynamic_part.mouse_right;

        *keyboard_ = dynamic_part.keyboard;
    }

    Component::OutputInterface<double> joystick_right_x_;
    Component::OutputInterface<double> joystick_right_y_;
    Component::OutputInterface<double> joystick_left_x_;
    Component::OutputInterface<double> joystick_left_y_;

    Component::OutputInterface<rmcs_core::msgs::Switch> switch_right_;
    Component::OutputInterface<rmcs_core::msgs::Switch> switch_left_;

    Component::OutputInterface<double> mouse_velocity_x_;
    Component::OutputInterface<double> mouse_velocity_y_;

    Component::OutputInterface<bool> mouse_left_;
    Component::OutputInterface<bool> mouse_right_;

    Component::OutputInterface<rmcs_core::msgs::Keyboard> keyboard_;
};

} // namespace rmcs_core::forwarder