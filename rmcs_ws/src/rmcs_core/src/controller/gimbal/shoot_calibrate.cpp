#pragma once

#include "rmcs_msgs/long_distance_shoot_mode.hpp"
#include <keyboard.hpp>
#include <mouse.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <switch.hpp>

namespace rmcs_core::controller::gimbal {

class ShootCalibrate
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ShootCalibrate()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))

    {
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        register_output(
            "/shoot/long_distance_shoot_mode", long_distance_shoot_mode_,
            rmcs_msgs::LongDistanceShootMode::Normal);
    }

    void update() override {
        const auto keyboard     = *keyboard_;
        const auto switch_right = *switch_right_;
        const auto switch_left  = *switch_left_;
        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
        } else {
            if (!last_keyboard_.q && keyboard_->q) {
                ++q_pressed_count_;
                q_pressed_count_ %= 3;
            }
            if ((!last_keyboard_.e && keyboard.e)||(!last_keyboard_.x&&keyboard.x))
                q_pressed_count_ = 0;
            
            if (q_pressed_count_ == 0)
                *long_distance_shoot_mode_ = LongDistanceShootMode::Normal;
            else if (q_pressed_count_ == 1)
                *long_distance_shoot_mode_ = LongDistanceShootMode::Outpost;
            else if (q_pressed_count_ == 2)
                *long_distance_shoot_mode_ = LongDistanceShootMode::Base;
        }
        last_keyboard_ = keyboard;
    }

private:
    void reset_all_controls() {
        q_pressed_count_           = 0;
        last_keyboard_             = rmcs_msgs::Keyboard::zero();
        *long_distance_shoot_mode_ = rmcs_msgs::LongDistanceShootMode::Normal;
    }
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

    OutputInterface<rmcs_msgs::LongDistanceShootMode> long_distance_shoot_mode_;
    rmcs_msgs::Keyboard last_keyboard_{rmcs_msgs::Keyboard::zero()};
    int q_pressed_count_{0};
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::ShootCalibrate, rmcs_executor::Component)