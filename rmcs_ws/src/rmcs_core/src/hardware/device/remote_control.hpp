#pragma once

#include <cstring>

#include <eigen3/Eigen/Dense>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

#include "hardware/device/dr16.hpp"
#include "hardware/device/vt13.hpp"

namespace rmcs_core::hardware::device {

class RemoteControl {
public:
    RemoteControl(rmcs_executor::Component& component, Dr16& dr16, Vt13& vt13)
        : dr16_(dr16)
        , vt13_(vt13) {
        component.register_output(
            "/remote/joystick/right", joystick_right_output_, Eigen::Vector2d::Zero());
        component.register_output(
            "/remote/joystick/left", joystick_left_output_, Eigen::Vector2d::Zero());

        component.register_output(
            "/remote/switch/right", switch_right_output_, rmcs_msgs::Switch::UNKNOWN);
        component.register_output(
            "/remote/switch/left", switch_left_output_, rmcs_msgs::Switch::UNKNOWN);

        component.register_output("/remote/rotary_knob", rotary_knob_output_, 0.0);
        component.register_output(
            "/remote/rotary_knob_switch", rotary_knob_switch_output_, rmcs_msgs::Switch::UNKNOWN);

        component.register_output(
            "/remote/mouse/velocity", mouse_velocity_output_, Eigen::Vector2d::Zero());
        component.register_output("/remote/mouse/mouse_wheel", mouse_wheel_output_, 0.0);

        component.register_output("/remote/mouse", mouse_output_, rmcs_msgs::Mouse::zero());
        component.register_output(
            "/remote/keyboard", keyboard_output_, rmcs_msgs::Keyboard::zero());
    }

    void update() {
        if (vt13_.mode_switch() == Vt13::ModeSwitch::kCine) {
            *switch_right_output_ = rmcs_msgs::Switch::DOWN;
            *switch_left_output_ = rmcs_msgs::Switch::DOWN;
        } else {
            *switch_right_output_ = dr16_.switch_right();
            *switch_left_output_ = dr16_.switch_left();
        }

        if (vt13_.mode_switch() == Vt13::ModeSwitch::kSport) {
            *joystick_right_output_ = vt13_.joystick_right();
            *joystick_left_output_ = vt13_.joystick_left();

            *mouse_velocity_output_ = vt13_.mouse_velocity();
            *mouse_wheel_output_ = vt13_.mouse_wheel();

            *mouse_output_ = vt13_.mouse();
            *keyboard_output_ = vt13_.keyboard();
        } else {
            *joystick_right_output_ = dr16_.joystick_right();
            *joystick_left_output_ = dr16_.joystick_left();

            *mouse_velocity_output_ = dr16_.mouse_velocity();
            *mouse_wheel_output_ = dr16_.mouse_wheel();

            *mouse_output_ = dr16_.mouse();
            *keyboard_output_ = dr16_.keyboard();
        }

        *rotary_knob_output_ = dr16_.rotary_knob();
        *rotary_knob_switch_output_ = dr16_.rotary_knob_switch();
    }

private:
    Dr16& dr16_;
    Vt13& vt13_;

    rmcs_executor::Component::OutputInterface<Eigen::Vector2d> joystick_right_output_;
    rmcs_executor::Component::OutputInterface<Eigen::Vector2d> joystick_left_output_;

    rmcs_executor::Component::OutputInterface<rmcs_msgs::Switch> switch_right_output_;
    rmcs_executor::Component::OutputInterface<rmcs_msgs::Switch> switch_left_output_;

    rmcs_executor::Component::OutputInterface<double> rotary_knob_output_;
    rmcs_executor::Component::OutputInterface<rmcs_msgs::Switch> rotary_knob_switch_output_;

    rmcs_executor::Component::OutputInterface<Eigen::Vector2d> mouse_velocity_output_;
    rmcs_executor::Component::OutputInterface<double> mouse_wheel_output_;

    rmcs_executor::Component::OutputInterface<rmcs_msgs::Mouse> mouse_output_;
    rmcs_executor::Component::OutputInterface<rmcs_msgs::Keyboard> keyboard_output_;
};

} // namespace rmcs_core::hardware::device
