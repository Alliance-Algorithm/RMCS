#pragma once

#include <eigen3/Eigen/Dense>
#include <librmcs/device/dr16.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::hardware::device {

class Dr16 : public librmcs::device::Dr16 {
public:
    explicit Dr16(rmcs_executor::Component& component) {
        component.register_output(
            "/remote/joystick/right", joystick_right_, Eigen::Vector2d::Zero());
        component.register_output("/remote/joystick/left", joystick_left_, Eigen::Vector2d::Zero());

        component.register_output(
            "/remote/switch/right", switch_right_, rmcs_msgs::Switch::UNKNOWN);
        component.register_output("/remote/switch/left", switch_left_, rmcs_msgs::Switch::UNKNOWN);

        component.register_output(
            "/remote/mouse/velocity", mouse_velocity_, Eigen::Vector2d::Zero());
        component.register_output("/remote/mouse/mouse_wheel", mouse_wheel_);

        component.register_output("/remote/mouse", mouse_);
        std::memset(&*mouse_, 0, sizeof(*mouse_));
        component.register_output("/remote/keyboard", keyboard_);
        std::memset(&*keyboard_, 0, sizeof(*keyboard_));

        component.register_output("/remote/rotary_knob", rotary_knob_);
    }

    void update_status() {
        librmcs::device::Dr16::update_status();

        *joystick_right_ = joystick_right();
        *joystick_left_  = joystick_left();

        *switch_right_ = switch_right();
        *switch_left_  = switch_left();

        *mouse_velocity_ = mouse_velocity();
        *mouse_wheel_    = mouse_wheel();

        *mouse_    = mouse();
        *keyboard_ = keyboard();

        *rotary_knob_ = rotary_knob();
    }

    Eigen::Vector2d joystick_right() const {
        return to_eigen_vector(librmcs::device::Dr16::joystick_right());
    }
    Eigen::Vector2d joystick_left() const {
        return to_eigen_vector(librmcs::device::Dr16::joystick_left());
    }

    rmcs_msgs::Switch switch_right() const {
        return std::bit_cast<rmcs_msgs::Switch>(librmcs::device::Dr16::switch_right());
    }
    rmcs_msgs::Switch switch_left() const {
        return std::bit_cast<rmcs_msgs::Switch>(librmcs::device::Dr16::switch_left());
    }

    Eigen::Vector2d mouse_velocity() const {
        return to_eigen_vector(librmcs::device::Dr16::mouse_velocity());
    }

    rmcs_msgs::Mouse mouse() const {
        return std::bit_cast<rmcs_msgs::Mouse>(librmcs::device::Dr16::mouse());
    }
    rmcs_msgs::Keyboard keyboard() const {
        return std::bit_cast<rmcs_msgs::Keyboard>(librmcs::device::Dr16::keyboard());
    }

private:
    static Eigen::Vector2d to_eigen_vector(Vector vector) { return {vector.x, vector.y}; }

    rmcs_executor::Component::OutputInterface<Eigen::Vector2d> joystick_right_;
    rmcs_executor::Component::OutputInterface<Eigen::Vector2d> joystick_left_;

    rmcs_executor::Component::OutputInterface<rmcs_msgs::Switch> switch_right_;
    rmcs_executor::Component::OutputInterface<rmcs_msgs::Switch> switch_left_;

    rmcs_executor::Component::OutputInterface<Eigen::Vector2d> mouse_velocity_;

    rmcs_executor::Component::OutputInterface<rmcs_msgs::Mouse> mouse_;
    rmcs_executor::Component::OutputInterface<rmcs_msgs::Keyboard> keyboard_;

    rmcs_executor::Component::OutputInterface<double> rotary_knob_;
    rmcs_executor::Component::OutputInterface<double> mouse_wheel_;
};

} // namespace rmcs_core::hardware::device