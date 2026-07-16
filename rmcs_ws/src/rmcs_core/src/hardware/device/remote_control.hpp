#pragma once

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include <eigen3/Eigen/Dense>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/tick_timer.hpp>

#include "hardware/device/dr16.hpp"
#include "hardware/device/vt13.hpp"

namespace rmcs_core::hardware::device {

class RemoteControl {
public:
    RemoteControl(rmcs_executor::Component& component, Dr16& dr16)
        : dr16_(dr16) {
        component.register_input(
            "/referee/image_transmission/vt13_frame", vt13_frame_input_);

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

        vt13_watchdog_.reset(5'000);
    }

    void update() {
        parse_vt13_frame();
        publish_remote_outputs();
    }

private:

    void parse_vt13_frame() {
        if (!vt13_frame_input_.ready())
            return;

        const auto& raw = *vt13_frame_input_;

        // Check if frame has valid header (0xa9 0x53); image_transmission already
        // verified CRC, but header check confirms fresh data vs. zeroed-out timeout
        if (raw[0] != 0xa9 || raw[1] != 0x53) {
            if (vt13_watchdog_.tick()) {
                vt13_.set_valid(false);
            }
            return;
        }

        Vt13::Vt13FrameData data{};
        std::memcpy(&data, raw.data(), sizeof(Vt13::Vt13FrameData));

        vt13_.set_mode_switch(
            static_cast<Vt13::ModeSwitch>(static_cast<uint8_t>(data.mode_switch + 1)));

        vt13_.set_joystick_right({
            channel_to_double(static_cast<uint16_t>(data.joystick_channel1)),
            -channel_to_double(static_cast<uint16_t>(data.joystick_channel0)),
        });
        vt13_.set_joystick_left({
            channel_to_double(static_cast<uint16_t>(data.joystick_channel2)),
            -channel_to_double(static_cast<uint16_t>(data.joystick_channel3)),
        });

        vt13_.set_mouse_velocity({
            -static_cast<double>(data.mouse_velocity_y) / 32768.0,
            -static_cast<double>(data.mouse_velocity_x) / 32768.0,
        });
        vt13_.set_mouse_wheel(
            -static_cast<double>(data.mouse_velocity_z) / 32768.0);

        vt13_.set_mouse({
            .left  = static_cast<bool>(data.mouse_left),
            .right = static_cast<bool>(data.mouse_right),
        });
        vt13_.set_keyboard(std::bit_cast<rmcs_msgs::Keyboard>(data.keyboard));

        vt13_.set_valid(true);
        vt13_watchdog_.reset(500);
    }

    // ---- Mode-switch logic ----

    void publish_remote_outputs() {
        if (dr16_.valid() || !vt13_.valid() || vt13_.mode_switch() == Vt13::ModeSwitch::kNormal) {
            *switch_right_output_ = dr16_.switch_right();
            *switch_left_output_  = dr16_.switch_left();

            *joystick_right_output_ = dr16_.joystick_right();
            *joystick_left_output_  = dr16_.joystick_left();

            *mouse_velocity_output_ = dr16_.mouse_velocity();
            *mouse_wheel_output_    = dr16_.mouse_wheel();

            *mouse_output_    = dr16_.mouse();
            *keyboard_output_ = dr16_.keyboard();
        } else if (vt13_.mode_switch() == Vt13::ModeSwitch::kCine) {
            *switch_right_output_ = rmcs_msgs::Switch::DOWN;
            *switch_left_output_  = rmcs_msgs::Switch::DOWN;

            *joystick_right_output_ = Eigen::Vector2d::Zero();
            *joystick_left_output_  = Eigen::Vector2d::Zero();

            *mouse_velocity_output_ = Eigen::Vector2d::Zero();
            *mouse_wheel_output_    = 0;

            *mouse_output_    = rmcs_msgs::Mouse::zero();
            *keyboard_output_ = rmcs_msgs::Keyboard::zero();
        } else if (vt13_.mode_switch() == Vt13::ModeSwitch::kSport) {
            *switch_right_output_ = rmcs_msgs::Switch::DOWN;
            *switch_left_output_  = rmcs_msgs::Switch::UP;

            *joystick_right_output_ = vt13_.joystick_right();
            *joystick_left_output_  = vt13_.joystick_left();

            *mouse_velocity_output_ = vt13_.mouse_velocity();
            *mouse_wheel_output_    = vt13_.mouse_wheel();

            *mouse_output_    = vt13_.mouse();
            *keyboard_output_ = vt13_.keyboard();
        }

        *rotary_knob_output_        = dr16_.rotary_knob();
        *rotary_knob_switch_output_ = dr16_.rotary_knob_switch();
    }

    static double channel_to_double(int32_t value) {
        value -= 1024;
        if (-660 <= value && value <= 660)
            return value / 660.0;
        return 0.0;
    }

    // ---- Members ----

    Dr16& dr16_;
    Vt13 vt13_;

    rmcs_executor::Component::InputInterface<std::array<uint8_t, sizeof(Vt13::Vt13FrameData)>>
        vt13_frame_input_;
    rmcs_utility::TickTimer vt13_watchdog_;

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
