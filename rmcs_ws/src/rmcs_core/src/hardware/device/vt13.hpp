#pragma once

#include <cstdint>

#include <eigen3/Eigen/Dense>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>

namespace rmcs_core::hardware::device {

class Vt13 {
public:
    enum class ModeSwitch : uint8_t {
        kUnknown = 0,
        kCine    = 1,
        kNormal  = 2,
        kSport   = 3,
    };

    Vt13() = default;

    // ---- Called by RemoteControl when parsing raw frame ----

    void set_mode_switch(ModeSwitch m) { mode_switch_ = m; }
    void set_valid(bool v) { valid_ = v; }

    void set_joystick_left(const Eigen::Vector2d& v) { joystick_left_ = v; }
    void set_joystick_right(const Eigen::Vector2d& v) { joystick_right_ = v; }

    void set_mouse_velocity(const Eigen::Vector2d& v) { mouse_velocity_ = v; }
    void set_mouse_wheel(double v) { mouse_wheel_ = v; }

    void set_mouse(rmcs_msgs::Mouse v) { mouse_ = v; }
    void set_keyboard(rmcs_msgs::Keyboard v) { keyboard_ = v; }

    // ---- Getters (used by RemoteControl for mode-switch) ----

    [[nodiscard]] ModeSwitch mode_switch() const noexcept { return mode_switch_; }
    [[nodiscard]] bool valid() const noexcept { return valid_; }

    [[nodiscard]] const Eigen::Vector2d& joystick_left() const noexcept { return joystick_left_; }
    [[nodiscard]] const Eigen::Vector2d& joystick_right() const noexcept { return joystick_right_; }

    [[nodiscard]] const Eigen::Vector2d& mouse_velocity() const noexcept { return mouse_velocity_; }
    [[nodiscard]] double mouse_wheel() const noexcept { return mouse_wheel_; }

    [[nodiscard]] rmcs_msgs::Mouse mouse() const noexcept { return mouse_; }
    [[nodiscard]] rmcs_msgs::Keyboard keyboard() const noexcept { return keyboard_; }

    struct [[gnu::packed]] Vt13FrameData {
        static constexpr uint16_t kHeaderMagic = 0x53a9;

        uint16_t header;

        uint16_t joystick_channel0 : 11;
        uint16_t joystick_channel1 : 11;
        uint16_t joystick_channel2 : 11;
        uint16_t joystick_channel3 : 11;

        uint8_t mode_switch         : 2;
        uint8_t pause_button        : 1;
        uint8_t left_custom_button  : 1;
        uint8_t right_custom_button : 1;
        uint16_t dial               : 11;
        uint8_t trigger             : 1;
        uint8_t padding1            : 3;

        int16_t mouse_velocity_x;
        int16_t mouse_velocity_y;
        int16_t mouse_velocity_z;
        uint8_t mouse_left   : 2;
        uint8_t mouse_right  : 2;
        uint8_t mouse_middle : 2;
        uint8_t padding2     : 2;

        uint16_t keyboard;

        uint16_t crc16;
    };

private:
    bool valid_             = false;
    ModeSwitch mode_switch_ = ModeSwitch::kUnknown;

    Eigen::Vector2d joystick_left_  = Eigen::Vector2d::Zero();
    Eigen::Vector2d joystick_right_ = Eigen::Vector2d::Zero();

    Eigen::Vector2d mouse_velocity_ = Eigen::Vector2d::Zero();
    double mouse_wheel_             = 0;

    rmcs_msgs::Mouse mouse_       = rmcs_msgs::Mouse::zero();
    rmcs_msgs::Keyboard keyboard_ = rmcs_msgs::Keyboard::zero();
};

} // namespace rmcs_core::hardware::device
