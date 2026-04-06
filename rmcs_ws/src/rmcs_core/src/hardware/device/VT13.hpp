#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <atomic>
#include <bit>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/VT13_switch.hpp>

#ifndef PACKED_STRUCT
#if defined(_MSC_VER)
# define PACKED_STRUCT(...) __pragma(pack(push, 1)) struct __VA_ARGS__ __pragma(pack(pop))
#elif defined(__GNUC__)
# define PACKED_STRUCT(...) struct __attribute__((packed)) __VA_ARGS__
#else
# define PACKED_STRUCT(...) struct __VA_ARGS__
#endif
#endif

namespace rmcs_core::hardware::device {

class Dr16 {
public:
    explicit Dr16(rmcs_executor::Component& component) {
        component.register_output(
            "/remote/joystick/right", joystick_right_output_, Eigen::Vector2d::Zero());
        component.register_output(
            "/remote/joystick/left", joystick_left_output_, Eigen::Vector2d::Zero());

        component.register_output(
            "/remote/VT13switch/state", switch_output_, rmcs_msgs::VT13Switch::UNKNOWN);

        component.register_output(
            "/remote/mouse/velocity", mouse_velocity_output_, Eigen::Vector2d::Zero());
        component.register_output("/remote/mouse/mouse_wheel", mouse_wheel_output_);

        component.register_output("/remote/mouse", mouse_output_);
        std::memset(&*mouse_output_, 0, sizeof(*mouse_output_));
        component.register_output("/remote/keyboard", keyboard_output_);
        std::memset(&*keyboard_output_, 0, sizeof(*keyboard_output_));

        component.register_output("/remote/mouse/middle", mouse_middle_output_);
        component.register_output("/remote/button/pause", pause_button_output_);
        component.register_output("/remote/button/left", button_left_output_);
        component.register_output("/remote/button/right", button_right_output_);
        component.register_output("/remote/button/trigger", trigger_output_);

        component.register_output("/remote/rotary_knob", rotary_knob_output_);

        // Simulate the rotary knob as a switch, with anti-shake algorithm.
    }

    void store_status(const std::byte* uart_data, size_t uart_data_length) {
        if (uart_data_length != frame_size || !verify_frame(uart_data))
            return;

        // Avoid using reinterpret_cast here because it does not account for pointer alignment.
        // Dr16DataPart structures are aligned, and using reinterpret_cast on potentially unaligned
        // uart_data can cause undefined behavior on architectures that enforce strict alignment
        // requirements (e.g., ARM).
        // Directly accessing unaligned memory through a casted pointer can lead to crashes,
        // inefficiencies, or incorrect data reads. Instead, std::memcpy safely copies the data from
        // unaligned memory to properly aligned structures without violating alignment or strict
        // aliasing rules.

        uint64_t part1{};
        std::memcpy(&part1, uart_data, 8);
        uart_data += 8;
        data_part1_.store(part1, std::memory_order::relaxed);

        uint64_t part2{};
        std::memcpy(&part2, uart_data, 2);
        uart_data += 2;
        data_part2_.store(part2, std::memory_order::relaxed);

        uint64_t part3{};
        std::memcpy(&part3, uart_data, 7);
        uart_data += 7;
        data_part3_.store(part3, std::memory_order::relaxed);

        uint32_t part4{};
        std::memcpy(&part4, uart_data, 4);
        uart_data += 4;
        data_part4_.store(part4, std::memory_order::relaxed);
    }

    void update_status() {
        auto part1 alignas(uint64_t) =
            std::bit_cast<Vt13DataPart1>(data_part1_.load(std::memory_order::relaxed));

        auto channel_to_double = [](int32_t value) {
            value -= 1024;
            if (-660 <= value && value <= 660)
                return value / 660.0;
            return 0.0;
        };
        joystick_right_.y = -channel_to_double(static_cast<uint16_t>(part1.joystick_channel0));
        joystick_right_.x = channel_to_double(static_cast<uint16_t>(part1.joystick_channel1));
        joystick_left_.y = -channel_to_double(static_cast<uint16_t>(part1.joystick_channel2));
        joystick_left_.x = channel_to_double(static_cast<uint16_t>(part1.joystick_channel3));

        switch_state = static_cast<rmcs_msgs::VT13Switch>(part1.switch_state_raw);

        pause_button_ = part1.pause_button;
        button_left_ = part1.button_left;

        auto part2 alignas(uint16_t) =
            std::bit_cast<VT13DataPart2>(data_part2_.load(std::memory_order::relaxed));


        button_right_ = part2.button_right;
        rotary_knob_ = channel_to_double(part2.knob);
        trigger_ = part2.trigger;


        auto part3 alignas(uint64_t) =
            std::bit_cast<VT13DataPart3>(data_part3_.load(std::memory_order::relaxed));

        mouse_velocity_.x = -part3.mouse_velocity_y / 32768.0;
        mouse_velocity_.y = -part3.mouse_velocity_x / 32768.0;

        mouse_wheel_ = -part3.mouse_velocity_z / 32768.0;

        mouse_.left = part3.mouse_left != 0;
        mouse_.right = part3.mouse_right != 0;
        mouse_middle_ = part3.mouse_middle != 0;

        auto part4 alignas(uint32_t) =
            std::bit_cast<VT13DataPart4>(data_part4_.load(std::memory_order::relaxed));

        keyboard_ = part4.keyboard;

        *joystick_right_output_ = joystick_right();
        *joystick_left_output_ = joystick_left();

        *switch_output_ = switch_();

        *mouse_velocity_output_ = mouse_velocity();
        *mouse_wheel_output_ = mouse_wheel();

        *mouse_output_ = mouse();
        *keyboard_output_ = keyboard();

        *rotary_knob_output_ = rotary_knob();

        botton_update();

    }

    struct Vector {
        constexpr static Vector zero() { return {.x = 0, .y = 0}; }
        double x, y;
    };

    enum class Switch : uint8_t { kUnknown = 0, kUp = 1, kDown = 2, kMiddle = 3 };

    struct [[gnu::packed]] Mouse {
        constexpr static Mouse zero() {
            constexpr uint8_t zero = 0;
            return std::bit_cast<Mouse>(zero);
        }

        bool left  : 1;
        bool right : 1;
    };
    static_assert(sizeof(Mouse) == 1);

    struct [[gnu::packed]] Keyboard {
        constexpr static Keyboard zero() {
            constexpr uint16_t zero = 0;
            return std::bit_cast<Keyboard>(zero);
        }

        bool w     : 1;
        bool s     : 1;
        bool a     : 1;
        bool d     : 1;
        bool shift : 1;
        bool ctrl  : 1;
        bool q     : 1;
        bool e     : 1;
        bool r     : 1;
        bool f     : 1;
        bool g     : 1;
        bool z     : 1;
        bool x     : 1;
        bool c     : 1;
        bool v     : 1;
        bool b     : 1;
    };
    static_assert(sizeof(Keyboard) == 2);

    Eigen::Vector2d joystick_right() const { return to_eigen_vector(joystick_right_); }
    Eigen::Vector2d joystick_left() const { return to_eigen_vector(joystick_left_); }

    rmcs_msgs::VT13Switch switch_() const {
        return std::bit_cast<rmcs_msgs::VT13Switch>(switch_state);
    }

    Eigen::Vector2d mouse_velocity() const { return to_eigen_vector(mouse_velocity_); }

    rmcs_msgs::Mouse mouse() const { return std::bit_cast<rmcs_msgs::Mouse>(mouse_); }
    rmcs_msgs::Keyboard keyboard() const { return std::bit_cast<rmcs_msgs::Keyboard>(keyboard_); }

    double rotary_knob() const { return rotary_knob_; }

    double mouse_wheel() const { return mouse_wheel_; }

    void botton_update() {
        *pause_button_output_ = pause_button_;
        *button_left_output_ = button_left_;
        *button_right_output_ = button_right_;
        *trigger_output_ = trigger_;
        *mouse_middle_output_ = mouse_middle_;
    }

private:
    constexpr static size_t frame_size = 8 + 2 + 7 + 4;

    static Eigen::Vector2d to_eigen_vector(Vector vector) { return {vector.x, vector.y}; }

    static bool verify_frame(const std::byte* uart_data) {
        if (std::to_integer<uint8_t>(uart_data[0]) != 0xA9
            || std::to_integer<uint8_t>(uart_data[1]) != 0x53)
            return false;

        uint16_t expected_crc{};
        std::memcpy(
            &expected_crc,
            uart_data + frame_size - sizeof(expected_crc),
            sizeof(expected_crc));

        return calculate_crc16_ccitt_false(uart_data, frame_size - sizeof(expected_crc))
            == expected_crc;
    }

    static uint16_t calculate_crc16_ccitt_false(const std::byte* data, size_t length) {
        uint16_t crc = 0xFFFF;
        while (length--) {
            crc ^= static_cast<uint16_t>(std::to_integer<uint8_t>(*data++)) << 8;
            for (int bit = 0; bit < 8; ++bit) {
                crc = (crc & 0x8000) != 0 ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                          : static_cast<uint16_t>(crc << 1);
            }
        }
        return crc;
    }


    struct [[gnu::packed]] Vt13DataPart1 {
        uint64_t identification0 : 8;
        uint64_t identification1 : 8;

        uint64_t joystick_channel0 : 11;
        uint64_t joystick_channel1 : 11;
        uint64_t joystick_channel2 : 11;
        uint64_t joystick_channel3 : 11;

        uint64_t switch_state_raw : 2;

        uint64_t pause_button : 1;
        uint64_t button_left : 1;
    };
    static_assert(sizeof(Vt13DataPart1) == 8);
    std::atomic<uint64_t> data_part1_{std::bit_cast<uint64_t>(Vt13DataPart1{
        .identification0 = 0xA9,
        .identification1 = 0x53,
        .joystick_channel0 = 1024,
        .joystick_channel1 = 1024,
        .joystick_channel2 = 1024,
        .joystick_channel3 = 1024,
        .switch_state_raw = static_cast<uint64_t>(rmcs_msgs::VT13Switch::UNKNOWN),
        .pause_button = 0,
        .button_left = 0,
    })};
    static_assert(decltype(data_part1_)::is_always_lock_free);
    struct [[gnu::packed]] VT13DataPart2 {

        uint64_t button_right : 1;
        uint64_t knob : 11;
        uint64_t trigger : 1;

        uint64_t padding : 3;
    };
    static_assert(sizeof(VT13DataPart2) == 2);
    std::atomic<uint16_t> data_part2_{std::bit_cast<uint16_t>(VT13DataPart2{

        .button_right = 0,
        .knob = 1024,
        .trigger = 0,
        .padding = 0,
    })};

    struct [[gnu::packed]] VT13DataPart3 {
        int16_t mouse_velocity_x : 16;
        int16_t mouse_velocity_y : 16;
        int16_t mouse_velocity_z : 16;

        uint64_t mouse_left : 2;
        uint64_t mouse_right : 2;
        uint64_t mouse_middle : 2;

        uint64_t padding : 10;
    };
    static_assert(sizeof(VT13DataPart3) == 8);
    std::atomic<uint64_t> data_part3_{std::bit_cast<uint64_t>(VT13DataPart3{
        .mouse_velocity_x = 0,
        .mouse_velocity_y = 0,
        .mouse_velocity_z = 0,
        .mouse_left = 0,
        .mouse_right = 0,
        .mouse_middle = 0,
        .padding = 0,
    })};
    static_assert(decltype(data_part3_)::is_always_lock_free);

    struct [[gnu::packed]] VT13DataPart4 {
        Keyboard keyboard;
        uint16_t CRCVerification;
    };
    static_assert(sizeof(VT13DataPart4) == 4);
    std::atomic<uint32_t> data_part4_ = {std::bit_cast<uint32_t>(VT13DataPart4{
        .keyboard = Keyboard::zero(),
        .CRCVerification = 0xFFFF,
    })};
    static_assert(decltype(data_part4_)::is_always_lock_free);

    Vector joystick_right_ = Vector::zero();
    Vector joystick_left_ = Vector::zero();

    rmcs_msgs::VT13Switch switch_state = rmcs_msgs::VT13Switch::UNKNOWN;

    Vector mouse_velocity_ = Vector::zero();

    Mouse mouse_ = Mouse::zero();
    Keyboard keyboard_ = Keyboard::zero();

    double rotary_knob_ = 0.0;
    double mouse_wheel_ = 0.0;

    bool pause_button_ = false;
    bool button_left_ = false;
    bool button_right_ = false;
    bool trigger_ = false;
    bool mouse_middle_ = false;

    rmcs_executor::Component::OutputInterface<Eigen::Vector2d> joystick_right_output_;
    rmcs_executor::Component::OutputInterface<Eigen::Vector2d> joystick_left_output_;

    rmcs_executor::Component::OutputInterface<rmcs_msgs::VT13Switch> switch_output_;

    rmcs_executor::Component::OutputInterface<Eigen::Vector2d> mouse_velocity_output_;
    rmcs_executor::Component::OutputInterface<double> mouse_wheel_output_;

    rmcs_executor::Component::OutputInterface<rmcs_msgs::Mouse> mouse_output_;
    rmcs_executor::Component::OutputInterface<rmcs_msgs::Keyboard> keyboard_output_;

    rmcs_executor::Component::OutputInterface<double> rotary_knob_output_;

    rmcs_executor::Component::OutputInterface<bool> pause_button_output_;
    rmcs_executor::Component::OutputInterface<bool> button_left_output_;
    rmcs_executor::Component::OutputInterface<bool> button_right_output_;
    rmcs_executor::Component::OutputInterface<bool> trigger_output_;
    rmcs_executor::Component::OutputInterface<bool> mouse_middle_output_;
    
};

} // namespace rmcs_core::hardware::device
