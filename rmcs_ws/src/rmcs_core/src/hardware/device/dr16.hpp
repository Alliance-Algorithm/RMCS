#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <atomic>
#include <bit>
#include <chrono>

#include <eigen3/Eigen/Dense>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::hardware::device {

class Dr16 {
public:
    Dr16() = default;

    void store_status(const std::byte* uart_data, size_t uart_data_length) {
        if (uart_data_length != 6 + 8 + 4)
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
        std::memcpy(&part1, uart_data, 6);
        uart_data += 6;
        data_part1_.store(part1, std::memory_order::relaxed);

        uint64_t part2{};
        std::memcpy(&part2, uart_data, 8);
        uart_data += 8;
        data_part2_.store(part2, std::memory_order::relaxed);

        uint32_t part3{};
        std::memcpy(&part3, uart_data, 4);
        uart_data += 4;
        data_part3_.store(part3, std::memory_order::relaxed);

        last_remote_control_received_at_ = Clock::now();
        valid_ = true;
    }

    void update_status() {
        const auto now = Clock::now();
        refresh_validity(now);
        if (!valid_)
            return;

        auto part1 alignas(uint64_t) =
            std::bit_cast<Dr16DataPart1>(data_part1_.load(std::memory_order::relaxed));

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

        switch_right_ = static_cast<Switch>(part1.switch_right);
        switch_left_ = static_cast<Switch>(part1.switch_left);

        auto part2 alignas(uint64_t) =
            std::bit_cast<Dr16DataPart2>(data_part2_.load(std::memory_order::relaxed));

        mouse_velocity_.x = -part2.mouse_velocity_y / 32768.0;
        mouse_velocity_.y = -part2.mouse_velocity_x / 32768.0;

        mouse_wheel_ = -part2.mouse_velocity_z / 32768.0;

        mouse_.left = part2.mouse_left;
        mouse_.right = part2.mouse_right;

        auto part3 alignas(uint32_t) =
            std::bit_cast<Dr16DataPart3>(data_part3_.load(std::memory_order::relaxed));

        keyboard_ = part3.keyboard;
        rotary_knob_ = channel_to_double(part3.rotary_knob);

        update_rotary_knob_switch();
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

    rmcs_msgs::Switch switch_right() const {
        return std::bit_cast<rmcs_msgs::Switch>(switch_right_);
    }
    rmcs_msgs::Switch switch_left() const { return std::bit_cast<rmcs_msgs::Switch>(switch_left_); }

    Eigen::Vector2d mouse_velocity() const { return to_eigen_vector(mouse_velocity_); }

    rmcs_msgs::Mouse mouse() const { return std::bit_cast<rmcs_msgs::Mouse>(mouse_); }
    rmcs_msgs::Keyboard keyboard() const { return std::bit_cast<rmcs_msgs::Keyboard>(keyboard_); }

    rmcs_msgs::Switch rotary_knob_switch() const { return rotary_knob_switch_; }

    bool valid() const noexcept { return valid_; }

    void set_timeout_enabled(bool enabled) { timeout_enabled_ = enabled; }

    double rotary_knob() const { return rotary_knob_; }

    double mouse_wheel() const { return mouse_wheel_; }

private:
    static Eigen::Vector2d to_eigen_vector(Vector vector) { return {vector.x, vector.y}; }

    void update_rotary_knob_switch() {
        constexpr double divider = 0.7, anti_shake_shift = 0.05;
        double upper_divider = divider, lower_divider = -divider;

        auto switch_value = rotary_knob_switch_;
        if (switch_value == rmcs_msgs::Switch::UP)
            upper_divider -= anti_shake_shift, lower_divider -= anti_shake_shift;
        else if (switch_value == rmcs_msgs::Switch::MIDDLE)
            upper_divider += anti_shake_shift, lower_divider -= anti_shake_shift;
        else if (switch_value == rmcs_msgs::Switch::DOWN)
            upper_divider += anti_shake_shift, lower_divider += anti_shake_shift;

        const auto knob_value = -rotary_knob_;
        if (knob_value > upper_divider) {
            switch_value = rmcs_msgs::Switch::UP;
        } else if (knob_value < lower_divider) {
            switch_value = rmcs_msgs::Switch::DOWN;
        } else {
            switch_value = rmcs_msgs::Switch::MIDDLE;
        }
        rotary_knob_switch_ = switch_value;
    }

    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    static constexpr auto kFreshTimeout = std::chrono::milliseconds(500);

    void refresh_validity(const TimePoint now) {
        if (!timeout_enabled_ || !valid_ || now - last_remote_control_received_at_ <= kFreshTimeout)
            return;

        reset_remote_control_state();
        valid_ = false;
    }

    void reset_remote_control_state() {
        joystick_right_ = Vector::zero();
        joystick_left_ = Vector::zero();
        switch_right_ = Switch::kUnknown;
        switch_left_ = Switch::kUnknown;
        mouse_velocity_ = Vector::zero();
        mouse_wheel_ = 0.0;
        mouse_ = Mouse::zero();
        keyboard_ = Keyboard::zero();
        rotary_knob_ = 0.0;
        rotary_knob_switch_ = rmcs_msgs::Switch::UNKNOWN;
    }

    struct [[gnu::packed]] Dr16DataPart1 {
        uint64_t joystick_channel0 : 11;
        uint64_t joystick_channel1 : 11;
        uint64_t joystick_channel2 : 11;
        uint64_t joystick_channel3 : 11;

        uint64_t switch_right : 2;
        uint64_t switch_left  : 2;

        uint64_t padding : 16;
    };
    static_assert(sizeof(Dr16DataPart1) == 8);
    std::atomic<uint64_t> data_part1_{std::bit_cast<uint64_t>(Dr16DataPart1{
        .joystick_channel0 = 1024,
        .joystick_channel1 = 1024,
        .joystick_channel2 = 1024,
        .joystick_channel3 = 1024,
        .switch_right = static_cast<uint64_t>(Switch::kUnknown),
        .switch_left = static_cast<uint64_t>(Switch::kUnknown),
        .padding = 0,
    })};
    static_assert(decltype(data_part1_)::is_always_lock_free);

    struct [[gnu::packed]] Dr16DataPart2 {
        int16_t mouse_velocity_x;
        int16_t mouse_velocity_y;
        int16_t mouse_velocity_z;

        bool mouse_left;
        bool mouse_right;
    };
    static_assert(sizeof(Dr16DataPart2) == 8);
    std::atomic<uint64_t> data_part2_{std::bit_cast<uint64_t>(Dr16DataPart2{
        .mouse_velocity_x = 0,
        .mouse_velocity_y = 0,
        .mouse_velocity_z = 0,
        .mouse_left = false,
        .mouse_right = false,
    })};
    static_assert(decltype(data_part2_)::is_always_lock_free);

    struct [[gnu::packed]] Dr16DataPart3 {
        Keyboard keyboard;
        uint16_t rotary_knob;
    };
    static_assert(sizeof(Dr16DataPart3) == 4);
    std::atomic<uint32_t> data_part3_ = {std::bit_cast<uint32_t>(Dr16DataPart3{
        .keyboard = Keyboard::zero(),
        .rotary_knob = 0,
    })};
    static_assert(decltype(data_part3_)::is_always_lock_free);

    Vector joystick_right_ = Vector::zero();
    Vector joystick_left_ = Vector::zero();

    Switch switch_right_ = Switch::kUnknown;
    Switch switch_left_ = Switch::kUnknown;

    Vector mouse_velocity_ = Vector::zero();
    double mouse_wheel_ = 0.0;

    Mouse mouse_ = Mouse::zero();
    Keyboard keyboard_ = Keyboard::zero();

    double rotary_knob_ = 0.0;
    rmcs_msgs::Switch rotary_knob_switch_ = rmcs_msgs::Switch::UNKNOWN;
    TimePoint last_remote_control_received_at_ = TimePoint::min();
    bool valid_ = false;
    bool timeout_enabled_ = true;
};

} // namespace rmcs_core::hardware::device
