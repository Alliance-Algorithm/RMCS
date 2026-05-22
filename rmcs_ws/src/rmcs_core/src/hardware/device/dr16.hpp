#pragma once

#include <atomic>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include <eigen3/Eigen/Dense>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::hardware::device {

class Dr16 {
public:
    Dr16() = default;

    void store_status(std::span<const std::byte> uart_data) {
        if (uart_data.size() != kStatusSize)
            return;

        last_receive_time_.store(Clock::now(), std::memory_order_relaxed);

        auto* cursor = uart_data.data();

        uint64_t part1{};
        std::memcpy(&part1, cursor, kPart1Size);
        cursor += kPart1Size;
        data_part1_.store(part1, std::memory_order::relaxed);

        uint64_t part2{};
        std::memcpy(&part2, cursor, kPart2Size);
        cursor += kPart2Size;
        data_part2_.store(part2, std::memory_order::relaxed);

        uint32_t part3{};
        std::memcpy(&part3, cursor, kPart3Size);
        data_part3_.store(part3, std::memory_order::relaxed);
    }

    void update_status() {
        const auto raw_part1 = data_part1_.load(std::memory_order::relaxed);
        const auto part1 = std::bit_cast<Dr16DataPart1>(raw_part1);

        joystick_right_ = {
            channel_to_double(static_cast<uint16_t>(part1.joystick_channel1)),
            -channel_to_double(static_cast<uint16_t>(part1.joystick_channel0)),
        };
        joystick_left_ = {
            channel_to_double(static_cast<uint16_t>(part1.joystick_channel3)),
            -channel_to_double(static_cast<uint16_t>(part1.joystick_channel2)),
        };

        switch_right_ = static_cast<rmcs_msgs::Switch>(part1.switch_right);
        switch_left_ = static_cast<rmcs_msgs::Switch>(part1.switch_left);

        const auto raw_part2 = data_part2_.load(std::memory_order::relaxed);
        const auto part2 = std::bit_cast<Dr16DataPart2>(raw_part2);

        mouse_velocity_ = {
            -static_cast<double>(part2.mouse_velocity_y) / 32768.0,
            -static_cast<double>(part2.mouse_velocity_x) / 32768.0,
        };
        mouse_wheel_ = -static_cast<double>(part2.mouse_velocity_z) / 32768.0;
        mouse_ = {
            .left = part2.mouse_left,
            .right = part2.mouse_right,
        };

        const auto raw_part3 = data_part3_.load(std::memory_order::relaxed);
        const auto part3 = std::bit_cast<Dr16DataPart3>(raw_part3);

        keyboard_ = std::bit_cast<rmcs_msgs::Keyboard>(part3.keyboard);
        rotary_knob_ = channel_to_double(part3.rotary_knob);
        update_rotary_knob_switch();
    }

    [[nodiscard]] const Eigen::Vector2d& joystick_right() const noexcept { return joystick_right_; }

    [[nodiscard]] bool valid() const noexcept {
        const auto last_receive_time = last_receive_time_.load(std::memory_order_relaxed);
        return last_receive_time != TimePoint::min()
            && Clock::now() - last_receive_time <= kFreshTimeout;
    }

    [[nodiscard]] const Eigen::Vector2d& joystick_left() const noexcept { return joystick_left_; }

    [[nodiscard]] rmcs_msgs::Switch switch_right() const noexcept { return switch_right_; }

    [[nodiscard]] rmcs_msgs::Switch switch_left() const noexcept { return switch_left_; }

    [[nodiscard]] const Eigen::Vector2d& mouse_velocity() const noexcept { return mouse_velocity_; }

    [[nodiscard]] rmcs_msgs::Mouse mouse() const noexcept { return mouse_; }

    [[nodiscard]] rmcs_msgs::Keyboard keyboard() const noexcept { return keyboard_; }

    [[nodiscard]] double rotary_knob() const noexcept { return rotary_knob_; }

    [[nodiscard]] double mouse_wheel() const noexcept { return mouse_wheel_; }

    [[nodiscard]] rmcs_msgs::Switch rotary_knob_switch() const noexcept {
        return rotary_knob_switch_;
    }

private:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    static constexpr auto kFreshTimeout = std::chrono::milliseconds(500);

    static constexpr std::size_t kPart1Size = 6;
    static constexpr std::size_t kPart2Size = 8;
    static constexpr std::size_t kPart3Size = 4;
    static constexpr std::size_t kStatusSize = kPart1Size + kPart2Size + kPart3Size;

    struct [[gnu::packed]] Dr16DataPart1 {
        uint64_t joystick_channel0 : 11;
        uint64_t joystick_channel1 : 11;
        uint64_t joystick_channel2 : 11;
        uint64_t joystick_channel3 : 11;
        uint64_t switch_right : 2;
        uint64_t switch_left : 2;
        uint64_t padding : 16;
    };
    static_assert(sizeof(Dr16DataPart1) == 8);

    struct [[gnu::packed]] Dr16DataPart2 {
        int16_t mouse_velocity_x;
        int16_t mouse_velocity_y;
        int16_t mouse_velocity_z;
        bool mouse_left;
        bool mouse_right;
    };
    static_assert(sizeof(Dr16DataPart2) == 8);

    struct [[gnu::packed]] Dr16DataPart3 {
        uint16_t keyboard;
        uint16_t rotary_knob;
    };
    static_assert(sizeof(Dr16DataPart3) == 4);

    static double channel_to_double(int32_t value) {
        value -= 1024;
        if (-660 <= value && value <= 660)
            return value / 660.0;
        return 0.0;
    }

    void update_rotary_knob_switch() {
        constexpr double divider = 0.7;
        constexpr double anti_shake_shift = 0.05;

        double upper_divider = divider;
        double lower_divider = -divider;
        if (rotary_knob_switch_ == rmcs_msgs::Switch::UP) {
            upper_divider -= anti_shake_shift;
            lower_divider -= anti_shake_shift;
        } else if (rotary_knob_switch_ == rmcs_msgs::Switch::MIDDLE) {
            upper_divider += anti_shake_shift;
            lower_divider -= anti_shake_shift;
        } else if (rotary_knob_switch_ == rmcs_msgs::Switch::DOWN) {
            upper_divider += anti_shake_shift;
            lower_divider += anti_shake_shift;
        }

        const auto knob_value = -rotary_knob_;
        if (knob_value > upper_divider) {
            rotary_knob_switch_ = rmcs_msgs::Switch::UP;
        } else if (knob_value < lower_divider) {
            rotary_knob_switch_ = rmcs_msgs::Switch::DOWN;
        } else {
            rotary_knob_switch_ = rmcs_msgs::Switch::MIDDLE;
        }
    }

    std::atomic<uint64_t> data_part1_{std::bit_cast<uint64_t>(Dr16DataPart1{
        .joystick_channel0 = 1024,
        .joystick_channel1 = 1024,
        .joystick_channel2 = 1024,
        .joystick_channel3 = 1024,
        .switch_right = static_cast<uint64_t>(rmcs_msgs::Switch::UNKNOWN),
        .switch_left = static_cast<uint64_t>(rmcs_msgs::Switch::UNKNOWN),
        .padding = 0,
    })};
    static_assert(decltype(data_part1_)::is_always_lock_free);

    std::atomic<uint64_t> data_part2_{std::bit_cast<uint64_t>(Dr16DataPart2{
        .mouse_velocity_x = 0,
        .mouse_velocity_y = 0,
        .mouse_velocity_z = 0,
        .mouse_left = false,
        .mouse_right = false,
    })};
    static_assert(decltype(data_part2_)::is_always_lock_free);

    std::atomic<uint32_t> data_part3_{std::bit_cast<uint32_t>(Dr16DataPart3{
        .keyboard = 0,
        .rotary_knob = 0,
    })};
    static_assert(decltype(data_part3_)::is_always_lock_free);

    std::atomic<TimePoint> last_receive_time_{TimePoint::min()};

    Eigen::Vector2d joystick_right_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d joystick_left_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d mouse_velocity_ = Eigen::Vector2d::Zero();

    rmcs_msgs::Switch switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch rotary_knob_switch_ = rmcs_msgs::Switch::UNKNOWN;

    rmcs_msgs::Mouse mouse_ = rmcs_msgs::Mouse::zero();
    rmcs_msgs::Keyboard keyboard_ = rmcs_msgs::Keyboard::zero();

    double rotary_knob_ = 0.0;
    double mouse_wheel_ = 0.0;
};

} // namespace rmcs_core::hardware::device