#pragma once

#include <cstddef>
#include <cstring>
#include <span>
#include <variant>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/crc/dji_crc.hpp>
#include <rmcs_utility/ring_buffer.hpp>

namespace rmcs_core::hardware::device {

class Vt13 {
public:
    enum class ModeSwitch : uint8_t {
        kUnknown = 0,
        kCine = 1,
        kNormal = 2,
        kSport = 3,
    };

    Vt13() = default;

    void store_status(std::span<const std::byte> uart_data) {
        const auto written = data_buffer_.emplace_back_n(
            [iter = uart_data.cbegin()](std::byte* storage) mutable noexcept {
                *storage = *iter++;
            },
            uart_data.size());
        if (written != uart_data.size()) {
            RCLCPP_WARN(
                rclcpp::get_logger("vt13"), "VT13 input buffer overflow: dropped %zu of %zu bytes",
                uart_data.size() - written, uart_data.size());
        }
    }

    void update_status() {
        auto readable = data_buffer_.readable();
        if (!readable)
            return;

        while (readable) {
            ReadResult result = VerificationFailed{};

            const std::byte front = *data_buffer_.peek_front();
            if (front == std::byte{0xa9})
                result = read_remote_control_data(readable);
            else if (front == std::byte(0xa5))
                result = read_referee_style_data(readable);

            if (std::holds_alternative<Incomplete>(result)) {
                break;
            }
            if (std::holds_alternative<VerificationFailed>(result)) {
                data_buffer_.pop_front([](std::byte&&) noexcept {});
                readable--;
                continue;
            }
            if (std::holds_alternative<Success>(result)) {
                readable -= std::get<Success>(result).read;
                continue;
            }
        }
    }

    [[nodiscard]] ModeSwitch mode_switch() const noexcept { return mode_switch_; }

    [[nodiscard]] const Eigen::Vector2d& mouse_velocity() const noexcept { return mouse_velocity_; }
    [[nodiscard]] double mouse_wheel() const noexcept { return mouse_wheel_; }

    [[nodiscard]] rmcs_msgs::Mouse mouse() const noexcept { return mouse_; }
    [[nodiscard]] rmcs_msgs::Keyboard keyboard() const noexcept { return keyboard_; }

private:
    struct Incomplete {};
    struct VerificationFailed {};
    struct Success {
        std::size_t read;
    };
    using ReadResult = std::variant<Incomplete, VerificationFailed, Success>;

    struct [[gnu::packed]] RemoteControlData {
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

    struct [[gnu::packed]] RefereeFrameHeader {
        uint8_t sof;
        uint16_t data_length;
        uint8_t seq;
        uint8_t crc8;
    };

    ReadResult read_remote_control_data(const std::size_t readable) {
        if (readable < sizeof(RemoteControlData))
            return Incomplete{};

        RemoteControlData data;
        data_buffer_.peek_front_n(
            [dst = reinterpret_cast<std::byte*>(&data)](std::byte src) mutable noexcept {
                *dst++ = src;
            },
            sizeof(RemoteControlData));

        if (data.header != RemoteControlData::kHeaderMagic
            || !rmcs_utility::dji_crc::verify_crc16(data))
            return VerificationFailed{};

        data_buffer_.pop_front_n([](std::byte&&) noexcept {}, sizeof(RemoteControlData));

        update_remote_control_data(data);
        return Success{sizeof(RemoteControlData)};
    }

    void update_remote_control_data(const RemoteControlData& data) {
        mode_switch_ = static_cast<ModeSwitch>(data.mode_switch + 1);

        mouse_velocity_ = {
            -data.mouse_velocity_y / 32768.0,
            -data.mouse_velocity_x / 32768.0,
        };
        mouse_wheel_ = -static_cast<double>(data.mouse_velocity_z) / 32768.0;

        mouse_ = {
            .left = static_cast<bool>(data.mouse_left),
            .right = static_cast<bool>(data.mouse_right),
        };
        keyboard_ = std::bit_cast<rmcs_msgs::Keyboard>(data.keyboard);
    }

    ReadResult read_referee_style_data(const std::size_t readable) {
        if (readable < sizeof(RefereeFrameHeader))
            return Incomplete{};

        RefereeFrameHeader header;
        data_buffer_.peek_front_n(
            [dst = reinterpret_cast<std::byte*>(&header)](std::byte src) mutable noexcept {
                *dst++ = src;
            },
            sizeof(RefereeFrameHeader));

        if (!rmcs_utility::dji_crc::verify_crc8(header))
            return VerificationFailed{};

        const std::size_t total_frame_size =
            sizeof(RefereeFrameHeader) + 2 + header.data_length + 2;
        if (readable < total_frame_size)
            return Incomplete{};

        data_buffer_.pop_front_n([](std::byte&&) noexcept {}, total_frame_size);
        return Success{total_frame_size};
    }

    rmcs_utility::RingBuffer<std::byte> data_buffer_{1024};

    ModeSwitch mode_switch_ = ModeSwitch::kUnknown;

    Eigen::Vector2d mouse_velocity_ = Eigen::Vector2d::Zero();
    double mouse_wheel_ = 0;

    rmcs_msgs::Mouse mouse_ = rmcs_msgs::Mouse::zero();
    rmcs_msgs::Keyboard keyboard_ = rmcs_msgs::Keyboard::zero();
};

} // namespace rmcs_core::hardware::device
