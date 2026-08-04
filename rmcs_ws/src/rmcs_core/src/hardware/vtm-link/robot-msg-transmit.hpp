#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <optional>

#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/crc/dji_crc.hpp>

#include "referee/frame.hpp"

namespace rmcs_core::hardware::vtm {

class RobotMsgTransmit {
public:
    using UartWriter = std::function<void(const std::byte*, size_t)>;

    RobotMsgTransmit(
        rmcs_executor::Component& component, std::chrono::milliseconds interval,
        UartWriter uart_writer, rclcpp::Logger logger)
        : uart_writer_(std::move(uart_writer))
        , interval_(interval)
        , logger_(std::move(logger)) {
        component.register_input("/gimbal/pitch/raw_angle", pitch_raw_angle_, false);
        component.register_input("/gimbal/top_yaw/raw_angle", top_yaw_raw_angle_, false);
        component.register_input(
            "/gimbal/first_front_friction/control_velocity", front_friction_control_, false);
        component.register_input(
            "/gimbal/first_back_friction/control_velocity", back_friction_control_, false);
        component.register_input(
            "/gimbal/first_front_friction/velocity", friction_velocity_[0], false);
        component.register_input(
            "/gimbal/second_front_friction/velocity", friction_velocity_[1], false);
        component.register_input(
            "/gimbal/third_front_friction/velocity", friction_velocity_[2], false);
        component.register_input(
            "/gimbal/first_back_friction/velocity", friction_velocity_[3], false);
        component.register_input(
            "/gimbal/second_back_friction/velocity", friction_velocity_[4], false);
        component.register_input(
            "/gimbal/third_back_friction/velocity", friction_velocity_[5], false);
    }

    void command_update() {
        auto now = std::chrono::steady_clock::now();
        if (next_publish_time_ && now < *next_publish_time_)
            return;

        pack_and_send();
        next_publish_time_ = now + interval_;

        if (!next_log_time_ || now >= *next_log_time_) {
            int32_t target_speed = 0;
            double front_ctrl = 0.0;
            bool front_ready = front_friction_control_.ready();
            if (front_ready) {
                front_ctrl = *front_friction_control_;
                if (front_ctrl == nan_) {
                    target_speed = 0;
                } else {
                    target_speed = front_ctrl < 420.0 ? 12 : 16;
                }
            } else {
                target_speed = 0;
            }
            double back_ctrl = back_friction_control_.ready() ? *back_friction_control_ : 0.0;
            RCLCPP_INFO(
                logger_, "target=%d m/s, front=%.0f rad/s, back=%.0f rad/s", target_speed,
                front_ready ? front_ctrl : NAN, back_friction_control_.ready() ? back_ctrl : NAN);
            next_log_time_ = now + kLogInterval;
        }
    }

private:
    static constexpr uint8_t kMsgType = 0x05;
    static constexpr uint16_t kCmdId = 0x0310;
    static constexpr size_t kDataSize = 300;

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr int32_t kNaN = INT32_MIN;
    static constexpr std::chrono::seconds kLogInterval{5};

    void pack_and_send() {
        std::array<uint8_t, kDataSize> packet{};
        packet[0] = kMsgType;

        write_int32(packet, 1, pitch_raw_angle_, [](auto& i) { return static_cast<int32_t>(*i); });
        write_int32(
            packet, 5, top_yaw_raw_angle_, [](auto& i) { return static_cast<int32_t>(*i); });
        if (front_friction_control_.ready()) {
            double v = *front_friction_control_;
            int32_t target = v < 420.0 ? 12 : 16;
            std::memcpy(&packet[9], &target, sizeof(target));
        }
        write_int32(packet, 13, front_friction_control_, [](auto& i) {
            return static_cast<int32_t>(std::lround(*i));
        });
        write_int32(packet, 17, back_friction_control_, [](auto& i) {
            return static_cast<int32_t>(std::lround(*i));
        });
        write_int32(packet, 21, friction_velocity_[0], [](auto& i) {
            return static_cast<int32_t>(std::lround(*i));
        });
        write_int32(packet, 25, friction_velocity_[1], [](auto& i) {
            return static_cast<int32_t>(std::lround(*i));
        });
        write_int32(packet, 29, friction_velocity_[2], [](auto& i) {
            return static_cast<int32_t>(std::lround(*i));
        });
        write_int32(packet, 33, friction_velocity_[3], [](auto& i) {
            return static_cast<int32_t>(std::lround(*i));
        });
        write_int32(packet, 37, friction_velocity_[4], [](auto& i) {
            return static_cast<int32_t>(std::lround(*i));
        });
        write_int32(packet, 41, friction_velocity_[5], [](auto& i) {
            return static_cast<int32_t>(std::lround(*i));
        });

        send_frame(packet);
    }

    template <typename Input, typename Func>
    void write_int32(
        std::array<uint8_t, kDataSize>& packet, size_t offset, Input& input, Func func) {
        const int32_t value = input.ready() ? func(input) : kNaN;
        std::memcpy(&packet[offset], &value, sizeof(value));
    }

    void send_frame(const std::array<uint8_t, kDataSize>& data) {
        static constexpr size_t kHeaderSize = sizeof(referee::FrameHeader);
        static constexpr size_t kCmdIdSize = sizeof(uint16_t);
        static constexpr size_t kCrc16Size = sizeof(uint16_t);
        static constexpr size_t kFrameSize = kHeaderSize + kCmdIdSize + kDataSize + kCrc16Size;

        referee::Frame frame;
        frame.header.sof = referee::sof_value;
        frame.header.data_length = kDataSize;
        frame.header.sequence = sequence_++;
        frame.header.crc8 = 0;
        frame.body.command_id = kCmdId;
        std::memcpy(frame.body.data, data.data(), kDataSize);

        rmcs_utility::dji_crc::append_crc8(frame.header);
        rmcs_utility::dji_crc::append_crc16(&frame, kFrameSize);

        uart_writer_(reinterpret_cast<const std::byte*>(&frame), kFrameSize);
    }

    rmcs_executor::Component::InputInterface<int64_t> pitch_raw_angle_;
    rmcs_executor::Component::InputInterface<int64_t> top_yaw_raw_angle_;
    rmcs_executor::Component::InputInterface<double> front_friction_control_;
    rmcs_executor::Component::InputInterface<double> back_friction_control_;
    rmcs_executor::Component::InputInterface<double> friction_velocity_[6];

    UartWriter uart_writer_;
    std::chrono::milliseconds interval_;
    uint8_t sequence_{0};
    rclcpp::Logger logger_;
    std::optional<std::chrono::steady_clock::time_point> next_publish_time_;
    std::optional<std::chrono::steady_clock::time_point> next_log_time_;
};

} // namespace rmcs_core::hardware::vtm
