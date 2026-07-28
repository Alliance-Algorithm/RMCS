#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <utility>

#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/crc/dji_crc.hpp>

#include "referee/frame.hpp"

namespace rmcs_core::hardware::vtm {

class LadarPackageTransmit {
public:
    using UartWriter = std::function<void(const std::byte*, size_t)>;
    using LidarMsgBroadcast = std::array<std::uint8_t, 118>;

    LadarPackageTransmit(
        rmcs_executor::Component& component, std::chrono::milliseconds interval,
        UartWriter uart_writer, rclcpp::Logger logger,
        const std::string& lidar_msg_broadcast_name =
            "/referee/multi_robot_communication/lidar_msg_broadcast")
        : uart_writer_(std::move(uart_writer))
        , interval_(interval)
        , logger_(std::move(logger)) {
        component.register_input(lidar_msg_broadcast_name, lidar_msg_broadcast_, false);
    }

    void command_update() {
        if (!lidar_msg_broadcast_.ready())
            return;

        auto now = std::chrono::steady_clock::now();
        if (now < next_publish_time_)
            return;

        publish_single_packet(*lidar_msg_broadcast_);
        next_publish_time_ = now + interval_;
    }

private:
    static constexpr uint16_t kLadarCmdId = 0x0310;
    static constexpr size_t kLadarDataSize = 118;

    void publish_single_packet(const LidarMsgBroadcast& lidar_msg_broadcast) {
        static constexpr size_t kHeaderSize = sizeof(referee::FrameHeader);
        static constexpr size_t kCmdIdSize = sizeof(uint16_t);
        static constexpr size_t kCrc16Size = sizeof(uint16_t);
        static constexpr size_t kFrameSize = kHeaderSize + kCmdIdSize + kLadarDataSize
                                           + kCrc16Size;

        referee::Frame frame;
        frame.header.sof = referee::sof_value;
        frame.header.data_length = kLadarDataSize;
        frame.header.sequence = sequence_++;
        frame.header.crc8 = 0;
        frame.body.command_id = kLadarCmdId;
        std::memcpy(frame.body.data, lidar_msg_broadcast.data(), kLadarDataSize);

        rmcs_utility::dji_crc::append_crc8(frame.header);
        rmcs_utility::dji_crc::append_crc16(&frame, kFrameSize);

        uart_writer_(reinterpret_cast<const std::byte*>(&frame), kFrameSize);
        RCLCPP_DEBUG(logger_, "uart sent ladar packet");
    }

    rmcs_executor::Component::InputInterface<LidarMsgBroadcast> lidar_msg_broadcast_;
    UartWriter uart_writer_;
    std::chrono::milliseconds interval_;
    std::chrono::steady_clock::time_point next_publish_time_ =
        std::chrono::steady_clock::time_point::min();
    uint8_t sequence_{0};
    rclcpp::Logger logger_;
};

} // namespace rmcs_core::hardware::vtm
