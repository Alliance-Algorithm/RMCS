#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/crc/dji_crc.hpp>

#include "referee/frame.hpp"

namespace rmcs_core::hardware::vtm {

class ImagePacketTransmit {

public:
    using UartWriter = std::function<void(const std::byte*, size_t)>;

    ImagePacketTransmit(
        rmcs_executor::Component& component, std::chrono::milliseconds interval,
        UartWriter uart_writer, rclcpp::Logger logger,
        const std::string& image_packets_background_name = "/vtm_link/background_imagepackages",
        const std::string& image_seq_background_name = "/vtm_link/background_imagepackage_seq",
        const std::string& image_packets_trajectory_name = "/vtm_link/trajectory_imagepackages",
        const std::string& image_seq_trajectory_name = "/vtm_link/trajectory_imagepackage_seq")
        : uart_writer_(std::move(uart_writer))
        , interval_(interval)
        , logger_(std::move(logger)) {
        component.register_input(image_packets_background_name, image_packets_background_, false);
        component.register_input(image_seq_background_name, image_seq_background_, false);
        component.register_input(image_packets_trajectory_name, image_packets_trajectory_, false);
        component.register_input(image_seq_trajectory_name, image_seq_trajectory_, false);
    }

    void command_update() {
        if (image_packets_background_.ready() && image_seq_background_.ready()) {
            int seq = *image_seq_background_;
            if (seq != last_seq_background_) {
                last_seq_background_ = seq;
                pending_queue_.emplace_back(
                    image_packets_background_->begin(), image_packets_background_->end());
                RCLCPP_INFO(
                    logger_, "received background packets seq=%d, count=%zu", seq,
                    pending_queue_.back().size());
            }
        }

        if (image_packets_trajectory_.ready() && image_seq_trajectory_.ready()) {
            int seq = *image_seq_trajectory_;
            if (seq != last_seq_trajectory_) {
                last_seq_trajectory_ = seq;
                pending_queue_.emplace_back(
                    image_packets_trajectory_->begin(), image_packets_trajectory_->end());
                RCLCPP_INFO(
                    logger_, "received trajectory packets seq=%d, count=%zu", seq,
                    pending_queue_.back().size());
            }
        }

        auto now = std::chrono::steady_clock::now();

        if (publish_index_ >= publish_buffer_.size() && !pending_queue_.empty()) {
            publish_buffer_ = std::move(pending_queue_.front());
            pending_queue_.pop_front();
            publish_index_ = 0;
            next_publish_time_ = now + interval_;
            RCLCPP_INFO(logger_, "start transmit buffer, %zu packets", publish_buffer_.size());
        }

        if (publish_index_ >= publish_buffer_.size())
            return;
        if (now < *next_publish_time_)
            return;

        publish_single_packet(publish_buffer_[publish_index_]);
        RCLCPP_INFO(
            logger_, "uart sent packet %zu/%zu", publish_index_ + 1, publish_buffer_.size());
        next_publish_time_ = now + interval_;
        ++publish_index_;
        if (publish_index_ >= publish_buffer_.size())
            next_publish_time_.reset();
    }

private:
    static constexpr uint16_t kImageCmdId = 0x0310;
    static constexpr size_t kImageDataSize = 300;

    void publish_single_packet(const std::array<std::uint8_t, kImageDataSize>& image_data) {
        static constexpr size_t kHeaderSize = sizeof(referee::FrameHeader);
        static constexpr size_t kCmdIdSize = sizeof(uint16_t);
        static constexpr size_t kCrc16Size = sizeof(uint16_t);
        static constexpr size_t kFrameSize = kHeaderSize + kCmdIdSize + kImageDataSize + kCrc16Size;

        referee::Frame frame;
        frame.header.sof = referee::sof_value;
        frame.header.data_length = kImageDataSize;
        frame.header.sequence = sequence_++;
        frame.header.crc8 = 0;
        frame.body.command_id = kImageCmdId;
        std::memcpy(frame.body.data, image_data.data(), kImageDataSize);

        rmcs_utility::dji_crc::append_crc8(frame.header);
        rmcs_utility::dji_crc::append_crc16(&frame, kFrameSize);

        uart_writer_(reinterpret_cast<const std::byte*>(&frame), kFrameSize);
    }

    using ImagePackets = std::vector<std::array<std::uint8_t, kImageDataSize>>;

    rmcs_executor::Component::InputInterface<ImagePackets> image_packets_background_;
    rmcs_executor::Component::InputInterface<int> image_seq_background_;
    rmcs_executor::Component::InputInterface<ImagePackets> image_packets_trajectory_;
    rmcs_executor::Component::InputInterface<int> image_seq_trajectory_;
    UartWriter uart_writer_;
    std::chrono::milliseconds interval_;
    std::optional<std::chrono::steady_clock::time_point> next_publish_time_;
    uint8_t sequence_{0};
    rclcpp::Logger logger_;

    std::deque<ImagePackets> pending_queue_;
    ImagePackets publish_buffer_;
    size_t publish_index_{0};
    int last_seq_background_{-1};
    int last_seq_trajectory_{-1};
};

} // namespace rmcs_core::hardware::vtm
