#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include <rmcs_executor/component.hpp>
#include <rmcs_utility/crc/dji_crc.hpp>
#include <rmcs_utility/ring_buffer.hpp>
#include <rmcs_utility/tick_timer.hpp>

#include "referee/frame.hpp"

namespace rmcs_core::referee {

class ImageTransmissionParser {
public:
    explicit ImageTransmissionParser(rmcs_executor::Component& component) {
        component.register_output("/referee/image_transmission/custom", custom_data_);
        std::fill(custom_data_->begin(), custom_data_->end(), 0);
        custom_watchdog_.reset(5'000);

        component.register_output("/referee/image_transmission/vt13_frame", vt13_frame_data_);
        std::fill(vt13_frame_data_->begin(), vt13_frame_data_->end(), 0);
        vt13_watchdog_.reset(5'000);
    }

    void push_bytes(const std::byte* data, size_t size) {
        ring_buffer_.emplace_back_n(
            [&](std::byte* storage) noexcept { *storage = *data++; }, size);
    }

    void update() {
        while (true) {
            auto view = ring_buffer_.const_readable_view();
            if (view.empty())
                break;

            auto front = view[0];

            if (front == std::byte{0xa9}) {
                if (pop_and_parse_remote_control(view) == 0)
                    break;
            } else if (front == std::byte{0xa5}) {
                if (pop_and_parse_referee_frame(view) == 0)
                    break;
            } else {
                ring_buffer_.pop_front_n([](std::byte&&) noexcept {}, 1);
            }
        }

        tick_watchdogs();
    }

private:
    static constexpr size_t kRemoteControlFrameSize = 21;
    static constexpr size_t kRefereeFrameMaxSize    = 256;

    template <typename ViewType>
    size_t pop_and_parse_remote_control(const ViewType& view) {
        if (view.size() < kRemoteControlFrameSize)
            return 0;

        if (view[1] != std::byte{0x53}) {
            ring_buffer_.pop_front_n([](std::byte&&) noexcept {}, 1);
            return 1;
        }

        std::array<std::byte, kRemoteControlFrameSize> frame;
        std::copy_n(view.begin(), kRemoteControlFrameSize, frame.begin());

        if (!rmcs_utility::dji_crc::verify_crc16(frame.data(), kRemoteControlFrameSize)) {
            ring_buffer_.pop_front_n([](std::byte&&) noexcept {}, 1);
            return 1;
        }

        size_t rc_idx = 0;
        ring_buffer_.pop_front_n(
            [&](std::byte&& b) noexcept { frame[rc_idx++] = b; }, kRemoteControlFrameSize);

        std::copy_n(
            reinterpret_cast<const uint8_t*>(frame.data()), kRemoteControlFrameSize,
            vt13_frame_data_->begin());
        vt13_watchdog_.reset(500);

        return kRemoteControlFrameSize;
    }

    template <typename ViewType>
    size_t pop_and_parse_referee_frame(const ViewType& view) {
        if (view.size() < sizeof(FrameHeader))
            return 0;

        FrameHeader header;
        std::copy_n(
            view.begin(), sizeof(header), reinterpret_cast<std::byte*>(&header));

        if (!rmcs_utility::dji_crc::verify_crc8(header)) {
            ring_buffer_.pop_front_n([](std::byte&&) noexcept {}, 1);
            return 1;
        }

        size_t total_frame_size = sizeof(FrameHeader)
                                + sizeof(uint16_t)                      // command_id
                                + header.data_length + sizeof(uint16_t); // crc16

        if (total_frame_size > kRefereeFrameMaxSize) {
            ring_buffer_.pop_front_n([](std::byte&&) noexcept {}, 1);
            return 1;
        }

        if (view.size() < total_frame_size)
            return 0;

        std::array<std::byte, kRefereeFrameMaxSize> frame_buffer;
        std::copy_n(view.begin(), total_frame_size, frame_buffer.begin());

        if (!rmcs_utility::dji_crc::verify_crc16(frame_buffer.data(), total_frame_size)) {
            ring_buffer_.pop_front_n([](std::byte&&) noexcept {}, 1);
            return 1;
        }

        size_t rf_idx = 0;
        ring_buffer_.pop_front_n(
            [&](std::byte&& b) noexcept { frame_buffer[rf_idx++] = b; }, total_frame_size);

        uint16_t command_id;
        std::memcpy(
            &command_id, frame_buffer.data() + sizeof(FrameHeader), sizeof(uint16_t));

        if (command_id == 0x0302) {
            const auto* payload_ptr = reinterpret_cast<const uint8_t*>(
                frame_buffer.data() + sizeof(FrameHeader) + sizeof(uint16_t));
            const auto payload_size =
                std::min<size_t>(header.data_length, custom_data_->size());

            std::fill(custom_data_->begin(), custom_data_->end(), 0);
            std::copy_n(payload_ptr, payload_size, custom_data_->begin());
            custom_watchdog_.reset(500);
        }

        return total_frame_size;
    }

    void tick_watchdogs() {
        if (custom_watchdog_.tick())
            std::fill(custom_data_->begin(), custom_data_->end(), 0);

        if (vt13_watchdog_.tick())
            std::fill(vt13_frame_data_->begin(), vt13_frame_data_->end(), 0);
    }

    rmcs_utility::RingBuffer<std::byte> ring_buffer_{1024};

    rmcs_executor::Component::OutputInterface<std::array<uint8_t, 30>> custom_data_;
    rmcs_executor::Component::OutputInterface<std::array<uint8_t, 21>> vt13_frame_data_;

    rmcs_utility::TickTimer custom_watchdog_;
    rmcs_utility::TickTimer vt13_watchdog_;
};

} // namespace rmcs_core::referee
