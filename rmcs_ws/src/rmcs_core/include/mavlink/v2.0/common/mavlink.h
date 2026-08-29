#pragma once

#include <array>
#include <atomic>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <stdexcept>
#include <type_traits>

namespace mavlink {

inline constexpr std::uint8_t MAVLINK_STX = 0xFD;
inline constexpr std::size_t MAVLINK_MAX_PAYLOAD_LEN = 255;
inline constexpr std::size_t MAVLINK_MAX_PACKET_LEN = 280;

enum : std::uint8_t {
    MAV_TYPE_ONBOARD_CONTROLLER = 18,
    MAV_AUTOPILOT_INVALID = 8,
    MAV_STATE_ACTIVE = 4,
};

struct mavlink_message_t {
    std::uint8_t magic = MAVLINK_STX;
    std::uint8_t len = 0;
    std::uint8_t incompat_flags = 0;
    std::uint8_t compat_flags = 0;
    std::uint8_t seq = 0;
    std::uint8_t sysid = 0;
    std::uint8_t compid = 0;
    std::uint32_t msgid = 0;
    std::array<std::uint8_t, MAVLINK_MAX_PAYLOAD_LEN> payload{};
};

inline std::atomic<std::uint8_t> mavlink_sequence{0};

constexpr std::uint16_t mavlink_crc_accumulate(std::uint8_t data, std::uint16_t crc) noexcept {
    data ^= static_cast<std::uint8_t>(crc & 0xFFu);
    data ^= static_cast<std::uint8_t>(data << 4);
    return static_cast<std::uint16_t>(
        (crc >> 8) ^ (static_cast<std::uint16_t>(data) << 8)
        ^ (static_cast<std::uint16_t>(data) << 3) ^ (static_cast<std::uint16_t>(data) >> 4));
}

constexpr std::uint16_t mavlink_crc_calculate(const std::uint8_t* bytes, std::size_t size) noexcept {
    std::uint16_t crc = 0xFFFFu;
    for (std::size_t i = 0; i < size; ++i)
        crc = mavlink_crc_accumulate(bytes[i], crc);
    return crc;
}

template <typename T>
constexpr void mavlink_write_le(std::uint8_t* dst, T value) noexcept {
    static_assert(std::is_trivially_copyable_v<T>);
    const auto bytes = std::bit_cast<std::array<std::uint8_t, sizeof(T)>>(value);
    if constexpr (std::endian::native == std::endian::little) {
        std::memcpy(dst, bytes.data(), sizeof(T));
    } else {
        for (std::size_t i = 0; i < sizeof(T); ++i)
            dst[i] = bytes[sizeof(T) - 1 - i];
    }
}

inline void mavlink_pack_common(
    mavlink_message_t* msg, std::uint8_t system_id, std::uint8_t component_id, std::uint32_t msgid,
    std::uint8_t len) {
    if (len > MAVLINK_MAX_PAYLOAD_LEN)
        throw std::length_error("MAVLink payload too large");

    msg->magic = MAVLINK_STX;
    msg->len = len;
    msg->incompat_flags = 0;
    msg->compat_flags = 0;
    msg->sysid = system_id;
    msg->compid = component_id;
    msg->msgid = msgid;
}

inline void mavlink_msg_heartbeat_pack(
    std::uint8_t system_id, std::uint8_t component_id, mavlink_message_t* msg,
    std::uint8_t type, std::uint8_t autopilot, std::uint8_t base_mode,
    std::uint32_t custom_mode, std::uint8_t system_status) {
    mavlink_pack_common(msg, system_id, component_id, 0, 9);
    mavlink_write_le(msg->payload.data() + 0, custom_mode);
    msg->payload[4] = type;
    msg->payload[5] = autopilot;
    msg->payload[6] = base_mode;
    msg->payload[7] = system_status;
    msg->payload[8] = 3;
}

inline void mavlink_msg_vision_position_estimate_pack(
    std::uint8_t system_id, std::uint8_t component_id, mavlink_message_t* msg, std::uint64_t usec,
    float x, float y, float z, float roll, float pitch, float yaw,
    const float covariance[21], std::uint8_t reset_counter) {
    mavlink_pack_common(msg, system_id, component_id, 102, 117);
    mavlink_write_le(msg->payload.data() + 0, usec);
    mavlink_write_le(msg->payload.data() + 8, x);
    mavlink_write_le(msg->payload.data() + 12, y);
    mavlink_write_le(msg->payload.data() + 16, z);
    mavlink_write_le(msg->payload.data() + 20, roll);
    mavlink_write_le(msg->payload.data() + 24, pitch);
    mavlink_write_le(msg->payload.data() + 28, yaw);

    for (std::size_t i = 0; i < 21; ++i)
        mavlink_write_le(msg->payload.data() + 32 + i * 4, covariance[i]);

    msg->payload[116] = reset_counter;
}

inline std::uint16_t mavlink_msg_to_send_buffer(std::uint8_t* buffer, const mavlink_message_t* msg) {
    const auto seq = mavlink_sequence.fetch_add(1, std::memory_order_relaxed);

    std::size_t index = 0;
    buffer[index++] = MAVLINK_STX;
    buffer[index++] = msg->len;
    buffer[index++] = msg->incompat_flags;
    buffer[index++] = msg->compat_flags;
    buffer[index++] = seq;
    buffer[index++] = msg->sysid;
    buffer[index++] = msg->compid;
    buffer[index++] = static_cast<std::uint8_t>(msg->msgid & 0xFFu);
    buffer[index++] = static_cast<std::uint8_t>((msg->msgid >> 8) & 0xFFu);
    buffer[index++] = static_cast<std::uint8_t>((msg->msgid >> 16) & 0xFFu);

    std::memcpy(buffer + index, msg->payload.data(), msg->len);
    index += msg->len;

    std::uint16_t crc = 0xFFFFu;
    crc = mavlink_crc_accumulate(buffer[1], crc);
    crc = mavlink_crc_accumulate(buffer[2], crc);
    crc = mavlink_crc_accumulate(buffer[3], crc);
    crc = mavlink_crc_accumulate(buffer[4], crc);
    crc = mavlink_crc_accumulate(buffer[5], crc);
    crc = mavlink_crc_accumulate(buffer[6], crc);
    crc = mavlink_crc_accumulate(buffer[7], crc);
    crc = mavlink_crc_accumulate(buffer[8], crc);
    crc = mavlink_crc_accumulate(buffer[9], crc);
    for (std::size_t i = 0; i < msg->len; ++i)
        crc = mavlink_crc_accumulate(buffer[10 + i], crc);

    switch (msg->msgid) {
    case 0: crc = mavlink_crc_accumulate(50, crc); break;
    case 102: crc = mavlink_crc_accumulate(158, crc); break;
    default: throw std::invalid_argument("Unsupported MAVLink message id");
    }

    buffer[index++] = static_cast<std::uint8_t>(crc & 0xFFu);
    buffer[index++] = static_cast<std::uint8_t>(crc >> 8);
    return static_cast<std::uint16_t>(index);
}

} // namespace mavlink

using mavlink::MAV_AUTOPILOT_INVALID;
using mavlink::MAVLINK_MAX_PACKET_LEN;
using mavlink::MAV_STATE_ACTIVE;
using mavlink::MAV_TYPE_ONBOARD_CONTROLLER;
using mavlink::mavlink_message_t;
using mavlink::mavlink_msg_heartbeat_pack;
using mavlink::mavlink_msg_to_send_buffer;
using mavlink::mavlink_msg_vision_position_estimate_pack;
