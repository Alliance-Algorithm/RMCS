#pragma once

#include <cstddef>
#include <cstdint>

namespace rmcs_core::referee {

constexpr uint8_t sof_value            = 0xa5;
constexpr size_t frame_data_max_length = 1024;

struct __attribute__((packed)) FrameHeader {
    uint8_t sof;
    uint16_t data_length;
    uint8_t sequence;
    uint8_t crc8;
};

struct __attribute__((packed)) FrameBody {
    uint16_t command_id;
    std::byte data[frame_data_max_length];
};

struct __attribute__((packed)) Frame {
    FrameHeader header;
    FrameBody body;
};

} // namespace rmcs_core::referee