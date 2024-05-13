#pragma once

#include <cstdint>

namespace rmcs_core::hardware::referee::package {

struct __attribute__((packed)) FrameHeader {
    const uint8_t start = 0xA5;
    uint16_t length;
    uint8_t sequence;
    uint8_t crc8;
};

template <typename SubPackageT>
struct __attribute__((packed)) Package {
    FrameHeader header;
    uint16_t command;
    SubPackageT data;
    uint16_t crc16;
};
} // namespace rmcs_core::hardware::referee::package
