#pragma once

#include <cstdint>

namespace rmcs_core::hardware::referee::package::send {

/// @brief 0x0308
/// @note robot -> player client
/// @note map display | 3hz
/// @note length 34
namespace robot_map {}

/// @brief 0x0307
/// @note sentry -> player client
/// @note map display | 1hz
/// @note length 103
namespace sentry_map {}

/// @brief 0x0305
/// @note lidar -> server -> player client
/// @note map display | 10hz
/// @note length 10
namespace lidar_map {}

/// @brief 0x0301
/// @note robot -> robot | ui
/// @note interact with robot | 30hz
namespace interact {

template <typename SubPackageT>
struct __attribute__((packed)) Data {
    uint16_t command;
    uint16_t sender;
    uint16_t receiver;
    SubPackageT data;
};

/// @brief chat with other robot
/// @note data length <= 112
/// @note id 0x0200 ~ 0x02FF
namespace chat {}

/// @brief data for sentry
/// @note data length equals 4
/// @note id 0x0120
namespace sentry {}

/// @brief data for lidar
/// @note data length equals 1
/// @note id 0x0121
namespace lidar {}

/// @brief data for ui
/// @note id 0x0100 ~ 0x0104 and 0x0110
namespace ui {
// 0x0100
struct __attribute__((packed)) DeleteLayer {
    uint8_t delete_type;
    uint8_t layer;
};

// 0x0101
struct __attribute__((packed)) Description {
    uint8_t name[3];
    uint8_t operate    : 3;
    uint8_t graphic    : 3;
    uint8_t layer      : 4;
    uint8_t color      : 4;
    uint16_t details_a : 9;
    uint16_t details_b : 9;
    uint16_t width     : 10;
    uint16_t start_x   : 11;
    uint16_t start_y   : 11;
    uint16_t details_c : 10;
    uint16_t details_d : 11;
    uint16_t details_e : 11;
};

// 0x0102
struct __attribute__((packed)) Description2 {
    Description description[2];
};

// 0x0103
struct __attribute__((packed)) Description5 {
    Description description[5];
};

// 0x0104
struct __attribute__((packed)) Description7 {
    Description description[7];
};

// 0x0110
struct __attribute__((packed)) String {
    Description description;
    uint8_t data[30];
};
} // namespace ui
} // namespace interact
} // namespace rmcs_core::hardware::referee::package::send
