#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class LkmotorStatus : uint8_t {
    UNKNOWN  = 0,
    REQUEST  = 1,
    DISABLE  = 2,
    START_UP = 3,
    ENABLE   = 4,
};

}