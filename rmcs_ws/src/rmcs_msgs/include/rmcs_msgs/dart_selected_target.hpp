#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class DartSelectedTarget : uint8_t {
    OUTPOST = 0,
    BASE = 1,
    UNKNOWN = UINT8_MAX,
};

} // namespace rmcs_msgs
