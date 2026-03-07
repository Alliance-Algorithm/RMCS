#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class ChassisMode : uint8_t {
    SPIN = 1,
    Flow,            
    Yaw_Free,
    None,
};

} // namespace rmcs_msgs