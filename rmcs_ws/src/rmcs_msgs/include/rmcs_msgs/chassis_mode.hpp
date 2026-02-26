#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class ChassisMode : uint8_t {
    AUTO        = 0,
    SPIN        = 1,
    STEP_DOWN   = 2,
    LAUNCH_RAMP = 3,
    Flow        = 4,
    Yaw_Free    = 5,
    None        = 6,
    Up_Stairs   = 7,
};

} // namespace rmcs_msgs
