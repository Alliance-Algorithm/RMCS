#pragma once

#include <cstdint>

namespace rmcs_msgs {
enum class WheelLegMode : uint8_t {
    BALANCELESS = 0,
    SPIN = 1,
    FOLLOW = 2,
    LAUNCH_RAMP = 3,
    RESCUE_TIP_OVER = 4,
};
}
