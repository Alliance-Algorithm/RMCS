#pragma once

#include <cstdint>

namespace rmcs_msgs {
enum class WheelLegMode : uint8_t {
    STOP = 0,
    SPIN = 1,
    FOLLOW = 2,
    LAUNCH_RAMP = 3,
    BALANCELESS = 4,
};

}
