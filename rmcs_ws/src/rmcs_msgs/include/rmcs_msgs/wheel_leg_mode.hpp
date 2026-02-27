#pragma once

#include <cstdint>

namespace rmcs_msgs {
enum class WheelLegMode : uint8_t {
    STOP = 0,
    SPIN = 1,
    SPIN_2_FOLLOW = 2,
    STEP_DOWN = 3,
    LAUNCH_RAMP = 4,
    BALANCELESS = 5,
};

}
