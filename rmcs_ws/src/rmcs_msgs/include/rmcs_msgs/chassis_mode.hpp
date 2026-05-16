#pragma once
#include <cstdint>

namespace rmcs_msgs {

enum class ChassisMode : std::uint8_t {
    AUTO = 0,
    SPIN_FAST,
    SPIN_SLOW,
    STEP_DOWN,
    LAUNCH_RAMP,
    ALIGNMENT,
};

} // namespace rmcs_msgs
