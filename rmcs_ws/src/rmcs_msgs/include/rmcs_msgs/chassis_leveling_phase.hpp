#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class ChassisLevelingPhase : uint8_t {
    WAIT = 0,
    ROLL = 1,
    PITCH = 2,
    MANUAL = 3,
};

} // namespace rmcs_msgs
