#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class ChassisLevelingPhase : uint8_t {
    IDLE         = 0,
    LEGACY_ROLL  = 1,
    LEGACY_PITCH = 2,
    STAGE_CONTACT = 3,
    STAGE_ROLL    = 4,
    STAGE_PITCH   = 5,
};

} // namespace rmcs_msgs
