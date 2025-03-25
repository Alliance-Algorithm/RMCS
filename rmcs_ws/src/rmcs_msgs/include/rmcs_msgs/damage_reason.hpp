#pragma once

#include <cstdint>
#include <string>

namespace rmcs_msgs {

enum class DamageReason : uint8_t {

    SUFFERED_BULLET_ATTACKED = 0,
    REFEREE_MODULE_OFFLINE   = 1,
    BULLET_SPEED_EXCEEDED    = 2,
    BULLET_HEAT_EXCEEDED     = 3,
    CHASSIS_POWER_EXCEEDED   = 4,
    SUFFERED_IMPACTED        = 5,

    NOT_DAMAGED = UINT8_MAX
};

} // namespace rmcs_msgs