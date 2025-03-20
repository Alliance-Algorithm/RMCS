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

    UNKNOWN = UINT8_MAX
};
std::string get_damaged_reason(DamageReason reason) {
    switch (reason) {
    case DamageReason::SUFFERED_BULLET_ATTACKED: return "SUFFERED_BULLET_ATTACKED";
    case DamageReason::REFEREE_MODULE_OFFLINE: return "REFEREE_MODULE_OFFLINE";
    case DamageReason::BULLET_SPEED_EXCEEDED: return "BULLET_SPEED_EXCEEDED";
    case DamageReason::BULLET_HEAT_EXCEEDED: return "BULLET_HEAT_EXCEEDED";
    case DamageReason::CHASSIS_POWER_EXCEEDED: return "CHASSIS_POWER_EXCEEDED";
    case DamageReason::SUFFERED_IMPACTED: return "SUFFERED_IMPACTED";
    case DamageReason::UNKNOWN:
    default: return "UNKNOWN";
    }
}
} // namespace rmcs_msgs