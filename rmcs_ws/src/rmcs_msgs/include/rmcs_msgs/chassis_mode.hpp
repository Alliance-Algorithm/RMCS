#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class ChassisMode : uint8_t {
    AUTO,
    SPIN_SLOW,
    SPIN_FAST,
    STEP_DOWN,
    LAUNCH_RAMP,
    ALIGNMENT,
    ALIGNMENT_POWERED,
    WIRELESS_CHARGING,
    CLIMB,
};

constexpr auto is_powered(ChassisMode mode) noexcept {
    return mode == ChassisMode::ALIGNMENT_POWERED || mode == ChassisMode::LAUNCH_RAMP
        || mode == ChassisMode::CLIMB;
}
constexpr auto is_spining(ChassisMode mode) noexcept {
    return mode == ChassisMode::SPIN_SLOW || mode == ChassisMode::SPIN_FAST;
}

} // namespace rmcs_msgs
