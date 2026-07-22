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
};

constexpr auto is_powered(ChassisMode mode) noexcept {
    return mode == ChassisMode::ALIGNMENT_POWERED || mode == ChassisMode::LAUNCH_RAMP;
}
constexpr auto is_spining(ChassisMode mode) noexcept {
    return mode == ChassisMode::SPIN_SLOW || mode == ChassisMode::SPIN_FAST;
}

} // namespace rmcs_msgs
