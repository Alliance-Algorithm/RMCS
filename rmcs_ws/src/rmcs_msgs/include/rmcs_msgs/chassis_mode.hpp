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
};

constexpr auto need_power(ChassisMode mode) noexcept {
    return mode == ChassisMode::ALIGNMENT_POWERED || mode == ChassisMode::LAUNCH_RAMP;
}

} // namespace rmcs_msgs
