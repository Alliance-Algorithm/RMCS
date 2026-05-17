#pragma once
#include <cstdint>

namespace rmcs_msgs {

enum class ChassisMode : std::uint8_t {
    NONE,
    AUTO,
    SPIN_FAST,
    SPIN_SLOW,
    STEP_DOWN,
    LAUNCH_RAMP,
    ALIGNMENT,
};
constexpr auto need_power(ChassisMode mode) noexcept {
    return mode == ChassisMode::ALIGNMENT || mode == ChassisMode::LAUNCH_RAMP;
}

} // namespace rmcs_msgs
