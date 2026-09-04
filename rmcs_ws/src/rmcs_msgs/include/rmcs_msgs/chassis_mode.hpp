#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class ChassisMode : uint8_t {
    SPIN = 1,
    Flow,
    Yaw_Free,
    None, // 为了编译视觉临时添加
    AUTO,
    SPIN_SLOW,
    SPIN_FAST,
    STEP_DOWN,
    LAUNCH_RAMP,
    ALIGNMENT,
    ALIGNMENT_POWERED,
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
