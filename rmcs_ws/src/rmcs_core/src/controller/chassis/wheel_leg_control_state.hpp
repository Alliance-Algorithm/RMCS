#pragma once

#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::chassis {

enum class WheelLegControlState : int {
    kInit = 0,
    kDisabled = 1,
    kHold = 2,
    kRl = 3,
    kUrdfZero = 4,
    kNominal = 5,
    kCalibratedZero = 6,
};

constexpr WheelLegControlState wheel_leg_control_state(
    rmcs_msgs::Switch left, rmcs_msgs::Switch right) {
    using rmcs_msgs::Switch;
    if (left == Switch::UNKNOWN || right == Switch::UNKNOWN
        || (left == Switch::DOWN && right == Switch::DOWN))
        return WheelLegControlState::kDisabled;
    if (left == Switch::DOWN && right == Switch::MIDDLE)
        return WheelLegControlState::kUrdfZero;
    if (left == Switch::MIDDLE && right == Switch::DOWN)
        return WheelLegControlState::kNominal;
    if (left == Switch::MIDDLE && right == Switch::UP)
        return WheelLegControlState::kCalibratedZero;
    if (left == Switch::MIDDLE && right == Switch::MIDDLE)
        return WheelLegControlState::kRl;
    return WheelLegControlState::kHold;
}

} // namespace rmcs_core::controller::chassis
