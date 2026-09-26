#pragma once

#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::chassis {

enum class WheelLegControlState : int {
    kInit = 0,
    kDisabled = 1,
    kHold = 2,
    kRl = 3,
    kUrdfZero = 4,
    kCalibratedZero = 5,
};

constexpr WheelLegControlState wheel_leg_control_state(
    rmcs_msgs::Switch left, rmcs_msgs::Switch right) {
    using rmcs_msgs::Switch;
    if (left == Switch::UNKNOWN || right == Switch::UNKNOWN
        || (left == Switch::DOWN && right == Switch::DOWN))
        return WheelLegControlState::kDisabled;
    // 左下右中：闭环到 URDF 零点（URDF 系下即为 0）。
    if (left == Switch::DOWN && right == Switch::MIDDLE)
        return WheelLegControlState::kUrdfZero;
    // 左中右上：闭环到电机内部零点（其 URDF 角 = default_joint_pos，见 YAML）。
    if (left == Switch::MIDDLE && right == Switch::UP)
        return WheelLegControlState::kCalibratedZero;
    // 双中：进入 RL。
    if (left == Switch::MIDDLE && right == Switch::MIDDLE)
        return WheelLegControlState::kRl;
    return WheelLegControlState::kHold;
}

} // namespace rmcs_core::controller::chassis
