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
    // 左下右中：闭环到电机内部标定姿态（其 URDF 角 q_cal，见 YAML）。
    if (left == Switch::DOWN && right == Switch::MIDDLE)
        return WheelLegControlState::kCalibratedZero;
    // 左中右下：闭环到 URDF 零点（URDF 系下为 0）。
    if (left == Switch::MIDDLE && right == Switch::DOWN)
        return WheelLegControlState::kUrdfZero;
    // 双中：进入 RL。
    if (left == Switch::MIDDLE && right == Switch::MIDDLE)
        return WheelLegControlState::kRl;
    return WheelLegControlState::kHold;
}

} // namespace rmcs_core::controller::chassis
