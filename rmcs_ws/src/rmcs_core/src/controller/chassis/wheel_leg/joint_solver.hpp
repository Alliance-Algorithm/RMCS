#pragma once

#include <algorithm>
#include <cmath>

#include "joint_coordinates.hpp"
#include "jump_v4.hpp"

namespace rmcs_core::controller::chassis::wheel_leg {

inline constexpr double kLegTorqueLimit = 40.;
inline constexpr double kWheelTorqueLimit = 4.5;
inline constexpr double kHingeMargin = .03;

inline JointVector
    pd_torque(const JointVector& target, const JointVector& q, const JointVector& dq) {
    JointVector torque;
    for (int i = 0; i < 4; ++i)
        torque[i] = jump_v4::kLegKp * (target[i] - q[i]) - jump_v4::kLegKd * dq[i];
    for (int i = 4; i < 6; ++i)
        torque[i] = std::clamp(.5 * (target[i] - dq[i]), -kWheelTorqueLimit, kWheelTorqueLimit);
    return torque;
}

inline JointVector limit_torque(JointVector torque, const JointVector& q) {
    for (int i = 0; i < 4; ++i)
        torque[i] = std::clamp(torque[i], -kLegTorqueLimit, kLegTorqueLimit);
    for (int side = 0; side < 2; ++side) {
        const int hip = 2 * side, knee = hip + 1;
        const double sign = side == 0 ? 1. : -1.;
        const double relative = sign * (q[knee] - q[hip]);
        const double outward = sign * (torque[knee] - torque[hip]);
        if ((relative >= jump_v4::kMechanicalMax - kHingeMargin && outward > 0.)
            || (relative <= jump_v4::kMechanicalMin + kHingeMargin && outward < 0.)) {
            torque[hip] += sign * outward / 2.;
            torque[knee] -= sign * outward / 2.;
        }
    }
    double scale = 1.;
    for (int i = 0; i < 4; ++i)
        if (std::abs(torque[i]) > kLegTorqueLimit)
            scale = std::min(scale, kLegTorqueLimit / std::abs(torque[i]));
    for (int i = 0; i < 4; ++i)
        torque[i] *= scale;
    for (int i = 4; i < 6; ++i)
        torque[i] = std::clamp(torque[i], -kWheelTorqueLimit, kWheelTorqueLimit);
    return motor_torque(torque);
}

} // namespace rmcs_core::controller::chassis::wheel_leg
