#pragma once

#include <algorithm>
#include <cmath>

#include "joint_coordinates.hpp"

namespace rmcs_core::controller::chassis::wheel_leg {

inline constexpr double kLegTorqueLimit = 40.;
inline constexpr double kWheelTorqueLimit = 4.5;
inline constexpr double kHingeMargin = .03;
// Normal running uses the same exported soft hinge limits on both sides.
inline constexpr LegBounds kRunningTorqueBounds{
    {{-0.14084732125908495, 1.4282761689185115}, {-0.14084732125908495, 1.4282761689185115}}};

inline JointVector pd_torque(
    const JointVector& target, const JointVector& q, const JointVector& dq, double leg_kp,
    double leg_kd) {
    JointVector torque;
    for (int i = 0; i < 4; ++i)
        torque[i] = leg_kp * (target[i] - q[i]) - leg_kd * dq[i];
    for (int i = 4; i < 6; ++i)
        torque[i] = std::clamp(.5 * (target[i] - dq[i]), -kWheelTorqueLimit, kWheelTorqueLimit);
    return torque;
}

inline JointVector limit_torque(
    JointVector torque, const JointVector& q, const LegBounds& bounds = kRunningTorqueBounds) {
    for (int i = 0; i < 4; ++i)
        torque[i] = std::clamp(torque[i], -kLegTorqueLimit, kLegTorqueLimit);
    for (int side = 0; side < 2; ++side) {
        const int hip = 2 * side, knee = hip + 1;
        const double sign = side == 0 ? 1. : -1.;
        const double relative = sign * (q[knee] - q[hip]);
        const double outward = sign * (torque[knee] - torque[hip]);
        if ((relative >= bounds[side][1] - kHingeMargin && outward > 0.)
            || (relative <= bounds[side][0] + kHingeMargin && outward < 0.)) {
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
