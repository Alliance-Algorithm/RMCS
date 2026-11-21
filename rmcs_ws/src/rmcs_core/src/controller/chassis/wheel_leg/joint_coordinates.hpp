#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

#include <rmcs_description/wheel_leg.hpp>

namespace rmcs_core::controller::chassis::wheel_leg {
using rmcs_description::wheel_leg::JointVector;
using rmcs_description::wheel_leg::kJointNames;
using rmcs_description::wheel_leg::kWindingBounds;
using rmcs_description::wheel_leg::LegBounds;
using rmcs_description::wheel_leg::LegVector;
using rmcs_description::wheel_leg::paired_targets;
inline constexpr LegVector kMotorOffsets{1.6, 3.0282761689183353, -1.6, -3.0283102010168};
// Per-side mechanical limits used during recovery and its capture check.
inline constexpr LegBounds kMechanicalBounds{
    {{-.14089326779459427, 1.4282761689183354}, {-.14084732125908661, 1.4283102010167998}}};

inline JointVector model_position(const JointVector& api) {
    auto q = api;
    for (int i = 0; i < 4; ++i)
        q[i] = -api[i] + kMotorOffsets[i];
    return q;
}
inline JointVector model_velocity(const JointVector& api) {
    auto dq = api;
    for (int i = 0; i < 4; ++i)
        dq[i] = -api[i];
    return dq;
}
inline JointVector motor_torque(const JointVector& model) { return model_velocity(model); }

// The drive span unwrap upstream already returns a continuous hip angle; this
// only picks the relative (closed-chain) branch, so no 2 pi multiple is ever
// added to the absolute angle here.
inline LegVector continuous_legs(const LegVector& measured, const LegBounds& bounds) {
    constexpr double tau = 2 * std::numbers::pi;
    auto result = measured;
    for (int side = 0; side < 2; ++side) {
        const int hip = 2 * side, aux = hip + 1;
        const double sign = side == 0 ? 1. : -1.;
        const double midpoint = (bounds[side][0] + bounds[side][1]) / 2;
        const double delta = sign * (measured[aux] - measured[hip]);
        const double relative = delta + tau * std::nearbyint((midpoint - delta) / tau);
        result[aux] = result[hip] + sign * relative;
    }
    return result;
}
} // namespace rmcs_core::controller::chassis::wheel_leg
