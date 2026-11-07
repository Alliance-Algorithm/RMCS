#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

#include "model/calibration.hpp"

namespace rmcs_core::controller::chassis {
using JointVector = std::array<double, 6>;
using LegVector = std::array<double, 4>;
inline constexpr std::array<const char*, 6> kMotorNames{"left_hip_joint",  "left_knee_joint",
                                                        "right_hip_joint", "right_knee_joint",
                                                        "left_wheel",      "right_wheel"};
inline constexpr JointVector kNominalPosition{
    -.42, .13722330866417465, .42, -.13721605817245947, 0., 0.};
inline constexpr LegVector kMotorOffsets{1.6, 3.0282761689183353, -1.6, -3.0283102010168};
inline constexpr std::array<std::array<double, 2>, 2> kMechanicalBounds{
    {{-.14089326779459427, 1.4282761689183354}, {-.14084732125908661, 1.4283102010167998}}};
using LegBounds = std::array<std::array<double, 2>, 2>;

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
inline JointVector motor_effort(const JointVector& model) { return model_velocity(model); }
inline double forward_wheel_velocity(const JointVector& velocity) {
    return .5 * (velocity[4] - velocity[5]);
}

inline LegVector
    continuous_legs(const LegVector& measured, const LegVector& previous, const LegBounds& bounds) {
    constexpr double tau = 2 * std::numbers::pi;
    auto result = measured;
    for (int side = 0; side < 2; ++side) {
        const int hip = 2 * side, aux = hip + 1;
        const double sign = side == 0 ? 1. : -1.;
        const double midpoint = (bounds[side][0] + bounds[side][1]) / 2;
        const double delta = sign * (measured[aux] - measured[hip]);
        const double relative = delta + tau * std::nearbyint((midpoint - delta) / tau);
        result[hip] += tau * std::nearbyint((previous[hip] - measured[hip]) / tau);
        result[aux] = result[hip] + sign * relative;
    }
    return result;
}
inline LegVector
    paired_targets(const LegVector& desired, const LegVector& current, const LegBounds& bounds) {
    constexpr double tau = 2 * std::numbers::pi;
    LegVector targets;
    for (int side = 0; side < 2; ++side) {
        const int hip = 2 * side, aux = hip + 1;
        const double sign = side == 0 ? 1. : -1.;
        const double mean = (desired[hip] + desired[aux]) / 2;
        const double delta =
            std::clamp(sign * (desired[aux] - desired[hip]), bounds[side][0], bounds[side][1]);
        const double turns =
            std::floor(((current[hip] * .5 + current[aux] * .5 - mean) / tau) + .5);
        targets[hip] = mean - sign * delta / 2 + turns * tau;
        targets[aux] = mean + sign * delta / 2 + turns * tau;
        const double actual = sign * (targets[aux] - targets[hip]);
        if (actual < bounds[side][0] || actual > bounds[side][1])
            targets[aux] = std::nextafter(targets[hip] + sign * delta, targets[hip]);
    }
    return targets;
}
} // namespace rmcs_core::controller::chassis
