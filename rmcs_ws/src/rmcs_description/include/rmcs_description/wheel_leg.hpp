#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace rmcs_description::wheel_leg {

using JointVector = std::array<double, 6>;
using LegVector = std::array<double, 4>;
using LegBounds = std::array<std::array<double, 2>, 2>;
inline constexpr std::array<const char*, 6> kJointNames{"left_hip_joint",  "left_knee_joint",
                                                        "right_hip_joint", "right_knee_joint",
                                                        "left_wheel",      "right_wheel"};
// Nominal per-side bounds for choosing a continuous closed-chain angle branch.
inline constexpr LegBounds kWindingBounds{
    {{-0.14089326779459421, 1.4282761689185115}, {-0.14084732125908495, 1.4283102010168576}}};

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

} // namespace rmcs_description::wheel_leg
