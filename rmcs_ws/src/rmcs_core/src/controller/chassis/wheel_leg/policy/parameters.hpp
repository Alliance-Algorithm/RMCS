#pragma once
#include "../joint_coordinates.hpp"
#include <string_view>
namespace rmcs_core::controller::chassis {
struct PolicyParameters {
    std::string_view name, observation_version, observation_sha;
    double policy_hz, leg_scale, leg_bound, wheel_bound, leg_kp, leg_kd, gas_gain;
    double forward_limit, yaw_limit;
    LegBounds winding_bounds;
    JointVector nominal = kNominalPosition;
    bool project_pair = true;
};
inline constexpr std::string_view kModelSpecSha256 =
    "8a9d759525c48955e8965b2e14b9e158c1d8fc9f3d68f33e4024a3c94bf8f812";
inline JointVector
    action_target(const PolicyParameters& p, const JointVector& raw, const JointVector& q) {
    JointVector target;
    LegVector desired{}, current{};
    for (int i = 0; i < 4; ++i) {
        desired[i] = p.nominal[i] + std::clamp(raw[i] * p.leg_scale, -p.leg_bound, p.leg_bound);
        current[i] = q[i];
    }
    auto legs = desired;
    if (p.project_pair) {
        legs = paired_targets(desired, current, p.winding_bounds);
    } else {
        constexpr double tau = 2. * std::numbers::pi;
        for (int side = 0; side < 2; ++side) {
            const int hip = 2 * side;
            const double turns = std::floor(
                ((current[hip] + current[hip + 1] - desired[hip] - desired[hip + 1]) * .5) / tau
                + .5);
            legs[hip] += turns * tau;
            legs[hip + 1] += turns * tau;
        }
    }
    std::copy(legs.begin(), legs.end(), target.begin());
    for (int i = 4; i < 6; ++i)
        target[i] = std::clamp(raw[i] * 10., -90., 90.);
    return target;
}
inline JointVector executed_history(const PolicyParameters& p, const JointVector& raw) {
    if (p.name == "wheelbipe")
        return raw;
    auto result = raw;
    for (int i = 0; i < 6; ++i) {
        const double scale = i < 4 ? p.leg_scale : 10.;
        const double bound = i < 4 ? p.leg_bound : p.wheel_bound;
        result[i] = std::clamp(raw[i] * scale, -bound, bound) / scale;
    }
    return result;
}
} // namespace rmcs_core::controller::chassis
