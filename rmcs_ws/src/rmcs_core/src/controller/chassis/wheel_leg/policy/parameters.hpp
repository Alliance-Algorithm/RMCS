#pragma once
#include "../joint_coordinates.hpp"
#include <string_view>
namespace rmcs_core::controller::chassis {
inline constexpr std::string_view kModelSpecSha256 =
    "8a9d759525c48955e8965b2e14b9e158c1d8fc9f3d68f33e4024a3c94bf8f812";
struct PolicyParameters {
    std::string_view name, observation_version, observation_sha;
    double policy_hz, leg_scale, leg_bound, wheel_bound, leg_kp, leg_kd, gas_gain;
    double forward_limit, yaw_limit;
    LegBounds winding_bounds;
    std::string_view model_spec_sha256 = kModelSpecSha256;
    bool jump_control = false;
};
inline bool wheelbipe_policy(const PolicyParameters& p) {
    return p.name == "wheelbipe" || p.jump_control;
}
inline JointVector decoded_action_target(
    const JointVector& decoded, const JointVector& q, const LegBounds& bounds) {
    LegVector desired{}, current{};
    for (int i = 0; i < 4; ++i) {
        desired[i] = kNominalPosition[i] + decoded[i];
        current[i] = q[i];
    }
    const auto legs = paired_targets(desired, current, bounds);
    auto target = decoded;
    std::copy(legs.begin(), legs.end(), target.begin());
    return target;
}
inline JointVector
    action_target(const PolicyParameters& p, const JointVector& raw, const JointVector& q) {
    JointVector target;
    LegVector desired{}, current{};
    for (int i = 0; i < 4; ++i) {
        desired[i] =
            kNominalPosition[i] + std::clamp(raw[i] * p.leg_scale, -p.leg_bound, p.leg_bound);
        current[i] = q[i];
    }
    const auto legs = paired_targets(desired, current, p.winding_bounds);
    std::copy(legs.begin(), legs.end(), target.begin());
    for (int i = 4; i < 6; ++i)
        target[i] = std::clamp(raw[i] * 10., -90., 90.);
    return target;
}
inline JointVector executed_history(const PolicyParameters& p, const JointVector& raw) {
    if (wheelbipe_policy(p))
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
