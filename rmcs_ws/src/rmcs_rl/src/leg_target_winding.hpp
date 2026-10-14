#pragma once

#include "policy.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>

namespace rmcs::rl {

enum class LegTargetWinding { kIndependentNearest, kPairedCommon, kPairedBranch107 };

inline constexpr std::string_view kBranch107ModelSpecSha256 =
    "0ac82045ab584f20091538aa782d93ea545382dc701faca1f8ae92aeb7cd1997";
// Canonical relative root-drive angle: left aux - hip, right hip - aux.
// These bounds belong to the exact model above, rather than encoder zeroes.
inline constexpr std::array<std::array<double, 2>, 2> kBranch107SignedDeltaBoundsRad{{
    {-.14062820701680934, 1.3692123767724917},
    {-.1406283965709381, 1.3692125038958158},
}};

inline constexpr std::string_view leg_target_winding_name(LegTargetWinding version) {
    if (version == LegTargetWinding::kPairedBranch107)
        return "paired_branch107_v1";
    return version == LegTargetWinding::kPairedCommon ? "paired_common_v1"
                                                   : "independent_nearest_v1";
}

inline LegTargetWinding
    parse_leg_target_winding(std::string_view version, std::string_view control_profile) {
    if (version == "independent_nearest_v1")
        return LegTargetWinding::kIndependentNearest;
    if (version != "paired_common_v1" && version != "paired_branch107_v1")
        throw std::runtime_error("Unknown leg_winding_version: " + std::string{version});
    if (control_profile != kV6PolicyProfile.name)
        throw std::runtime_error(
            std::string{version} + " requires the continuous root-drive v6 profile");
    return version == "paired_branch107_v1" ? LegTargetWinding::kPairedBranch107
                                           : LegTargetWinding::kPairedCommon;
}

inline void validate_leg_target_model_spec(
    LegTargetWinding version, std::string_view model_spec_sha256, std::string_view origin) {
    if (version == LegTargetWinding::kPairedBranch107
        && model_spec_sha256 != kBranch107ModelSpecSha256)
        throw std::runtime_error(
            std::string{origin} + " model_spec_sha256 must match paired_branch107_v1");
}

inline void validate_leg_target_recovery_profile(
    LegTargetWinding version, bool script_recovery_enabled, std::string_view model_spec_sha256) {
    if (script_recovery_enabled)
        validate_leg_target_model_spec(version, model_spec_sha256, "Script recovery profile");
}

inline std::array<double, 4> wind_leg_targets(
    const std::array<double, 4>& desired, const std::array<double, 4>& current,
    LegTargetWinding version) {
    constexpr double tau = 2 * std::numbers::pi;
    std::array<double, 4> targets;
    if (version == LegTargetWinding::kPairedBranch107) {
        for (int side = 0; side < 2; ++side) {
            const int hip = 2 * side, aux = hip + 1;
            const double sign = side == 0 ? 1.0 : -1.0;
            const auto bounds = kBranch107SignedDeltaBoundsRad[side];
            const double mean = (desired[hip] + desired[aux]) / 2;
            const double delta = std::clamp(
                sign * (desired[aux] - desired[hip]), bounds[0], bounds[1]);
            const double turns = std::floor(
                ((current[hip] * .5 + current[aux] * .5 - mean) / tau) + .5);
            const double common = turns * tau;
            targets[hip] = mean - sign * delta / 2 + common;
            targets[aux] = mean + sign * delta / 2 + common;
            // The final double subtraction can round just outside the fixed
            // domain after a large common turn. Reconstruct only the
            // auxiliary target, then move one ULP toward the unchanged hip.
            const auto canonical_delta = [&] { return sign * (targets[aux] - targets[hip]); };
            if (canonical_delta() < bounds[0] || canonical_delta() > bounds[1]) {
                targets[aux] = std::nextafter(targets[hip] + sign * delta, targets[hip]);
            }
        }
    } else if (version == LegTargetWinding::kPairedCommon) {
        for (int side = 0; side < 2; ++side) {
            const int first = 2 * side;
            // Keep double arithmetic in the deployed controller. Both root
            // drives receive one common turn; no relative-angle projection.
            const double offset =
                ((current[first] - desired[first]) + (current[first + 1] - desired[first + 1]))
                / 2;
            const double turns = std::floor(offset / tau + .5);
            targets[first] = desired[first] + turns * tau;
            targets[first + 1] = desired[first + 1] + turns * tau;
        }
    } else {
        for (int i = 0; i < 4; ++i)
            targets[i] = current[i] + std::remainder(desired[i] - current[i], tau);
    }
    return targets;
}

} // namespace rmcs::rl
