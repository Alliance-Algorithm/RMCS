#pragma once

#include "policy.hpp"

#include <array>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>

namespace rmcs::rl {

enum class LegTargetWinding { kIndependentNearest, kPairedCommon };

inline constexpr std::string_view leg_target_winding_name(LegTargetWinding version) {
    return version == LegTargetWinding::kPairedCommon ? "paired_common_v1"
                                                   : "independent_nearest_v1";
}

inline LegTargetWinding
    parse_leg_target_winding(std::string_view version, std::string_view control_profile) {
    if (version == "independent_nearest_v1")
        return LegTargetWinding::kIndependentNearest;
    if (version != "paired_common_v1")
        throw std::runtime_error("Unknown leg_winding_version: " + std::string{version});
    if (control_profile != kV6PolicyProfile.name)
        throw std::runtime_error("paired_common_v1 requires the continuous root-drive v6 profile");
    return LegTargetWinding::kPairedCommon;
}

inline std::array<double, 4> wind_leg_targets(
    const std::array<double, 4>& desired, const std::array<double, 4>& current,
    LegTargetWinding version) {
    constexpr double tau = 2 * std::numbers::pi;
    std::array<double, 4> targets;
    if (version == LegTargetWinding::kPairedCommon) {
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
