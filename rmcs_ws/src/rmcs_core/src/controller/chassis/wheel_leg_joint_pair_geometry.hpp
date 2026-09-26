#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>

namespace rmcs_core::controller::chassis {

struct WheelLegJointPair {
    double hip = 0.0;
    double knee = 0.0;
    bool difference_clamped = false;
};

class WheelLegJointPairGeometry {
public:
    enum class Side { kLeft, kRight };

    // V5 source URDF closure, on the physical assembly branch. The user-provided
    // 30-110 degree inner-knee range maps to these active-motor differences.
    // Left: hip - knee. Right: knee - hip. The source CAD asymmetry is <2e-5 rad.
    static constexpr double kDefaultMinDifference = -0.47375;
    static constexpr double kDefaultMaxDifference = 1.42848;
    static constexpr double kPeriod = 2.0 * std::numbers::pi;

    static double difference(Side side, double hip, double knee) {
        return sign_(side) * (hip - knee);
    }

    static std::optional<double> feedback_knee_offset(
        Side side, double hip, double knee, double min_difference, double max_difference,
        double tolerance) {
        if (!std::isfinite(hip) || !std::isfinite(knee))
            return std::nullopt;
        const double center = (min_difference + max_difference) / 2.0;
        const double raw_difference = difference(side, hip, knee);
        const double turns = std::round(sign_(side) * (raw_difference - center) / kPeriod);
        const double offset = kPeriod * turns;
        const double aligned_difference = difference(side, hip, knee + offset);
        if (aligned_difference < min_difference - tolerance
            || aligned_difference > max_difference + tolerance)
            return std::nullopt;
        return offset;
    }

    static std::optional<WheelLegJointPair> nearest_feasible_target(
        Side side, double current_hip, double current_knee, double desired_hip, double desired_knee,
        double min_difference, double max_difference, double margin) {
        if (!std::isfinite(current_hip) || !std::isfinite(current_knee)
            || !std::isfinite(desired_hip) || !std::isfinite(desired_knee))
            return std::nullopt;

        const double center = (min_difference + max_difference) / 2.0;
        const double desired_difference = difference(side, desired_hip, desired_knee);
        const double knee_turns = std::round(sign_(side) * (desired_difference - center) / kPeriod);
        const double knee_branch = desired_knee + kPeriod * knee_turns;
        const double branch_difference = difference(side, desired_hip, knee_branch);
        const double safe_difference =
            std::clamp(branch_difference, min_difference + margin, max_difference - margin);
        const double mean = (desired_hip + knee_branch) / 2.0;
        const double hip = mean + sign_(side) * safe_difference / 2.0;
        const double knee = mean - sign_(side) * safe_difference / 2.0;

        // A common full turn preserves the leg shape. Choose the common turn
        // that minimizes the larger of the two motor travels, as a pair.
        const double current_mean = (current_hip + current_knee) / 2.0;
        const double nearest_turns = std::round((current_mean - mean) / kPeriod);
        WheelLegJointPair best;
        double best_max_travel = std::numeric_limits<double>::infinity();
        double best_total_travel = std::numeric_limits<double>::infinity();
        for (int neighbor = -1; neighbor <= 1; ++neighbor) {
            const double common_turn = kPeriod * (nearest_turns + neighbor);
            const double candidate_hip = hip + common_turn;
            const double candidate_knee = knee + common_turn;
            const double hip_travel = std::abs(candidate_hip - current_hip);
            const double knee_travel = std::abs(candidate_knee - current_knee);
            const double max_travel = std::max(hip_travel, knee_travel);
            const double total_travel = hip_travel + knee_travel;
            if (max_travel < best_max_travel
                || (max_travel == best_max_travel && total_travel < best_total_travel)) {
                best = {
                    candidate_hip, candidate_knee,
                    std::abs(branch_difference - safe_difference) > 1e-12};
                best_max_travel = max_travel;
                best_total_travel = total_travel;
            }
        }
        return best;
    }

private:
    static double sign_(Side side) { return side == Side::kLeft ? 1.0 : -1.0; }
};

} // namespace rmcs_core::controller::chassis
