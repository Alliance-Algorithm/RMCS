#pragma once

#include "joint_reference_recovery_controller.hpp"

#include <array>
#include <filesystem>
#include <vector>

namespace rmcs::rl {

struct ClosedChainLegGeometry {
    std::vector<float> delta_rad;
    std::vector<Eigen::Vector3f> wheel_center_b_m;
    Eigen::Vector3f hip_origin_b_m;
    Eigen::Vector3f hip_axis_b;
    Eigen::Vector3f wheel_axis_b;
    float hip_reference_rad = 0.0f;
    float wheel_radius_m = 0.06f;
};

// Immutable geometry and references bound to the 2026-10-03 V6 candidate.
// Loading these simulation candidates never grants hardware readiness.
struct RecoveryProfile {
    JointReferenceRecoveryConfig controller;
    std::array<ClosedChainLegGeometry, 2> geometry;
    static RecoveryProfile
        load(const std::filesystem::path& profile, const std::filesystem::path& lookup);
};

} // namespace rmcs::rl
