#pragma once

#include <algorithm>
#include <cmath>

#include <eigen3/Eigen/Dense>

namespace rmcs_core::deformable_rl {

struct FlatReturnTarget {
    Eigen::Vector4d position;
    Eigen::Vector4d velocity;
};

// Match coordinated_flat_return in the training runtime_guard.py: one quintic
// phase brings all four URDF joints to the baseline together.
inline FlatReturnTarget coordinated_flat_return(const Eigen::Vector4d& start, double baseline,
    double elapsed, double minimum_duration, double rate_limit, double acceleration_limit) {
    const Eigen::Vector4d delta = Eigen::Vector4d::Constant(baseline) - start;
    const double distance = delta.cwiseAbs().maxCoeff();
    const double duration = std::max({minimum_duration, 1.875 * distance / rate_limit,
        std::sqrt(5.774 * distance / acceleration_limit)});
    const double phase = std::clamp(elapsed / duration, 0.0, 1.0);
    const double progress = phase * phase * phase * (10.0 - 15.0 * phase + 6.0 * phase * phase);
    const double speed = 30.0 * phase * phase * (1.0 - phase) * (1.0 - phase) / duration;
    return {start + delta * progress, delta * speed};
}

inline double flat_terminal_velocity(double measured, double baseline,
    double trajectory_velocity, double acceleration_limit) {
    const double error = baseline - measured;
    if (trajectory_velocity * error <= 0.0)
        return 0.0;
    return std::copysign(std::min(std::abs(trajectory_velocity),
        std::sqrt(2.0 * acceleration_limit * std::abs(error))), error);
}

} // namespace rmcs_core::deformable_rl
