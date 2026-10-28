#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <eigen3/Eigen/Dense>

namespace rmcs_core::deformable_rl {

// CAD/URDF zero; the encoder's physical-angle calibration remains independent.
inline constexpr double kPhysicalZero = 1.3613532921012992;
inline constexpr double kLowerQ = 0.05235635310555198;
inline constexpr double kUpperQ = 1.0821006117822065;
inline constexpr double kBaselineQ = 1.0646473192622632;

inline double suspension_target(double action, double baseline, double lower, double upper) {
    if (!std::isfinite(action) || !std::isfinite(baseline) || !std::isfinite(lower)
        || !std::isfinite(upper) || !(lower <= baseline && baseline <= upper))
        return std::numeric_limits<double>::quiet_NaN();
    action = std::clamp(action, -1.0, 1.0);
    return baseline + action * (action < 0.0 ? baseline - lower : upper - baseline);
}

// RF, LF, LB, RB order. Encoder odometry uses the same per-leg geometry and
// regularized least-squares inverse as the training observation, including slip.
inline Eigen::Vector3d encoder_twist(const Eigen::Vector4d& q, const Eigen::Vector4d& speed) {
    if (!q.array().isFinite().all() || !speed.array().isFinite().all())
        return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    constexpr double kInvSqrt2 = 0.7071067811865475;
    constexpr double kSignX[4] = {1, 1, -1, -1};
    constexpr double kSignY[4] = {-1, 1, 1, -1};
    Eigen::Matrix<double, 4, 3> matrix;
    for (int i = 0; i < 4; ++i) {
        const double radial =
            0.12921 / kInvSqrt2 + 0.029108 * std::cos(q[i]) + 0.13694 * std::sin(q[i]) + 0.08542;
        matrix.row(i) << kSignY[i] * kInvSqrt2, -kSignX[i] * kInvSqrt2, -radial;
    }
    matrix /= 0.0769;
    const Eigen::Matrix3d normal = matrix.transpose() * matrix + 1e-5 * Eigen::Matrix3d::Identity();
    return normal.ldlt().solve(matrix.transpose() * speed);
}

} // namespace rmcs_core::deformable_rl
