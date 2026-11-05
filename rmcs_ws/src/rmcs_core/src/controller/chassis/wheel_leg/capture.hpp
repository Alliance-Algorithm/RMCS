#pragma once

#include "joint_coordinates.hpp"
#include "model/body_geometry.hpp"
#include "model/recovery_profile.hpp"

#include <cmath>
#include <numbers>

namespace rmcs_core::controller::chassis::wheel_leg {

// dev/wheel_leg_rl @ 0fcc9b45, controller_recovery.cpp and recovery YAML.
// Moving capture: neither script completion nor stationary balance is required.
struct RecoveryCaptureLimits {
    double tilt = std::numbers::pi / 6.;
    double leg_error = .45;
    double leg_velocity = 4.;
    double wheel_velocity = 20.;
    double gyro = 1.;
    double upright_gyro = 2.;
    double upright_tilt = .2;
};

enum RecoveryCaptureBlocker {
    kCaptureSensing = 1 << 0,
    kCaptureTilt = 1 << 1,
    kCaptureGyro = 1 << 2,
    kCaptureWheelVelocity = 1 << 3,
    kCaptureLegVelocity = 1 << 4,
    kCaptureReference = 1 << 5,
    kCaptureHinge = 1 << 6,
    kCaptureSupportingAcceleration = 1 << 7,
    kCaptureShell = 1 << 8,
    kCaptureOutwardTilt = 1 << 9,
};

inline int recovery_capture_blockers(
    const JointReferenceRecoveryFeedback& feedback, const Eigen::Vector3d& acceleration,
    const Eigen::Vector4d& reference, const RecoveryCaptureLimits& limits = {}) {
    if (!feedback.q.allFinite() || !feedback.dq.allFinite() || !feedback.gravity.allFinite()
        || !feedback.gyro.allFinite() || !acceleration.allFinite()
        || !std::isfinite(feedback.estimated_height)
        || std::abs(feedback.gravity.squaredNorm() - 1.) > 1e-3)
        return kCaptureSensing;
    int blockers = 0;
    const auto& g = feedback.gravity;
    if (-g.z() < std::cos(limits.tilt))
        blockers |= kCaptureTilt;
    const bool upright = -g.z() >= std::cos(limits.upright_tilt);
    if (feedback.gyro.norm() > (upright ? limits.upright_gyro : limits.gyro))
        blockers |= kCaptureGyro;
    if (feedback.dq.head<4>().cwiseAbs().maxCoeff() > limits.leg_velocity)
        blockers |= kCaptureLegVelocity;
    if (feedback.dq.tail<2>().cwiseAbs().maxCoeff() > limits.wheel_velocity)
        blockers |= kCaptureWheelVelocity;
    for (int i = 0; i < 4; ++i)
        if (std::abs(std::remainder(reference[i] - feedback.q[i], 2. * std::numbers::pi))
            > limits.leg_error)
            blockers |= kCaptureReference;
    for (int side = 0; side < 2; ++side) {
        const int hip = 2 * side;
        const double relative = (side == 0 ? 1. : -1.) * (feedback.q[hip + 1] - feedback.q[hip]);
        if (relative < kMechanicalBounds[side][0] || relative > kMechanicalBounds[side][1]
            || relative > kRecoveryCaptureHingeMax[side])
            blockers |= kCaptureHinge;
    }
    if (acceleration.dot(-g) < 6.)
        blockers |= kCaptureSupportingAcceleration;
    if (feedback.estimated_height + .003 < body_geometry::support_depth(g.cast<float>()))
        blockers |= kCaptureShell;
    if (-g.z() < std::cos(std::numbers::pi / 6.)
        && g.x() * feedback.gyro.y() - g.y() * feedback.gyro.x() > .05 * std::hypot(g.x(), g.y()))
        blockers |= kCaptureOutwardTilt;
    return blockers;
}

} // namespace rmcs_core::controller::chassis::wheel_leg
