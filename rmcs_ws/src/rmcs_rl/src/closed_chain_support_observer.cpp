#include "closed_chain_support_observer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

#include <eigen3/Eigen/Geometry>

namespace rmcs::rl {
ClosedChainSupportObserver::ClosedChainSupportObserver(
    std::array<ClosedChainLegGeometry, 2> geometry, double period_seconds)
    : geometry_{std::move(geometry)} {
    if (!std::isfinite(period_seconds)
        || (std::abs(period_seconds - 0.001) > 1e-12 && std::abs(period_seconds - 0.005) > 1e-12))
        throw std::invalid_argument("Closed-chain observation period must be 1 ms or 5 ms");
    period_seconds_ = std::abs(period_seconds - 0.001) <= 1e-12 ? 0.001 : 0.005;
    reset();
}

void ClosedChainSupportObserver::reset() {
    initialized_ = geometry_valid_ = plausible_ = confirmed_ = false;
    winding_.setZero();
    previous_velocity_.setZero();
    previous_gyro_.setZero();
    previous_pulse_.setZero();
    evidence_ = {};
    bad_samples_ = {};
    bad_elapsed_seconds_ = {};
    for (auto& side : evidence_age_)
        side.fill(std::numeric_limits<float>::infinity());
    for (auto& side : evidence_elapsed_seconds_)
        side.fill(std::numeric_limits<double>::infinity());
    lost_seconds_ = 0.0f;
    lost_elapsed_seconds_ = 0.0;
    probe_elapsed_seconds_ = 0.0;
    reference_tick_ = 0;
}

JointReferenceRecoveryFeedback ClosedChainSupportObserver::observe(
    const RecoverySensorData& sensors, const JointReferenceRecoveryVector6& raw_q,
    const JointReferenceRecoveryVector6& dq, const Eigen::Vector3d& gravity,
    const Eigen::Vector3d& gyro, const Eigen::Vector3d& acceleration,
    JointReferenceRecoveryPhase phase) {
    return observe(sensors, raw_q, dq, gravity, gyro, acceleration, phase, period_seconds_);
}

JointReferenceRecoveryFeedback ClosedChainSupportObserver::observe(
    const RecoverySensorData&, const JointReferenceRecoveryVector6& raw_q,
    const JointReferenceRecoveryVector6& dq, const Eigen::Vector3d& gravity,
    const Eigen::Vector3d& gyro, const Eigen::Vector3d& acceleration,
    JointReferenceRecoveryPhase phase, double elapsed_seconds) {
    if (!std::isfinite(elapsed_seconds) || elapsed_seconds <= 0.0 || elapsed_seconds > 0.020) {
        reset();
        JointReferenceRecoveryFeedback feedback;
        feedback.q.setConstant(std::numeric_limits<double>::quiet_NaN());
        return feedback;
    }
    const bool fixed_period = period_seconds_ == 0.005;
    const float dt = fixed_period ? 0.005f : static_cast<float>(elapsed_seconds);
    constexpr float tau = 2.0f * std::numbers::pi_v<float>;
    const Eigen::Matrix<float, 6, 1> raw = raw_q.cast<float>();
    if (!initialized_) {
        q_ = raw;
        for (int side = 0; side < 2; ++side) {
            const auto& table = geometry_[side];
            const int hip = 2 * side, auxiliary = hip + 1;
            const float midpoint = (table.delta_rad.front() + table.delta_rad.back()) / 2.0f;
            const float difference = raw[auxiliary] - raw[hip];
            if (std::isfinite(difference))
                winding_[auxiliary] =
                    static_cast<int>(std::nearbyint((midpoint - difference) / tau));
        }
        initialized_ = true;
    } else {
        for (int axis = 0; axis < 6; ++axis)
            if (std::isfinite(raw[axis]) && std::isfinite(last_raw_[axis]))
                winding_[axis] -=
                    static_cast<int>(std::nearbyint((raw[axis] - last_raw_[axis]) / tau));
    }
    for (int axis = 0; axis < 6; ++axis)
        if (std::isfinite(q_[axis]))
            q_[axis] = winding_[axis] == 0 ? raw[axis] : raw[axis] + winding_[axis] * tau;
    last_raw_ = raw;
    const Eigen::Vector3f g = gravity.cast<float>(), omega = gyro.cast<float>();
    geometry_valid_ = q_.allFinite() && g.allFinite() && std::abs(g.squaredNorm() - 1.0f) <= 1e-3f;
    const bool gravity_valid = geometry_valid_;
    bool height_valid = geometry_valid_;
    for (int side = 0; side < 2; ++side) {
        const auto& table = geometry_[side];
        const int hip = 2 * side;
        const float delta = q_[hip + 1] - q_[hip];
        const bool in_domain = std::isfinite(delta) && delta >= table.delta_rad.front() - 1e-6f
                            && delta <= table.delta_rad.back() + 1e-6f;
        geometry_valid_ &= in_domain;
        const auto found = std::lower_bound(table.delta_rad.begin(), table.delta_rad.end(), delta);
        const auto upper =
            std::clamp<std::size_t>(found - table.delta_rad.begin(), 1, table.delta_rad.size() - 1);
        const auto lower = upper - 1;
        const float blend = std::clamp(
            (delta - table.delta_rad[lower]) / (table.delta_rad[upper] - table.delta_rad[lower]),
            0.0f, 1.0f);
        const Eigen::Vector3f point =
            table.wheel_center_b_m[lower]
            + blend * (table.wheel_center_b_m[upper] - table.wheel_center_b_m[lower]);
        const Eigen::Vector3f vector = point - table.hip_origin_b_m;
        const float angle = q_[hip] - table.hip_reference_rad;
        const Eigen::Vector3f rotated =
            std::cos(angle) * vector + std::sin(angle) * table.hip_axis_b.cross(vector)
            + (1.0f - std::cos(angle)) * table.hip_axis_b * table.hip_axis_b.dot(vector);
        const Eigen::Vector3f center =
            in_domain ? Eigen::Vector3f{table.hip_origin_b_m + rotated} : Eigen::Vector3f::Zero();
        const float axle_g = g.dot(table.wheel_axis_b);
        heights_[side] = center.dot(g)
                       + table.wheel_radius_m * std::sqrt(std::max(0.0f, 1.0f - axle_g * axle_g));
        if (!gravity_valid)
            heights_[side] = 0.0f;
        height_valid &= in_domain && std::isfinite(heights_[side]) && heights_[side] > 0.0f;
    }
    const float acceleration_norm = acceleration.cast<float>().norm();
    // Allow a small roll/asymmetric-leg capture window. These are conditional
    // wheel-plane heights, not proof that both wheels touch the ground.
    plausible_ = height_valid && heights_.minCoeff() > 0.12f
              && std::abs(heights_[0] - heights_[1]) < 0.045f && acceleration_norm > 6.0f
              && acceleration_norm < 16.0f && omega.norm() < 8.0f;
    const Eigen::Vector2f velocity = dq.tail<2>().cast<float>();
    const Eigen::Vector2f wheel_acceleration = (velocity - previous_velocity_) / dt;
    const float gyro_acceleration = (omega - previous_gyro_).norm() / dt;
    if (fixed_period)
        lost_seconds_ = plausible_ ? 0.0f : lost_seconds_ + dt;
    else
        lost_elapsed_seconds_ = plausible_ ? 0.0 : lost_elapsed_seconds_ + elapsed_seconds;
    confirmed_ = plausible_;
    for (int side = 0; side < 2; ++side) {
        const bool tested = plausible_ && std::abs(previous_pulse_[side]) >= 0.14f;
        const bool loaded =
            tested && std::abs(wheel_acceleration[side]) < 100.0f && gyro_acceleration < 90.0f;
        for (int direction = 0; direction < 2; ++direction) {
            const bool test_direction =
                tested && (previous_pulse_[side] > 0.0f) == (direction != 0);
            if (fixed_period) {
                // Retain the frozen 200 Hz float and sample-count arithmetic.
                evidence_age_[side][direction] += dt;
                if (test_direction) {
                    bad_samples_[side][direction] = loaded ? 0 : bad_samples_[side][direction] + 1;
                    if (loaded) {
                        evidence_[side][direction] = true;
                        evidence_age_[side][direction] = 0.0f;
                    }
                }
                evidence_[side][direction] &=
                    bad_samples_[side][direction] < 2 && lost_seconds_ < 0.05f;
                confirmed_ &= evidence_[side][direction] && evidence_age_[side][direction] < 0.6f;
            } else {
                // The 1 kHz path measures bad response, loss and expiry in time.
                constexpr double epsilon = 1e-12;
                evidence_elapsed_seconds_[side][direction] += elapsed_seconds;
                if (test_direction) {
                    bad_elapsed_seconds_[side][direction] =
                        loaded ? 0.0 : bad_elapsed_seconds_[side][direction] + elapsed_seconds;
                    if (loaded) {
                        evidence_[side][direction] = true;
                        evidence_elapsed_seconds_[side][direction] = 0.0;
                    }
                }
                evidence_[side][direction] &=
                    bad_elapsed_seconds_[side][direction] + epsilon < 0.010
                    && lost_elapsed_seconds_ + epsilon < 0.050;
                confirmed_ &= evidence_[side][direction]
                           && evidence_elapsed_seconds_[side][direction] + epsilon < 0.600;
            }
        }
    }
    previous_velocity_ = velocity;
    previous_gyro_ = omega;
    previous_pulse_.setZero();
    JointReferenceRecoveryFeedback feedback;
    feedback.q = q_.cast<double>();
    feedback.dq = dq;
    feedback.gravity = gravity;
    feedback.gyro = gyro;
    feedback.estimated_height = heights_.minCoeff();
    feedback.support = plausible_;
    feedback.support_confirmed = confirmed_;
    feedback.body_clear = (confirmed_ || phase == JointReferenceRecoveryPhase::kBlend
                           || phase == JointReferenceRecoveryPhase::kRl)
                       && plausible_ && heights_.minCoeff() > 0.27f;
    return feedback;
}

Eigen::Vector2d ClosedChainSupportObserver::probe_command(bool scripted_and_released) {
    return probe_command(scripted_and_released, period_seconds_);
}

Eigen::Vector2d
    ClosedChainSupportObserver::probe_command(bool scripted_and_released, double elapsed_seconds) {
    if (!std::isfinite(elapsed_seconds) || elapsed_seconds <= 0.0 || elapsed_seconds > 0.020) {
        reset();
        return Eigen::Vector2d::Zero();
    }
    previous_pulse_.setZero();
    if (scripted_and_released && plausible_ && !confirmed_) {
        const auto stage =
            period_seconds_ == 0.005
                ? (reference_tick_ / 3) % 4
                : static_cast<std::uint64_t>(std::floor((probe_elapsed_seconds_ + 1e-12) / 0.015))
                      % 4;
        previous_pulse_[stage / 2] = stage % 2 == 0 ? 0.18f : -0.18f;
    }
    if (period_seconds_ == 0.005)
        ++reference_tick_;
    else
        probe_elapsed_seconds_ = std::fmod(probe_elapsed_seconds_ + elapsed_seconds, 0.060);
    return previous_pulse_.cast<double>();
}
} // namespace rmcs::rl
