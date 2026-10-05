#include "v6_recovery_observer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include <eigen3/Eigen/Geometry>

namespace rmcs::rl {
V6RecoveryObserver::V6RecoveryObserver(std::array<V6RecoverySideGeometry, 2> geometry)
    : geometry_{std::move(geometry)} {
    reset();
}

void V6RecoveryObserver::reset() {
    initialized_ = geometry_valid_ = plausible_ = confirmed_ = false;
    winding_.setZero();
    previous_velocity_.setZero();
    previous_gyro_.setZero();
    previous_pulse_.setZero();
    evidence_ = {};
    bad_samples_ = {};
    for (auto& side : evidence_age_)
        side.fill(std::numeric_limits<float>::infinity());
    lost_seconds_ = 0.0f;
    reference_tick_ = 0;
}

V6RecoveryFeedback V6RecoveryObserver::observe(
    const RecoverySensorData&, const V6RecoveryVector6& raw_q, const V6RecoveryVector6& dq,
    const Eigen::Vector3d& gravity, const Eigen::Vector3d& gyro,
    const Eigen::Vector3d& acceleration, V6RecoveryPhase phase) {
    constexpr float dt = 0.005f, tau = 2.0f * std::numbers::pi_v<float>;
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
    lost_seconds_ = plausible_ ? 0.0f : lost_seconds_ + dt;
    confirmed_ = plausible_;
    for (int side = 0; side < 2; ++side) {
        const bool tested = plausible_ && std::abs(previous_pulse_[side]) >= 0.14f;
        const bool loaded =
            tested && std::abs(wheel_acceleration[side]) < 100.0f && gyro_acceleration < 90.0f;
        for (int direction = 0; direction < 2; ++direction) {
            const bool test_direction =
                tested && (previous_pulse_[side] > 0.0f) == (direction != 0);
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
        }
    }
    previous_velocity_ = velocity;
    previous_gyro_ = omega;
    previous_pulse_.setZero();
    V6RecoveryFeedback feedback;
    feedback.q = q_.cast<double>();
    feedback.dq = dq;
    feedback.gravity = gravity;
    feedback.gyro = gyro;
    feedback.estimated_height = heights_.minCoeff();
    feedback.support = plausible_;
    feedback.support_confirmed = confirmed_;
    feedback.body_clear =
        (confirmed_ || phase == V6RecoveryPhase::kBlend || phase == V6RecoveryPhase::kRl)
        && plausible_ && heights_.minCoeff() > 0.27f;
    return feedback;
}

Eigen::Vector2d V6RecoveryObserver::probe_command(bool scripted_and_released) {
    previous_pulse_.setZero();
    if (scripted_and_released && plausible_ && !confirmed_) {
        const auto stage = (reference_tick_ / 3) % 4;
        previous_pulse_[stage / 2] = stage % 2 == 0 ? 0.18f : -0.18f;
    }
    ++reference_tick_;
    return previous_pulse_.cast<double>();
}
} // namespace rmcs::rl
