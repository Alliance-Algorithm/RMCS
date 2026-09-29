#include "rl_controller.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace rmcs::rl {

bool RlController::update_prepare_() {
    const double dt = 1.0 / *update_rate_;
    bool reached = true;
    for (int i = 0; i < 4; ++i) {
        const double delta = std::remainder(nominal_[i] - targets_[i], 2 * std::numbers::pi);
        targets_[i] += std::clamp(delta, -prepare_max_velocity_ * dt, prepare_max_velocity_ * dt);
        reached &= std::abs(std::remainder(nominal_[i] - q_[i], 2 * std::numbers::pi))
                 < prepare_reach_threshold_;
    }
    targets_[4] = targets_[5] = 0.0;
    const Eigen::Quaterniond q_world_base =
        orientation_->normalized() * Eigen::Quaterniond{imu_to_base_.transpose()};
    const double gravity_z = (q_world_base.conjugate() * -Eigen::Vector3d::UnitZ()).z();
    if (!reached || -gravity_z < std::cos(prepare_max_tilt_rad_)
        || gyro_->norm() > prepare_max_angular_velocity_
        || dq_.cwiseAbs().maxCoeff() > prepare_max_joint_velocity_) {
        prepare_stable_since_.reset();
        return false;
    }
    if (!prepare_stable_since_ || *timestamp_ < *prepare_stable_since_)
        prepare_stable_since_ = *timestamp_;
    return *timestamp_ - *prepare_stable_since_
        >= std::chrono::duration<double>{prepare_stable_seconds_};
}

std::optional<RecoveryFeedback> RlController::observe_recovery_() {
    if (!recovery_observer_ || !acceleration_.ready() || !acceleration_ns_.ready()
        || !acceleration_->allFinite() || *acceleration_ns_ == 0)
        return std::nullopt;
    const auto now = Clock::now();
    const auto stamp =
        Clock::time_point{std::chrono::nanoseconds{static_cast<std::int64_t>(*acceleration_ns_)}};
    const auto age = now - stamp;
    if (age < Clock::duration::zero() || age > std::chrono::milliseconds{30})
        return std::nullopt;
    // Use the same canonical policy base_link as the 35D observation. Do not
    // apply the original CAD mesh rotation to the IMU a second time.
    const Eigen::Quaterniond world_base =
        orientation_->normalized() * Eigen::Quaterniond{imu_to_base_.transpose()};
    const Eigen::Vector3d gravity = world_base.conjugate() * -Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d gyro = imu_to_base_ * *gyro_;
    const Eigen::Vector3d acceleration = imu_to_base_ * *acceleration_;
    auto feedback = recovery_observer_->update(q_, dq_, gravity, gyro, acceleration, recovery_dt_);
    feedback.stamp = recovery_update_time_;
    last_recovery_feedback_ = feedback;
    return feedback;
}

bool RlController::advance_recovery_() {
    auto feedback = observe_recovery_();
    if (!feedback) {
        return false;
    }
    if (!recovery_started_) {
        if (!recovery_.start(*feedback)) {
            return false;
        }
        recovery_started_ = true;
    }
    recovery_command_ = recovery_.step(*feedback, recovery_dt_);
    if (recovery_command_.phase == RecoveryPhase::kFailed) {
        return false;
    }
    if (recovery_observer_ && state_ == State::kPrepare) {
        const bool probing = recovery_command_.phase == RecoveryPhase::kPrepare
                          || recovery_command_.phase == RecoveryPhase::kCapture;
        const Eigen::Vector2d pulse = recovery_observer_->probe_command(probing, *feedback);
        for (int i = 0; i < 2; ++i)
            if (pulse[i] != 0.0)
                recovery_command_.torque[4 + i] = pulse[i];
    }
    if (recovery_command_.phase == RecoveryPhase::kComplete)
        enter_(State::kRl);
    return true;
}

} // namespace rmcs::rl
