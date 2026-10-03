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
    const Eigen::Quaterniond q_world_base = world_base_orientation_();
    const double gravity_z = (q_world_base.conjugate() * -Eigen::Vector3d::UnitZ()).z();
    if (policy_profile_.name == kV6PolicyProfile.name) {
        // A free-base balancing actor must take over from its upright reset
        // domain before passive springs/static leg PD tip the unbalanced base.
        // V6 never waits for a non-balancing controller to demonstrate static
        // balance. Fresh sensors, drive readiness and the real hinge bounds
        // remain mandatory; the legacy/recovery dwell below is unchanged.
        if (!feedback_valid_ || !recovery_sensor_status_.valid
            || -gravity_z < std::cos(prepare_max_tilt_rad_)
            || (imu_to_base_ * *gyro_).norm() > v6_capture_max_angular_velocity_
            || dq_.head<4>().cwiseAbs().maxCoeff() > v6_capture_max_leg_velocity_
            || dq_.tail<2>().cwiseAbs().maxCoeff() > v6_capture_max_wheel_velocity_)
            return false;
        for (int i = 0; i < 4; ++i)
            if (std::abs(std::remainder(nominal_[i] - q_[i], 2 * std::numbers::pi))
                > v6_capture_max_leg_error_rad_)
                return false;
        for (int side = 0; side < 2; ++side) {
            const int hip = 2 * side;
            const double relative = hinge_coeff_[hip] * q_[hip]
                                    + hinge_coeff_[hip + 1] * q_[hip + 1] + hinge_bias_[side];
            if (relative < hinge_min_[side] + hinge_margin_
                || relative > hinge_max_[side] - hinge_margin_)
                return false;
        }
        return true;
    }
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
    if (!recovery_observer_ || !recovery_sensor_status_.valid || !acceleration_.ready()
        || !acceleration_->allFinite())
        return std::nullopt;
    RecoverySensorData sensors;
    sensors.steady_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
            .count());
    sensors.imu_feedback_ns = *imu_ns_;
    sensors.imu_feedback_sequence = *imu_sequence_;
    for (int side = 0; side < 2; ++side) {
        const int axis = 4 + side;
        sensors.wheel_feedback_ns[side] = *motor_feedback_ns_[axis];
        sensors.wheel_feedback_sequence[side] = *motor_feedback_sequences_[axis];
        sensors.wheel_torque_feedback_nm[side] =
            *torque_feedback_inputs_[axis] / wheel_scale_[side];
        if (wheel_submitted_torque_[side].ready() && wheel_submitted_ns_[side].ready()
            && wheel_submitted_kind_[side].ready()) {
            sensors.wheel_torque_submitted_nm[side] =
                *wheel_submitted_torque_[side] / wheel_scale_[side];
            sensors.wheel_torque_submitted_ns[side] = *wheel_submitted_ns_[side];
            sensors.wheel_tx_kind[side] = *wheel_submitted_kind_[side];
        }
    }
    const Eigen::Quaterniond world_base = world_base_orientation_();
    sensors.world_base_orientation = world_base;
    const Eigen::Vector3d gravity = world_base.conjugate() * -Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d gyro = imu_to_base_ * *gyro_;
    const Eigen::Vector3d acceleration = imu_to_base_ * *acceleration_;
    auto feedback =
        recovery_observer_->update(q_, dq_, gravity, gyro, acceleration, recovery_dt_, sensors);
    feedback.stamp = recovery_update_time_;
    last_recovery_feedback_ = feedback;
    return feedback;
}

bool RlController::advance_recovery_() {
    const bool had_sensor_baseline =
        recovery_observer_ && recovery_observer_->sensor_baseline_ready();
    auto feedback = observe_recovery_();
    if (!feedback) {
        return false;
    }
    if (!recovery_started_) {
        if (!feedback->geometry_valid || !feedback->spring_compensation_valid)
            return false;
        // First acquire derivative baselines, even if the drives are already
        // ready. A cold observer must not choose a fallen route for an upright body.
        if (!had_sensor_baseline || !recovery_observer_->sensor_baseline_ready()) {
            recovery_command_ = {};
            return true;
        }
        if (!recovery_.start(*feedback)) {
            return false;
        }
        recovery_started_ = true;
    }
    const auto previous_phase = recovery_command_.phase;
    recovery_command_ = recovery_.step(*feedback, recovery_dt_);
    if (previous_phase != RecoveryPhase::kBlend
        && recovery_command_.phase == RecoveryPhase::kBlend) {
        // Clear history once without changing the global 50 Hz policy clock.
        // Hold the latest shadow targets until the next scheduled evaluation.
        previous_action_.fill(0.0f);
    }
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
