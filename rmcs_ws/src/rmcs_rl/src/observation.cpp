#include "rl_controller.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <span>

namespace rmcs::rl {

Eigen::Quaterniond RlController::world_base_orientation_() const {
    // Eigen composes body -> world with base -> body. The CAD export rotation
    // already belongs to the frozen asset and must not be applied here.
    return orientation_->normalized() * Eigen::Quaterniond{imu_to_base_.transpose()};
}

bool RlController::read_model_state_() {
    if (!*feedback_fresh_)
        return feedback_valid_ = false;
    Eigen::Vector4d motor_q, motor_dq;
    for (int i = 0; i < 6; ++i) {
        if (!std::isfinite(*angle_inputs_[i]) || !std::isfinite(*velocity_inputs_[i])
            || !std::isfinite(*torque_feedback_inputs_[i]) || !std::isfinite(*max_torque_inputs_[i])
            || *max_torque_inputs_[i] <= 0)
            return feedback_valid_ = false;
        if (i < 4) {
            if (*fault_inputs_[i] != 0)
                return feedback_valid_ = false;
            motor_q[i] = *angle_inputs_[i];
            motor_dq[i] = *velocity_inputs_[i];
            if (strict_feedback_ && std::abs(motor_q[i]) >= dm_feedback_position_max_[i] - 0.1)
                return feedback_valid_ = false;
        }
    }
    if (strict_feedback_ && (!acceleration_.ready() || !acceleration_->allFinite()))
        return feedback_valid_ = false;
    const bool awaiting_initial_drives = !motor_control_started_ && !recovery_started_
                                      && state_ != State::kRl
                                      && (!dm_control_ready_.ready() || !*dm_control_ready_);
    if (strict_feedback_) {
        const bool samples_valid = [&] {
            const auto now = Clock::now();
            const auto now_ns =
                std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch())
                    .count();
            std::array<RecoverySampleStamp, 8> samples{};
            for (std::size_t i = 0; i < 6; ++i)
                if (motor_feedback_sequences_[i].ready() && motor_feedback_ns_[i].ready())
                    samples[i] = {*motor_feedback_sequences_[i], *motor_feedback_ns_[i]};
            if (imu_sequence_.ready() && imu_ns_.ready())
                samples[6] = {*imu_sequence_, *imu_ns_};
            if (acceleration_sequence_.ready() && acceleration_ns_.ready())
                samples[7] = {*acceleration_sequence_, *acceleration_ns_};
            recovery_sensor_status_ = recovery_sensor_guard_.update(
                now_ns > 0 ? static_cast<std::uint64_t>(now_ns) : 0, samples);
            if (!recovery_sensor_status_.valid)
                return false;
            for (int i = 0; i < 4; ++i) {
                const auto sequence = *motor_feedback_sequences_[i];
                const auto stamp = *motor_feedback_ns_[i];
                if (motor_feedback_initialized_) {
                    if (sequence < previous_motor_sequence_[i]
                        || (sequence == previous_motor_sequence_[i]
                            && stamp != previous_motor_sample_ns_[i]))
                        return false;
                    if (sequence != previous_motor_sequence_[i]) {
                        if (stamp <= previous_motor_sample_ns_[i])
                            return false;
                        const double dt = (stamp - previous_motor_sample_ns_[i]) * 1e-9;
                        if (dt > 0.05
                            || std::abs(motor_q[i] - previous_motor_angle_[i]) > 45.0 * dt + 0.15)
                            // MIT P_MAX is a quantization bound, not proof of wrapping.
                            return false;
                    }
                }
                previous_motor_sequence_[i] = sequence;
                previous_motor_sample_ns_[i] = stamp;
                previous_motor_angle_[i] = motor_q[i];
            }
            motor_feedback_initialized_ = true;
            return true;
        }();
        if (!samples_valid) {
            if (!awaiting_initial_drives)
                return feedback_valid_ = false;
            // FC/neutral-MIT handshaking can refresh the paired buses at
            // different times. Preserve hardware freshness and zero torque;
            // the first ready snapshot must pass the complete sample guard.
            recovery_sensor_guard_.reset();
            recovery_sensor_status_ = {};
            motor_feedback_initialized_ = false;
        }
    }
    if (!gyro_->allFinite() || !orientation_->coeffs().allFinite()
        || orientation_->squaredNorm() < 0.5 || orientation_->squaredNorm() > 1.5)
        return feedback_valid_ = false;
    q_.head<4>() = leg_jacobian_ * motor_q + leg_offset_;
    dq_.head<4>() = leg_jacobian_ * motor_dq;
    for (int i = 0; i < 2; ++i) {
        q_[4 + i] = wheel_scale_[i] * *angle_inputs_[4 + i];
        dq_[4 + i] = wheel_scale_[i] * *velocity_inputs_[4 + i];
    }
    return feedback_valid_ = q_.allFinite() && dq_.allFinite();
}

void RlController::update_command_reference_() {
    const double requested_vx = velocity_command_->vector.x();
    const double requested_yaw = velocity_command_->vector.z();
    const bool spinning = rmcs_msgs::is_spining(*chassis_mode_);
    constexpr double policy_dt = DeployedPolicyContract::kPolicyPeriodSeconds;
    if (spinning)
        vx_reference_ = 0.0;
    else
        vx_reference_ += std::clamp(
            requested_vx - vx_reference_, -policy_profile_.forward_slew * policy_dt,
            policy_profile_.forward_slew * policy_dt);
    yaw_reference_ += std::clamp(
        requested_yaw - yaw_reference_, -DeployedPolicyContract::kYawSlew * policy_dt,
        DeployedPolicyContract::kYawSlew * policy_dt);

    // Respect the wheel geometry and the combined command envelope.
    const double max_linear = DeployedPolicyContract::kWheelSpeedLimit * wheel_radius_;
    const double demand = std::max(
        std::abs(vx_reference_ - wheel_track_ * yaw_reference_ / 2),
        std::abs(vx_reference_ + wheel_track_ * yaw_reference_ / 2));
    if (demand > max_linear) {
        const double scale = max_linear / demand;
        vx_reference_ *= scale;
        yaw_reference_ *= scale;
    }
    if (std::abs(vx_reference_ * yaw_reference_) > 2.06)
        vx_reference_ = std::copysign(2.06 / std::abs(yaw_reference_), vx_reference_);
}

bool RlController::assemble_observation_(bool shadow_recovery) {
    if (!feedback_valid_ || !std::isfinite(*height_command_)
        || !velocity_command_->vector.allFinite() || *height_command_ < policy_profile_.height_min
        || *height_command_ > policy_profile_.height_max || !std::isfinite(*jump_apex_command_))
        return false;
    const bool hold_command = recovery_motion_hold_();
    if (!shadow_recovery && !hold_command)
        update_command_reference_();
    else if (hold_command)
        vx_reference_ = yaw_reference_ = 0.0;
    if (!shadow_recovery && (policy_profile_.name != kV6PolicyProfile.name || !hold_command)
        && std::abs(*height_command_ - height_target_) > 1e-6) {
        height_from_ = height_reference_;
        height_target_ = *height_command_;
        height_start_ = *timestamp_;
    }
    const double u = std::clamp(
        std::chrono::duration<double>(*timestamp_ - height_start_).count()
            / height_transition_seconds_,
        0.0, 1.0);
    height_reference_ =
        height_from_ + (height_target_ - height_from_) * (3 * u * u - 2 * u * u * u);
    const Eigen::Quaterniond q_world_base = world_base_orientation_();
    const Eigen::Vector3d gravity = q_world_base.conjugate() * -Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d omega = imu_to_base_ * *gyro_;

    auto observation = std::span{observation_};
    auto command = observation.subspan<ObservationLayout::kCommand, 3>();
    command[0] = (shadow_recovery || hold_command) ? 0.0 : vx_reference_;
    command[1] = 0.0; // Both frozen flat actors have no lateral command capability.
    command[2] = (shadow_recovery || hold_command) ? 0.0 : yaw_reference_;
    observation[ObservationLayout::kHeight] =
        ((shadow_recovery || hold_command) ? DeployedPolicyContract::kNominalHeight
                                           : height_reference_)
        * 5.0;

    auto angular_velocity = observation.subspan<ObservationLayout::kAngularVelocity, 3>();
    auto projected_gravity = observation.subspan<ObservationLayout::kProjectedGravity, 3>();
    for (int i = 0; i < 3; ++i) {
        angular_velocity[i] = omega[i] * 0.5;
        projected_gravity[i] = gravity[i];
    }

    auto joint_position = observation.subspan<ObservationLayout::kJointPosition, 6>();
    std::ranges::fill(joint_position, 0.0f); // wheel positions are deliberately zero
    for (int i = 0; i < 4; ++i)
        joint_position[i] = std::remainder(q_[i] - nominal_[i], 2 * std::numbers::pi);

    auto joint_velocity = observation.subspan<ObservationLayout::kJointVelocity, 6>();
    for (int i = 0; i < 6; ++i)
        joint_velocity[i] = dq_[i] * 0.1;
    auto previous_action = observation.subspan<ObservationLayout::kPreviousAction, 6>();
    // Native recovery stores the effective script/actor reference from the
    // last control sample of the preceding policy interval.
    std::ranges::copy(previous_action_, previous_action.begin());

    auto context = observation.subspan<ObservationLayout::kContext, 7>();
    std::ranges::fill(context, 0.0f);
    context[ObservationLayout::kNormal] = 1.0f;
    for (auto& x : observation) {
        if (!std::isfinite(x))
            return false;
        x = std::clamp(x, -100.0f, 100.0f);
    }
    return true;
}

} // namespace rmcs::rl
