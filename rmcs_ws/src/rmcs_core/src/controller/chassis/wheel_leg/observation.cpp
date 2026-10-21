#include "wheel_leg_controller.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <span>

namespace rmcs_core::controller::chassis {

Eigen::Quaterniond WheelLegController::world_base_orientation_() const {
    // Eigen composes body -> world with base -> body. The CAD export rotation
    // already belongs to the frozen asset and must not be applied here.
    return orientation_->normalized() * base_to_imu_orientation_;
}

bool WheelLegController::read_model_state_() {
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
                        const double delta = motor_q[i] - previous_motor_angle_[i];
                        const double displacement =
                            leg_target_winding_ == LegTargetWinding::kPairedBranch107
                                ? std::remainder(delta, 2 * std::numbers::pi)
                                : delta;
                        if (dt > 0.05 || std::abs(displacement) > 45.0 * dt + 0.15)
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
    const Eigen::Vector4d measured = leg_jacobian_ * motor_q + leg_offset_;
    std::array<double, 4> raw{}, previous{};
    for (int i = 0; i < 4; ++i) {
        raw[i] = measured[i];
        previous[i] = leg_feedback_initialized_ ? q_[i] : measured[i];
    }
    const auto continuous = wind_leg_feedback(raw, previous, leg_target_winding_);
    q_.head<4>() = Eigen::Map<const Eigen::Vector4d>{continuous.data()};
    leg_feedback_initialized_ = true;
    dq_.head<4>() = leg_jacobian_ * motor_dq;
    for (int i = 0; i < 2; ++i) {
        q_[4 + i] = wheel_scale_[i] * *angle_inputs_[4 + i];
        dq_[4 + i] = wheel_scale_[i] * *velocity_inputs_[4 + i];
    }
    return feedback_valid_ = q_.allFinite() && dq_.allFinite();
}

void WheelLegController::reset_heading_hold_() {
    heading_target_rad_.reset();
    *heading_hold_active_output_ = false;
    *heading_yaw_output_ = 0.0;
    *heading_target_output_ = 0.0;
    *heading_error_output_ = 0.0;
    *heading_raw_rate_output_ = 0.0;
    *heading_limited_rate_output_ = 0.0;
}

void WheelLegController::update_command_reference_() {
    const double requested_vx = velocity_command_->vector.x();
    const double requested_yaw = velocity_command_->vector.z();
    const bool spinning = rmcs_msgs::is_spining(*chassis_mode_);
    const Eigen::Vector3d forward = world_base_orientation_() * Eigen::Vector3d::UnitX();
    const double yaw = std::atan2(forward.y(), forward.x());
    double desired_yaw = requested_yaw;
    const bool blending = (recovery_started_ && joint_reference_recovery_
                           && joint_reference_recovery_->command().blending)
                       || (!recovery_started_ && policy_profile_.name == kV6PolicyProfile.name
                           && takeover_blend_seconds_ > 0.0 && takeover_blend_fraction_ < 1.0);
    // The upstream input owns the deadzone and manual turn intent. Capture
    // once per straight segment, including direct forward/reverse changes.
    if (heading_hold_enabled_ && !blending && !spinning && requested_vx != 0.0
        && requested_yaw == 0.0) {
        if (!heading_target_rad_)
            heading_target_rad_ = yaw;
        const double error =
            std::atan2(std::sin(*heading_target_rad_ - yaw), std::cos(*heading_target_rad_ - yaw));
        desired_yaw = std::clamp(
            heading_hold_gain_ * error, -heading_hold_yaw_rate_max_, heading_hold_yaw_rate_max_);
        *heading_hold_active_output_ = true;
        *heading_target_output_ = *heading_target_rad_;
        *heading_error_output_ = error;
    } else {
        reset_heading_hold_();
    }
    *heading_yaw_output_ = yaw;
    *heading_raw_rate_output_ = desired_yaw;
    desired_yaw = std::clamp(
        desired_yaw, -policy_profile_.negative_yaw_limit, policy_profile_.positive_yaw_limit);
    *heading_limited_rate_output_ = desired_yaw;
    // Match training: commands reach the actor on the next policy tick.
    vx_reference_ = spinning ? 0.0 : requested_vx;
    yaw_reference_ = desired_yaw;

    // Respect the wheel geometry and the combined command envelope.
    const double max_linear = wheel_speed_limit_ * wheel_radius_;
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

bool WheelLegController::assemble_observation_(bool shadow_recovery) {
    if (!feedback_valid_ || !std::isfinite(*height_command_)
        || !velocity_command_->vector.allFinite() || *height_command_ < policy_profile_.height_min
        || *height_command_ > policy_profile_.height_max || !std::isfinite(*jump_apex_command_))
        return false;
    const bool hold_command = recovery_motion_hold_();
    if (!shadow_recovery && !hold_command)
        update_command_reference_();
    else {
        reset_heading_hold_();
        if (hold_command)
            vx_reference_ = yaw_reference_ = 0.0;
    }
    const bool height_command_active =
        !shadow_recovery && (policy_profile_.name != kV6PolicyProfile.name || !hold_command);
    if (height_command_is_reference_) {
        if (height_command_active) {
            // The chassis integrates the height input. Follow that continuous
            // reference without restarting a full transition for every sample.
            // Limit pending commands on release from recovery as well.
            const double max_step = height_reference_rate_max_ / inference_frequency_;
            height_reference_ +=
                std::clamp(*height_command_ - height_reference_, -max_step, max_step);
            height_from_ = height_reference_;
            height_target_ = *height_command_;
            height_start_ = *timestamp_;
        }
    } else {
        if (height_command_active && std::abs(*height_command_ - height_target_) > 1e-6) {
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
    }
    *policy_command_output_ = Eigen::Vector3d{
        (shadow_recovery || hold_command) ? 0.0 : vx_reference_, 0.0,
        (shadow_recovery || hold_command) ? 0.0 : yaw_reference_};
    *policy_height_output_ = (shadow_recovery || hold_command)
                               ? DeployedPolicyContract::kNominalHeight
                               : height_reference_;
    return true;
}

void WheelLegController::publish_model_feedback_() {
    // The bridge samples on its own policy clock. Keep its physical inputs
    // current on every executor tick, even between command-reference updates.
    *policy_gyro_output_ = imu_to_base_ * *gyro_;
    *policy_gravity_output_ = world_base_orientation_().conjugate() * -Eigen::Vector3d::UnitZ();
    for (int i = 0; i < 6; ++i) {
        *model_angle_outputs_[i] = q_[i];
        *model_velocity_outputs_[i] = dq_[i];
    }
}

} // namespace rmcs_core::controller::chassis
