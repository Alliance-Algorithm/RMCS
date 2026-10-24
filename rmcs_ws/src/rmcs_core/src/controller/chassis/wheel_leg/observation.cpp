#include "wheel_leg_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rmcs_core::controller::chassis {

Eigen::Quaterniond WheelLegController::world_base_orientation_() const {
    // Hardware applies the configured sensor mounting before attitude fusion.
    return orientation_->normalized();
}

bool WheelLegController::read_model_state_() {
    if (!*feedback_fresh_)
        return feedback_valid_ = false;
    Eigen::Vector4d motor_q, motor_dq;
    for (int i = 0; i < 6; ++i) {
        if (!std::isfinite(*angle_inputs_[i]) || !std::isfinite(*velocity_inputs_[i])
            || !std::isfinite(*max_torque_inputs_[i]) || *max_torque_inputs_[i] <= 0)
            return feedback_valid_ = false;
        if (i < 4) {
            motor_q[i] = *angle_inputs_[i];
            motor_dq[i] = *velocity_inputs_[i];
        }
    }
    if (strict_feedback_) {
        const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                Clock::now().time_since_epoch())
                                .count();
        std::array<FeedbackSampleStamp, 8> samples{};
        for (std::size_t i = 0; i < 6; ++i)
            if (motor_feedback_sequences_[i].ready() && motor_feedback_ns_[i].ready())
                samples[i] = {*motor_feedback_sequences_[i], *motor_feedback_ns_[i]};
        if (imu_sequence_.ready() && imu_ns_.ready())
            samples[6] = {*imu_sequence_, *imu_ns_};
        if (acceleration_sequence_.ready() && acceleration_ns_.ready())
            samples[7] = {*acceleration_sequence_, *acceleration_ns_};
        sensor_status_ =
            sensor_guard_.update(now_ns > 0 ? static_cast<std::uint64_t>(now_ns) : 0, samples);
        const bool awaiting_initial_drives = state_ != State::kRl
                                          && (!dm_control_ready_.ready() || !*dm_control_ready_);
        if (!sensor_status_.valid && !awaiting_initial_drives)
            return feedback_valid_ = false;
    }
    if (!gyro_->allFinite() || !orientation_->coeffs().allFinite()
        || !std::isfinite(orientation_->norm()) || orientation_->norm() == 0.0)
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
    // The upstream input owns the deadzone and manual turn intent. Capture
    // once per straight segment, including direct forward/reverse changes.
    if (heading_hold_enabled_ && !spinning && requested_vx != 0.0 && requested_yaw == 0.0) {
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
    vx_reference_ = spinning
                      ? 0.0
                      : std::clamp(
                            requested_vx, -policy_profile_.forward_limit,
                            policy_profile_.forward_limit);
    yaw_reference_ = desired_yaw;
}

bool WheelLegController::assemble_observation_() {
    if (!feedback_valid_ || !std::isfinite(*height_command_)
        || !velocity_command_->vector.allFinite())
        return false;
    update_command_reference_();
    const double requested_height = std::clamp(
        *height_command_, policy_profile_.height_min, policy_profile_.height_max);
    if (height_command_is_reference_) {
        // The chassis integrates the height input. Follow its continuous reference.
        const double max_step = height_reference_rate_max_ / inference_frequency_;
        height_reference_ += std::clamp(requested_height - height_reference_, -max_step, max_step);
        height_from_ = height_reference_;
        height_target_ = requested_height;
        height_start_ = *timestamp_;
    } else {
        if (std::abs(requested_height - height_target_) > 1e-6) {
            height_from_ = height_reference_;
            height_target_ = requested_height;
            height_start_ = *timestamp_;
        }
        const double u = std::clamp(
            std::chrono::duration<double>(*timestamp_ - height_start_).count()
                / height_transition_seconds_,
            0.0, 1.0);
        height_reference_ =
            height_from_ + (height_target_ - height_from_) * (3 * u * u - 2 * u * u * u);
    }
    *policy_command_output_ = Eigen::Vector3d{vx_reference_, 0.0, yaw_reference_};
    *policy_height_output_ = height_reference_;
    return true;
}

bool WheelLegController::publish_unified_observation_() {
    if (next_policy_frame_sequence_ == std::numeric_limits<std::size_t>::max())
        return false;
    std::array<double, 6> position{}, velocity{};
    for (std::size_t i = 0; i < 6; ++i) {
        position[i] = q_[i];
        velocity[i] = dq_[i];
    }
    if (!policy_targets_valid_)
        applied_policy_action_ = measured_policy_history(position);
    const Eigen::Vector3d omega = *gyro_;
    const Eigen::Vector3d gravity =
        world_base_orientation_().conjugate() * -Eigen::Vector3d::UnitZ();
    const auto& command = *policy_command_output_;
    const auto frame = unified_observation_.sample(
        ++next_policy_frame_sequence_, position, velocity, {omega.x(), omega.y(), omega.z()},
        {gravity.x(), gravity.y(), gravity.z()}, {command.x(), command.y(), command.z()},
        *policy_height_output_, applied_policy_action_, state_ == State::kRl);
    if (!frame)
        return false;
    std::ranges::copy(*frame, policy_frame_output_->begin());
    *policy_frame_sequence_output_ = next_policy_frame_sequence_;
    return true;
}

void WheelLegController::publish_model_feedback_() {
    // The bridge samples on its own policy clock. Keep its physical inputs
    // current on every executor tick, even between command-reference updates.
    *policy_gyro_output_ = *gyro_;
    *policy_gravity_output_ = world_base_orientation_().conjugate() * -Eigen::Vector3d::UnitZ();
    for (int i = 0; i < 6; ++i) {
        *model_angle_outputs_[i] = q_[i];
        *model_velocity_outputs_[i] = dq_[i];
    }
}

} // namespace rmcs_core::controller::chassis
