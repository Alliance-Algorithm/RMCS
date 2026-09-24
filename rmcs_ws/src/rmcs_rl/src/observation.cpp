#include "rl_controller.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <span>

namespace rmcs::rl {

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
    const bool rotating_reference = *chassis_mode_ == rmcs_msgs::ChassisMode::SPIN_FAST;
    constexpr double policy_dt = 0.02;
    if (rotating_reference)
        vx_reference_ = requested_vx; // training bypasses ordinary vx slew for rotating references
    else
        vx_reference_ +=
            std::clamp(requested_vx - vx_reference_, -0.6 * policy_dt, 0.6 * policy_dt);
    yaw_reference_ += std::clamp(requested_yaw - yaw_reference_, -4.0 * policy_dt, 4.0 * policy_dt);

    // Respect the wheel geometry and the V5 combined command envelope.
    const double max_linear = 90.0 * wheel_radius_;
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

bool RlController::assemble_observation_() {
    if (!feedback_valid_ || !std::isfinite(*height_command_)
        || !velocity_command_->vector.allFinite() || *height_command_ < 0.21
        || *height_command_ > 0.35 || !std::isfinite(*jump_apex_command_))
        return false;
    update_command_reference_();
    if (std::abs(*height_command_ - height_target_) > 1e-6) {
        height_from_ = height_reference_;
        height_target_ = *height_command_;
        height_start_ = *timestamp_;
    }
    // Same cubic zero-end-velocity profile used by the V5 height reference.
    const double u = std::clamp(
        std::chrono::duration<double>(*timestamp_ - height_start_).count()
            / height_transition_seconds_,
        0.0, 1.0);
    height_reference_ =
        height_from_ + (height_target_ - height_from_) * (3 * u * u - 2 * u * u * u);
    const bool rotating_reference = *chassis_mode_ == rmcs_msgs::ChassisMode::SPIN_FAST;
    // The V5 task starts jump requests after a standing dwell (about 0.8 s).
    const bool jumping =
        *jump_command_ && *timestamp_ - rl_start_ >= std::chrono::milliseconds(800);
    if (jumping && !jump_was_requested_)
        jump_start_ = *timestamp_;
    jump_was_requested_ = jumping;
    const auto orientation = orientation_->normalized();
    const Eigen::Quaterniond q_world_base =
        orientation * Eigen::Quaterniond{imu_to_base_.transpose()};
    const Eigen::Vector3d gravity = q_world_base.conjugate() * -Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d omega = imu_to_base_ * *gyro_;

    auto observation = std::span{observation_};
    auto command = observation.subspan<ObservationLayout::kCommand, 3>();
    command[0] = vx_reference_;
    command[1] = rotating_reference ? velocity_command_->vector.y() : 0.0;
    command[2] = yaw_reference_;
    observation[ObservationLayout::kHeight] = height_reference_ * 5.0;

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
    std::ranges::copy(previous_action_, previous_action.begin()); // clipped P order

    auto context = observation.subspan<ObservationLayout::kContext, 7>();
    std::ranges::fill(context, 0.0f);
    context[ObservationLayout::kNormal] = jumping ? 0.0f : 1.0f;
    context[ObservationLayout::kJumpRequest] = jumping ? 1.0f : 0.0f;
    context[ObservationLayout::kJumpApex] = jumping ? *jump_apex_command_ * 5.0 : 0.0f;
    context[ObservationLayout::kJumpElapsed] =
        jumping
            ? std::clamp(std::chrono::duration<double>(*timestamp_ - jump_start_).count(), 0.0, 5.0)
            : 0.0f;
    for (auto& x : observation) {
        if (!std::isfinite(x))
            return false;
        x = std::clamp(x, -100.0f, 100.0f);
    }
    return true;
}

} // namespace rmcs::rl
