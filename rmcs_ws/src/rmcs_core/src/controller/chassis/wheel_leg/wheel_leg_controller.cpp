#include "wheel_leg_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

#include "leg_target_winding.hpp"

namespace rmcs_core::controller::chassis {
namespace {
bool almost_integer(double value) {
    return std::isfinite(value) && value >= 1.0 && std::abs(value - std::round(value)) < 1e-6;
}

} // namespace

WheelLegController::~WheelLegController() = default;

bool accepts_motion_command(
    bool jump, double height, rmcs_msgs::ChassisMode mode,
    const rmcs_description::BaseLink::DirectionVector& command, const PolicyProfile& profile) {
    if (!std::isfinite(height) || !command.vector.allFinite() || jump || height < profile.height_min
        || height > profile.height_max || std::abs(command.vector.x()) > profile.forward_limit
        || command.vector.z() < -profile.negative_yaw_limit
        || command.vector.z() > profile.positive_yaw_limit
        || (mode != rmcs_msgs::ChassisMode::AUTO && !rmcs_msgs::is_spining(mode)))
        return false;
    if (rmcs_msgs::is_spining(mode)
        && (std::abs(command.vector.x()) > 1e-3 || std::abs(command.vector.y()) > 1e-3))
        return false;
    return std::abs(command.vector.y()) <= 1e-3;
}

void WheelLegController::before_updating() {
    inference_frequency_ = *policy_rate_input_;
    if (!std::isfinite(inference_frequency_) || inference_frequency_ <= 0.0
        || !almost_integer(pd_frequency_ / inference_frequency_))
        throw std::runtime_error("Bridge policy rate must divide the feedback PD frequency");
    for (std::size_t i = 0; i < kMotorNames.size(); ++i) {
        action_scales_[i] = *policy_scale_inputs_[i];
        action_clip_min_[i] = *policy_clip_min_inputs_[i];
        action_clip_max_[i] = *policy_clip_max_inputs_[i];
        if (!std::isfinite(action_scales_[i]) || action_scales_[i] == 0.0
            || !std::isfinite(action_clip_min_[i]) || !std::isfinite(action_clip_max_[i])
            || action_clip_min_[i] > 0.0 || action_clip_max_[i] < 0.0
            || action_clip_min_[i] >= action_clip_max_[i])
            throw std::runtime_error("Invalid wheel-leg bridge action contract");
    }
    wheel_speed_limit_ = std::min(
        {-action_clip_min_[4], action_clip_max_[4], -action_clip_min_[5], action_clip_max_[5]});
    if (wheel_speed_limit_ <= 0.0)
        throw std::runtime_error("Wheel action bounds must allow both drive directions");
    validate_leg_target_model_spec(
        leg_target_winding_, *policy_model_spec_input_, "Wheel-leg bridge model");
    policy_ready_ = true;
    // Executor::start sets /predefined/update_rate AFTER before_updating().
    // Calculate divisors on the first update instead.
    last_reset_count_ = *reset_count_;
    use_motor_position_pd_ = motor_position_pd_.ready() && *motor_position_pd_;
    if (use_motor_position_pd_) {
        if (policy_profile_.name != kV6PolicyProfile.name || !motor_position_kp_.ready()
            || !motor_position_kd_.ready() || *motor_position_kp_ != policy_profile_.leg_kp
            || *motor_position_kd_ != policy_profile_.leg_kd || takeover_blend_seconds_ != 0.0)
            throw std::runtime_error("Motor position PD requires native gains and direct takeover");
        if ((leg_jacobian_.array().abs() - Eigen::Matrix4d::Identity().array()).abs().maxCoeff()
            > 1e-9)
            throw std::runtime_error("Motor position PD requires independent signed 1:1 axes");
    }
    RCLCPP_INFO(
        get_logger(), "Joint PD backend: %s, Kp %.1f, Kd %.1f",
        use_motor_position_pd_ ? "DM internal MIT" : "host torque", policy_profile_.leg_kp,
        policy_profile_.leg_kd);
    const bool native_profile_ready =
        policy_profile_.name != kV6PolicyProfile.name || joint_reference_profile_ready_;
    if (!calibration_ready_ || !soft_limits_ready_ || !imu_alignment_ready_ || !policy_ready_
        || !native_profile_ready)
        RCLCPP_WARN(
            get_logger(),
            "RL disarmed: calibration_ready=%d, soft_limits_ready=%d, imu_alignment_ready=%d, "
            "policy_ready=%d, recovery_profile_ready=%d",
            calibration_ready_, soft_limits_ready_, imu_alignment_ready_, policy_ready_,
            native_profile_ready);
    if (!recovery_enabled_)
        RCLCPP_INFO(get_logger(), "Self-righting disabled: recovery_enabled=false");
    else if (!recovery_profile_ready_)
        RCLCPP_WARN(get_logger(), "Self-righting disarmed: recovery_profile_ready=false");
    RCLCPP_INFO(
        get_logger(), "Heading hold %s: P %.2f 1/s, correction limit %.2f rad/s at policy rate",
        heading_hold_enabled_ ? "enabled" : "disabled", heading_hold_gain_,
        heading_hold_yaw_rate_max_);
}

void WheelLegController::clear_outputs_() {
    for (auto& output : torque_outputs_)
        *output = 0.0;
    for (auto& output : angle_outputs_)
        *output = std::numeric_limits<double>::quiet_NaN();
    *joint_command_ns_ = 0;
}

void WheelLegController::log_fault_(
    std::optional<RecoveryFailure> reason, std::source_location location) const {
    if (fault_latched_)
        return;
    std::array<double, 2> hinge{};
    for (int side = 0; side < 2; ++side) {
        const int hip = 2 * side;
        hinge[side] =
            hinge_coeff_[hip] * q_[hip] + hinge_coeff_[hip + 1] * q_[hip + 1] + hinge_bias_[side];
    }
    RCLCPP_ERROR(
        get_logger(),
        "RL stopped at %s:%u: state=%d failure=%d feedback_fresh=%d sensor_valid=%d "
        "sensor_issue=%d mask=0x%02x age_ms[motor,imu,accel]=[%.3f,%.3f,%.3f] "
        "last_hinge=[%.4f,%.4f] dq_max[leg,wheel]=[%.3f,%.3f] gyro=%.3f height_command=%.3f "
        "command_vx=%.3f obs_vx=%.3f wheel_ref=[%.3f,%.3f] wheel_dq=[%.3f,%.3f] "
        "leg_torque_fb=[%.3f,%.3f,%.3f,%.3f]",
        location.file_name(), location.line(), static_cast<int>(state_),
        reason ? static_cast<int>(*reason) : 0, *feedback_fresh_, recovery_sensor_status_.valid,
        static_cast<int>(recovery_sensor_status_.issue),
        static_cast<unsigned>(recovery_sensor_status_.invalid_mask),
        recovery_sensor_status_.maximum_motor_age_ms, recovery_sensor_status_.imu_age_ms,
        recovery_sensor_status_.acceleration_age_ms, hinge[0], hinge[1],
        dq_.head<4>().cwiseAbs().maxCoeff(), dq_.tail<2>().cwiseAbs().maxCoeff(), gyro_->norm(),
        *height_command_, velocity_command_->vector.x(), policy_command_output_->x(),
        policy_targets_[4], policy_targets_[5], dq_[4], dq_[5], *torque_feedback_inputs_[0],
        *torque_feedback_inputs_[1], *torque_feedback_inputs_[2], *torque_feedback_inputs_[3]);
}

void WheelLegController::latch_fault_(
    std::optional<RecoveryFailure> reason, std::source_location location) {
    log_fault_(reason, location);
    fault_latched_ = true;
    if (reason)
        recovery_failure_latched_ = *reason;
    enter_(State::kIdle);
    clear_outputs_();
}

void WheelLegController::enter_(State next, bool force_reset) {
    if (state_ == next && !force_reset)
        return;
    if (next == State::kIdle && recovery_.phase() == RecoveryPhase::kFailed)
        recovery_failure_latched_ = recovery_.failure();
    state_ = next;
    reset_heading_hold_();
    prepare_stable_since_.reset();
    if (next != State::kRl) {
        clear_outputs_();
        takeover_targets_.setZero();
        takeover_blend_fraction_ = 0.0;
        *pd_time_us_ = 0.0;
        std::ranges::fill(previous_action_, 0.0);
        std::ranges::fill(policy_action_, 0.0);
        *policy_enable_output_ = false;
        *policy_context_output_ = 0;
        policy_wait_started_.reset();
        accepted_policy_sequence_ = *policy_sequence_input_;
        for (auto& output : effective_action_outputs_)
            *output = 0.0;
        for (auto& output : action_outputs_)
            *output = 0.0;
        *policy_command_output_ = Eigen::Vector3d::Zero();
        *policy_height_output_ = 0.0;
        *policy_gyro_output_ = Eigen::Vector3d::Zero();
        *policy_gravity_output_ = Eigen::Vector3d::Zero();
        for (auto& output : model_angle_outputs_)
            *output = 0.0;
        for (auto& output : model_velocity_outputs_)
            *output = 0.0;
    }
    if (next == State::kIdle || next == State::kInit) {
        *enable_request_ = false;
        recovery_.reset();
        if (joint_reference_recovery_)
            joint_reference_recovery_->reset(q_, false);
        recovery_peak_budget_.reset();
        recovery_interval_.reset();
        recovery_actuation_interval_.reset();
        schedule_.last_pd_tick.reset();
        schedule_.last_recovery_tick.reset();
        motor_control_started_ = false;
        motor_feedback_initialized_ = false;
        recovery_sensor_guard_.reset();
        recovery_sensor_status_ = {};
        recovery_command_ = {};
        last_recovery_feedback_ = {};
        joint_reference_feedback_ = {};
        recovery_tuck_state_ = {};
        if (recovery_tuck_)
            recovery_tuck_->reset();
        if (recovery_observer_)
            recovery_observer_->reset();
        if (support_observer_)
            support_observer_->reset();
        recovery_started_ = false;
        recovery_upright_seconds_ = 0.0;
        policy_targets_valid_ = false;
        policy_targets_.setZero();
        accepted_policy_submitted_at_.reset();
        jump_was_requested_ = false;
        vx_reference_ = yaw_reference_ = 0.0;
        height_reference_ = height_from_ = height_target_ = DeployedPolicyContract::kNominalHeight;
        height_start_ = *timestamp_;
    } else if (next == State::kPrepare) {
        recovery_tuck_state_ = {};
        if (recovery_tuck_)
            recovery_tuck_->reset();
        recovery_.reset();
        if (joint_reference_recovery_)
            joint_reference_recovery_->reset(q_, false);
        if (support_observer_)
            support_observer_->reset();
        recovery_started_ = false;
        recovery_upright_seconds_ = 0.0;
        motor_feedback_initialized_ = false;
        recovery_sensor_guard_.reset();
        recovery_command_ = {};
        targets_ = q_;
        policy_targets_valid_ = false;
        schedule_.last_policy_tick = std::numeric_limits<std::size_t>::max();
        enable_wait_start_ = Clock::now();
    } else if (next == State::kRl) {
        // Freeze the last preparation pose, not its old torque. The optional
        // V6 takeover recomputes this PD at the configured feedback cadence.
        if (!recovery_started_ && policy_profile_.name == kV6PolicyProfile.name) {
            takeover_targets_ = targets_;
            takeover_targets_.tail<2>().setZero();
        }
        if (!recovery_started_)
            schedule_.last_policy_tick = std::numeric_limits<std::size_t>::max();
        else
            recovery_upright_seconds_ = 0.0;
        rl_start_ = *timestamp_;
        recovery_rl_start_ = recovery_update_time_;
        jump_was_requested_ = false;
    }
}

void WheelLegController::update_state_output_() {
    *recovery_sensors_valid_output_ = (state_ == State::kPrepare || state_ == State::kRl)
                                   && recovery_sensor_status_.valid && feedback_valid_
                                   && !fault_latched_;
    *recovery_sensor_issue_output_ = std::to_underlying(recovery_sensor_status_.issue);
    *recovery_sensor_mask_output_ = recovery_sensor_status_.invalid_mask;
    *recovery_motor_age_output_ = recovery_sensor_status_.maximum_motor_age_ms;
    *recovery_imu_age_output_ = recovery_sensor_status_.imu_age_ms;
    *recovery_acceleration_age_output_ = recovery_sensor_status_.acceleration_age_ms;
    const bool native = policy_profile_.name == kV6PolicyProfile.name && joint_reference_recovery_;
    *recovery_native_phase_output_ =
        native && recovery_started_ ? std::to_underlying(joint_reference_recovery_->command().phase)
        : native && joint_reference_failure_latched_ != 0
            ? std::to_underlying(JointReferenceRecoveryPhase::kFailed)
            : -1;
    *recovery_native_route_output_ =
        native && recovery_started_ ? std::to_underlying(joint_reference_recovery_->command().route)
                                    : -1;
    *recovery_native_failure_output_ = joint_reference_failure_latched_;
    const bool tuck_valid = recovery_started_ && recovery_tuck_state_.valid;
    *recovery_tuck_ready_output_ = tuck_valid && recovery_tuck_state_.ready;
    for (int side = 0; side < 2; ++side) {
        *recovery_tuck_knee_outputs_[side] = tuck_valid ? recovery_tuck_state_.knee_deg[side]
                                                        : std::numeric_limits<double>::quiet_NaN();
        *recovery_tuck_sagittal_outputs_[side] = tuck_valid
                                                   ? recovery_tuck_state_.sagittal_error_m[side]
                                                   : std::numeric_limits<double>::quiet_NaN();
    }
    *recovery_native_motion_released_output_ =
        native && recovery_started_ && joint_reference_recovery_->command().motion_released;
    *recovery_contact_output_ =
        recovery_started_
        && (native ? joint_reference_feedback_.support : last_recovery_feedback_.contact_candidate);
    *recovery_height_output_ = !recovery_started_ ? 0.0
                             : native             ? joint_reference_feedback_.estimated_height
                             : last_recovery_feedback_.geometry_valid
                                 ? last_recovery_feedback_.height_if_grounded
                                 : 0.0;
    *recovery_blend_output_ = !recovery_started_   ? 0.0
                            : native               ? joint_reference_recovery_->command().blend
                            : state_ == State::kRl ? 1.0
                                                   : recovery_command_.blend;
    if ((state_ == State::kPrepare || state_ == State::kRl) && feedback_valid_)
        publish_model_feedback_();
    for (std::size_t i = 0; i < previous_action_.size(); ++i)
        *effective_action_outputs_[i] = previous_action_[i];
    *takeover_blend_output_ = takeover_blend_fraction_;
    *state_output_ = std::to_underlying(state_);
    *recovery_phase_output_ = std::to_underlying(recovery_phase_());
    *recovery_failure_output_ = std::to_underlying(recovery_failure_latched_);
    *recovery_support_output_ = recovery_started_
                             && (native ? support_observer_->support_confirmed()
                                        : last_recovery_feedback_.support_confirmed);
    *recovery_geometry_output_ =
        recovery_started_
        && (native ? support_observer_->geometry_valid() : last_recovery_feedback_.geometry_valid);
    *recovery_motion_hold_output_ = recovery_motion_hold_();
}

bool WheelLegController::evaluate_policy_(std::size_t tick, bool recovering) {
    const bool native = policy_profile_.name == kV6PolicyProfile.name;
    const bool shadow = recovering && (native || recovery_command_.phase != RecoveryPhase::kBlend);
    const auto now = Clock::now();
    if (!policy_wait_started_)
        policy_wait_started_ = now;
    if (!*policy_healthy_input_)
        return false;
    if (*policy_valid_input_ > 0.5 && *policy_sequence_input_ > accepted_policy_sequence_) {
        std::array<double, 6> targets;
        for (std::size_t i = 0; i < targets.size(); ++i)
            targets[i] = *policy_target_inputs_[i];
        const bool result_shadow = *policy_context_input_ != 0;
        const auto processed = process_action_(targets, result_shadow);
        if (!processed) {
            node::error("Policy action rejected: {}", processed.error());
            return false;
        }
        accepted_policy_sequence_ = *policy_sequence_input_;
        accepted_policy_submitted_at_ =
            Clock::time_point{std::chrono::duration_cast<Clock::duration>(
                std::chrono::nanoseconds{*policy_submitted_ns_input_})};
        for (std::size_t i = 0; i < action_outputs_.size(); ++i)
            *action_outputs_[i] = native ? policy_action_[i] : previous_action_[i];
    }
    if (schedule_.last_policy_tick == std::numeric_limits<std::size_t>::max()
        || tick - schedule_.last_policy_tick >= schedule_.policy_divisor) {
        if (!assemble_observation_(recovering))
            return false;
        schedule_.last_policy_tick = tick;
    }
    *policy_context_output_ = shadow ? 1 : 0;
    *policy_enable_output_ = true;
    if (!policy_targets_valid_)
        return now - *policy_wait_started_ <= std::chrono::duration<double>{inference_timeout_};
    return policy_action_fresh_();
}

bool WheelLegController::policy_action_fresh_() const {
    return !policy_targets_valid_
        || (accepted_policy_submitted_at_
            && Clock::now() - *accepted_policy_submitted_at_
                   <= std::chrono::duration<double>{inference_timeout_});
}

void WheelLegController::update() {
    recovery_update_time_ = Clock::now();
    *policy_enable_output_ = false;
    if (!schedule_.ready) {
        const double rate = *update_rate_;
        if (!almost_integer(rate / inference_frequency_) || !almost_integer(rate / pd_frequency_)
            || !almost_integer(rate / recovery_frequency_))
            throw std::runtime_error(
                "RMCS update_rate must be an integer multiple of PD, recovery and policy "
                "frequencies");
        schedule_.policy_divisor =
            static_cast<std::size_t>(std::llround(rate / inference_frequency_));
        schedule_.pd_divisor = static_cast<std::size_t>(std::llround(rate / pd_frequency_));
        schedule_.recovery_divisor =
            static_cast<std::size_t>(std::llround(rate / recovery_frequency_));
        schedule_.ready = true;
    }
    const int requested = *state_command_;
    const bool reset = last_reset_count_ != *reset_count_;
    last_reset_count_ = *reset_count_;
    if (reset) {
        fault_latched_ = false;
        recovery_failure_latched_ = RecoveryFailure::kNone;
        joint_reference_failure_latched_ = 0;
        enter_(State::kIdle, true);
    }
    if (requested == 0) {
        if (recovery_started_)
            recovery_failure_latched_ = RecoveryFailure::kCancelled;
        enter_(State::kInit);
        clear_outputs_();
        update_state_output_();
        return;
    }
    // Auto entry is only for simulation / startup; a manual reset must never
    // re-arm itself merely because auto_enter_rl was configured.
    const bool automatic = requested == 1 && auto_enter_rl_ && *reset_count_ == 0;
    if (requested == 1 && !automatic) {
        if (recovery_started_)
            recovery_failure_latched_ = RecoveryFailure::kCancelled;
        enter_(State::kIdle);
        clear_outputs_();
        update_state_output_();
        return;
    }
    const bool unsupported_command = !accepts_motion_command(
        *jump_command_, *height_command_, *chassis_mode_, *velocity_command_, policy_profile_);
    if (unsupported_command)
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Requested motion exceeds the active policy capability profile");
    if ((requested != 2 && requested != 3 && !automatic) || unsupported_command || fault_latched_
        || !policy_ready_ || !calibration_ready_ || !soft_limits_ready_ || !imu_alignment_ready_
        || (policy_profile_.name == kV6PolicyProfile.name && !joint_reference_profile_ready_)
        || (recovery_enabled_
            && (!recovery_profile_ready_
                || (policy_profile_.name == kV6PolicyProfile.name
                    && (!joint_reference_recovery_ || !support_observer_))))
        || !read_model_state_()) {
        if (requested >= 2 && calibration_ready_ && soft_limits_ready_ && policy_ready_
            && imu_alignment_ready_) {
            log_fault_(
                !feedback_valid_ ? std::optional{RecoveryFailure::kInvalidFeedback} : std::nullopt);
            fault_latched_ = true;
        }
        if (recovery_started_ && !feedback_valid_)
            recovery_failure_latched_ = RecoveryFailure::kInvalidFeedback;
        enter_(State::kIdle);
        clear_outputs_();
        update_state_output_();
        return;
    }

    if (recovery_enabled_ && requested == 3 && state_ != State::kRl
        && (*chassis_mode_ != rmcs_msgs::ChassisMode::AUTO
            || (!recovery_started_
                && !neutral_recovery_request(true, velocity_command_->vector)))) {
        enter_(State::kIdle);
        fault_latched_ = true;
        recovery_failure_latched_ = RecoveryFailure::kUnsafeRequest;
        update_state_output_();
        return;
    }

    const std::size_t tick = *update_count_;
    const bool pd_due =
        !schedule_.last_pd_tick || tick - *schedule_.last_pd_tick >= schedule_.pd_divisor;
    const bool recovery_due =
        recovery_enabled_
        && (!schedule_.last_recovery_tick
            || tick - *schedule_.last_recovery_tick >= schedule_.recovery_divisor);
    if (recovery_due) {
        const auto elapsed = recovery_interval_.sample(
            recovery_update_time_, std::chrono::duration_cast<Clock::duration>(
                                       std::chrono::duration<double>{1.0 / recovery_frequency_}));
        if (!elapsed) {
            latch_fault_(RecoveryFailure::kInvalidFeedback);
            update_state_output_();
            return;
        }
        recovery_dt_ = *elapsed;
        schedule_.last_recovery_tick = tick;
    }
    if ((state_ == State::kRl && recovery_started_)
        || (policy_profile_.name == kV6PolicyProfile.name
            && !(recovery_enabled_ && requested == 3 && state_ != State::kRl))) {
        const Eigen::Quaterniond world_base = world_base_orientation_();
        const Eigen::Vector3d gravity = world_base.conjugate() * -Eigen::Vector3d::UnitZ();
        if (-gravity.z() < std::cos(45.0 * std::numbers::pi / 180.0)) {
            latch_fault_(RecoveryFailure::kLostUpright);
            update_state_output_();
            return;
        }
        if (recovery_due && recovery_started_) {
            if (policy_profile_.name == kV6PolicyProfile.name) {
                if (!advance_recovery_()) {
                    latch_fault_(RecoveryFailure::kInvalidFeedback);
                    update_state_output_();
                    return;
                }
            } else {
                const auto feedback = observe_recovery_();
                if (!feedback || !feedback->geometry_valid
                    || !feedback->spring_compensation_valid) {
                    latch_fault_(RecoveryFailure::kInvalidFeedback);
                    update_state_output_();
                    return;
                }
                recovery_upright_seconds_ =
                    recovery_upright_for_motion(*feedback)
                        ? std::min(1.0, recovery_upright_seconds_ + recovery_dt_)
                        : 0.0;
            }
        }
    }
    if (!policy_action_fresh_()) {
        node::error("Last accepted RL action exceeded inference_timeout");
        fault_latched_ = true;
    }
    if (fault_latched_) {
        enter_(State::kIdle);
        clear_outputs_();
        update_state_output_();
        return;
    }
    if (requested == 2 || (state_ != State::kPrepare && state_ != State::kRl))
        enter_(State::kPrepare);
    if (requested == 2 && recovery_started_) {
        latch_fault_(RecoveryFailure::kCancelled);
        update_state_output_();
        return;
    }
    *enable_request_ = true;
    if (state_ == State::kRl && (!dm_control_ready_.ready() || !*dm_control_ready_)) {
        latch_fault_(RecoveryFailure::kInvalidFeedback);
        update_state_output_();
        return;
    }
    if (state_ == State::kPrepare) {
        if (!dm_control_ready_.ready() || !*dm_control_ready_) {
            clear_outputs_();
            if (motor_control_started_ || recovery_started_) {
                latch_fault_(RecoveryFailure::kInvalidFeedback);
                update_state_output_();
                return;
            }
            if (requested == 3 && recovery_due && recovery_observer_ && !observe_recovery_()) {
                // Contact evidence must not survive a gap in IMU feedback.
                recovery_observer_->reset();
                last_recovery_feedback_ = {};
            }
            // Waiting for the first MIT does not change the recovery cadence.
            if (pd_due)
                schedule_.last_pd_tick = tick;
            if (recovery_update_time_ - enable_wait_start_ > std::chrono::seconds{1}) {
                latch_fault_();
            }
            update_state_output_();
            return;
        }
    }
    if (requested == 3 || automatic) {
        const bool recovering = state_ == State::kPrepare && recovery_enabled_ && requested == 3;
        if (!evaluate_policy_(tick, recovering)) {
            latch_fault_();
            update_state_output_();
            return;
        }
        if (!policy_targets_valid_) {
            clear_outputs_();
            update_state_output_();
            return;
        }
    }
    if (state_ == State::kPrepare) {
        if (recovery_enabled_ && requested == 3) {
            if (recovery_due) {
                if (!advance_recovery_()) {
                    latch_fault_();
                    update_state_output_();
                    return;
                }
            }
        } else {
            const bool ready = update_prepare_();
            if (ready && (requested == 3 || automatic))
                enter_(State::kRl);
        }
    }
    // Recovery and PD share a tick when their configured frequencies match.
    if (recovery_due && recovery_started_ && policy_profile_.name == kV6PolicyProfile.name
        && !advance_joint_reference_recovery_()) {
        latch_fault_();
        update_state_output_();
        return;
    }
    if (pd_due) {
        const auto pd_start = Clock::now();
        motor_control_started_ = true;
        compute_motor_torques_();
        if (state_ == State::kPrepare || state_ == State::kRl) {
            *pd_time_us_ =
                std::chrono::duration<double, std::micro>{Clock::now() - pd_start}.count();
            schedule_.last_pd_tick = tick;
        }
    }
    // A lower configured PD rate holds its last effort between actuation ticks.
    // With pd_frequency=1000 and a 1 kHz executor, recompute on every tick.
    // clear_outputs_ above is only for states that cannot execute the PD.
    update_state_output_();
}

} // namespace rmcs_core::controller::chassis

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::WheelLegController, rmcs_executor::Component)
