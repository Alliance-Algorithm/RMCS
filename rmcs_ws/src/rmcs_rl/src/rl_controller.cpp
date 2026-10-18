#include "rl_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

namespace rmcs::rl {

namespace {
bool almost_integer(double value) {
    return std::isfinite(value) && value >= 1.0 && std::abs(value - std::round(value)) < 1e-6;
}

} // namespace

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

void RlController::before_updating() {
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

void RlController::clear_outputs_() {
    for (auto& output : torque_outputs_)
        *output = 0.0;
    for (auto& output : angle_outputs_)
        *output = std::numeric_limits<double>::quiet_NaN();
    *joint_command_ns_ = 0;
}

void RlController::log_fault_(
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
        *height_command_, velocity_command_->vector.x(), observation_[ObservationLayout::kCommand],
        policy_targets_[4], policy_targets_[5], dq_[4], dq_[5], *torque_feedback_inputs_[0],
        *torque_feedback_inputs_[1], *torque_feedback_inputs_[2], *torque_feedback_inputs_[3]);
}

void RlController::latch_fault_(
    std::optional<RecoveryFailure> reason, std::source_location location) {
    log_fault_(reason, location);
    fault_latched_ = true;
    if (reason)
        recovery_failure_latched_ = *reason;
    enter_(State::kIdle);
    clear_outputs_();
}

void RlController::enter_(State next) {
    if (state_ == next)
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
        *inference_time_us_ = 0.0;
        *pd_time_us_ = 0.0;
        previous_action_.fill(0);
        policy_action_.fill(0);
        for (auto& output : observation_outputs_)
            *output = 0.0;
        for (auto& output : action_outputs_)
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
        last_pd_tick_.reset();
        last_recovery_tick_.reset();
        motor_control_started_ = false;
        motor_feedback_initialized_ = false;
        recovery_sensor_guard_.reset();
        if (recovery_observer_)
            recovery_observer_->reset();
        if (support_observer_)
            support_observer_->reset();
        recovery_started_ = false;
        recovery_upright_seconds_ = 0.0;
        policy_targets_valid_ = false;
        jump_was_requested_ = false;
        vx_reference_ = yaw_reference_ = 0.0;
        height_reference_ = height_from_ = height_target_ = DeployedPolicyContract::kNominalHeight;
        height_start_ = *timestamp_;
    } else if (next == State::kPrepare) {
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
        last_policy_tick_ = std::numeric_limits<std::size_t>::max();
        enable_wait_start_ = Clock::now();
    } else if (next == State::kRl) {
        // Freeze the last preparation pose, not its old torque. The optional
        // V6 takeover recomputes this PD at the configured feedback cadence.
        if (!recovery_started_ && policy_profile_.name == kV6PolicyProfile.name) {
            takeover_targets_ = targets_;
            takeover_targets_.tail<2>().setZero();
        }
        if (!recovery_started_)
            last_policy_tick_ = std::numeric_limits<std::size_t>::max();
        else
            recovery_upright_seconds_ = 0.0;
        rl_start_ = *timestamp_;
        recovery_rl_start_ = recovery_update_time_;
        jump_was_requested_ = false;
    }
}

void RlController::update_state_output_() {
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

bool RlController::evaluate_policy_(std::size_t tick, bool recovering) {
    const bool native = policy_profile_.name == kV6PolicyProfile.name;
    const bool shadow = recovering && (native || recovery_command_.phase != RecoveryPhase::kBlend);
    if (!assemble_observation_(recovering)) {
        return false;
    }
    const auto inference_start = Clock::now();
    const auto inference =
        policy_->run(observation_).and_then([this, shadow](const PolicyAction& raw) {
            return process_action_(raw, shadow);
        });
    *inference_time_us_ =
        std::chrono::duration<double, std::micro>{Clock::now() - inference_start}.count();
    if (!inference) {
        node::error("ONNX inference failed: {}", inference.error());
        return false;
    }
    last_policy_tick_ = tick;
    for (std::size_t i = 0; i < observation_outputs_.size(); ++i)
        *observation_outputs_[i] = observation_[i];
    if (!shadow || native)
        for (std::size_t i = 0; i < action_outputs_.size(); ++i)
            *action_outputs_[i] = native ? policy_action_[i] : previous_action_[i];
    return true;
}

void RlController::update() {
    recovery_update_time_ = Clock::now();
    if (!timing_ready_) {
        const double rate = *update_rate_;
        if (!almost_integer(rate / inference_frequency_) || !almost_integer(rate / pd_frequency_)
            || !almost_integer(rate / recovery_frequency_))
            throw std::runtime_error(
                "RMCS update_rate must be an integer multiple of PD, recovery and policy "
                "frequencies");
        policy_divisor_ = static_cast<std::size_t>(std::llround(rate / inference_frequency_));
        pd_divisor_ = static_cast<std::size_t>(std::llround(rate / pd_frequency_));
        recovery_divisor_ = static_cast<std::size_t>(std::llround(rate / recovery_frequency_));
        timing_ready_ = true;
    }
    const int requested = *state_command_;
    const bool reset = last_reset_count_ != *reset_count_;
    last_reset_count_ = *reset_count_;
    if (reset) {
        fault_latched_ = false;
        recovery_failure_latched_ = RecoveryFailure::kNone;
        joint_reference_failure_latched_ = 0;
        enter_(State::kIdle);
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
    const bool pd_due = !last_pd_tick_ || tick - *last_pd_tick_ >= pd_divisor_;
    const bool recovery_due =
        recovery_enabled_
        && (!last_recovery_tick_ || tick - *last_recovery_tick_ >= recovery_divisor_);
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
        last_recovery_tick_ = tick;
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
                last_pd_tick_ = tick;
            if (recovery_update_time_ - enable_wait_start_ > std::chrono::seconds{1}) {
                latch_fault_();
            }
            update_state_output_();
            return;
        }
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
    const bool recovering = state_ == State::kPrepare && recovery_started_;
    if (state_ == State::kRl || recovering) {
        if (last_policy_tick_ == std::numeric_limits<std::size_t>::max()
            || tick - last_policy_tick_ >= policy_divisor_) {
            if (!evaluate_policy_(tick, recovering)) {
                latch_fault_();
                update_state_output_();
                return;
            }
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
            last_pd_tick_ = tick;
        }
    }
    // A lower configured PD rate holds its last effort between actuation ticks.
    // With pd_frequency=1000 and a 1 kHz executor, recompute on every tick.
    // clear_outputs_ above is only for states that cannot execute the PD.
    update_state_output_();
}

} // namespace rmcs::rl

PLUGINLIB_EXPORT_CLASS(rmcs::rl::RlController, rmcs_executor::Component)
