#include "wheel_leg_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

#include "gas_spring_model.hpp"

namespace rmcs_core::controller::chassis {
namespace {
bool almost_integer(double value) {
    return std::isfinite(value) && value >= 1.0 && std::abs(value - std::round(value)) < 1e-6;
}
} // namespace

WheelLegController::~WheelLegController() = default;

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
    const double wheel_speed_limit = std::min(
        {-action_clip_min_[4], action_clip_max_[4], -action_clip_min_[5], action_clip_max_[5]});
    if (wheel_speed_limit <= 0.0)
        throw std::runtime_error("Wheel action bounds must allow both drive directions");
    validate_leg_target_model_spec(
        leg_target_winding_, *policy_model_spec_input_, "Wheel-leg bridge model");
    if (gas_spring_compensation_enabled_
        && (policy_profile_.name != kV6PolicyProfile.name
            || *policy_model_spec_input_ != gas_spring_model::kModelSpecSha256))
        throw std::runtime_error(
            "Gas-spring compensation requires the calibrated V7 model specification");
    policy_ready_ = true;
    // Executor::start sets /predefined/update_rate AFTER before_updating().
    last_reset_count_ = *reset_count_;
    use_motor_position_pd_ = motor_position_pd_.ready() && *motor_position_pd_;
    if (unified_full_rl_) {
        if (policy_profile_.name != kV6PolicyProfile.name || heading_hold_enabled_
            || use_motor_position_pd_ || inference_frequency_ != 200.0 || pd_frequency_ != 1000.0
            || leg_target_winding_ != LegTargetWinding::kPairedBranch107
            || !policy_observation_contract_input_.ready()
            || *policy_observation_contract_input_ != kStandingObservationContractSha256
            || !policy_frame_path_input_.ready()
            || *policy_frame_path_input_ != "/wheel_leg/rl/input/frame"
            || !policy_frame_sequence_path_input_.ready()
            || *policy_frame_sequence_path_input_ != "/wheel_leg/rl/input/frame_sequence")
            throw std::runtime_error(
                "Unified full RL requires its exact 200Hz observation/frame/host-torque contract");
        for (std::size_t i = 0; i < 6; ++i) {
            const double scale = i < 4 ? .25 : 10.0;
            const double bound = i < 4 ? 4.0 : 90.0;
            if (action_scales_[i] != scale || action_clip_min_[i] != -bound
                || action_clip_max_[i] != bound)
                throw std::runtime_error("Unified full RL requires raw leg16/wheel9 action bounds");
        }
    } else if (
        policy_observation_contract_input_.ready()
        && !policy_observation_contract_input_->empty()) {
        throw std::runtime_error("This wheel-leg observation contract requires unified_full_rl");
    }
    if (use_motor_position_pd_) {
        if (policy_profile_.name != kV6PolicyProfile.name || !motor_position_kp_.ready()
            || !motor_position_kd_.ready() || *motor_position_kp_ != policy_profile_.leg_kp
            || *motor_position_kd_ != policy_profile_.leg_kd)
            throw std::runtime_error("Motor position PD requires native gains");
        if ((leg_jacobian_.array().abs() - Eigen::Matrix4d::Identity().array()).abs().maxCoeff()
            > 1e-9)
            throw std::runtime_error("Motor position PD requires independent signed 1:1 axes");
    }
    RCLCPP_INFO(
        get_logger(), "Joint PD backend: %s, Kp %.1f, Kd %.1f",
        use_motor_position_pd_ && !gas_spring_compensation_enabled_ ? "DM internal MIT"
                                                                    : "host torque",
        policy_profile_.leg_kp, policy_profile_.leg_kd);
    RCLCPP_INFO(
        get_logger(), "Gas-spring compensation %s: gain %.2f, full soft-limit coverage, model %s",
        gas_spring_compensation_enabled_ ? "enabled" : "disabled", gas_spring_compensation_gain_,
        gas_spring_model::kModelSha256.data());
    if (!calibration_ready_ || !soft_limits_ready_ || !imu_alignment_ready_ || !policy_ready_)
        RCLCPP_WARN(
            get_logger(),
            "RL disarmed: calibration_ready=%d, soft_limits_ready=%d, imu_alignment_ready=%d, "
            "policy_ready=%d",
            calibration_ready_, soft_limits_ready_, imu_alignment_ready_, policy_ready_);
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
    *joint_torque_override_ = false;
    for (auto& output : gas_compensation_torque_outputs_)
        *output = 0.0;
    for (int side = 0; side < 2; ++side) {
        *gas_beta_outputs_[side] = std::numeric_limits<double>::quiet_NaN();
        *gas_force_outputs_[side] = 0.0;
        *gas_weight_outputs_[side] = 0.0;
        *gas_extrapolated_outputs_[side] = 0.0;
    }
}

void WheelLegController::log_fault_(const char* reason, std::source_location location) const {
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
        "RL stopped at %s:%u: %s, state=%d feedback_fresh=%d feedback_valid=%d "
        "sensor_valid=%d sensor_issue=%d mask=0x%02x age_ms[motor,imu,accel]=[%.3f,%.3f,%.3f] "
        "hinge=[%.4f,%.4f] dq_max[leg,wheel]=[%.3f,%.3f]",
        location.file_name(), location.line(), reason, static_cast<int>(state_), *feedback_fresh_,
        feedback_valid_, sensor_status_.valid, static_cast<int>(sensor_status_.issue),
        static_cast<unsigned>(sensor_status_.invalid_mask), sensor_status_.maximum_motor_age_ms,
        sensor_status_.imu_age_ms, sensor_status_.acceleration_age_ms, hinge[0], hinge[1],
        dq_.head<4>().cwiseAbs().maxCoeff(), dq_.tail<2>().cwiseAbs().maxCoeff());
}

void WheelLegController::latch_fault_(const char* reason, std::source_location location) {
    log_fault_(reason, location);
    fault_latched_ = true;
    enter_(State::kIdle);
    clear_outputs_();
}

void WheelLegController::enter_(State next, bool force_reset) {
    if (state_ == next && !force_reset)
        return;
    state_ = next;
    if (unified_full_rl_) {
        unified_observation_.reset(next_policy_frame_sequence_);
        if (next != State::kRl) {
            *policy_frame_sequence_output_ = 0;
            std::ranges::fill(*policy_frame_output_, 0.0);
            applied_policy_action_.fill(0.0);
        }
    }
    reset_heading_hold_();
    if (next == State::kRl)
        return;

    clear_outputs_();
    *pd_time_us_ = 0.0;
    *policy_enable_output_ = false;
    std::ranges::fill(previous_action_, 0.0);
    policy_wait_started_.reset();
    accepted_policy_sequence_ = *policy_sequence_input_;
    accepted_policy_submitted_at_.reset();
    policy_targets_valid_ = false;
    policy_targets_.setZero();
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
    schedule_.last_policy_tick = std::numeric_limits<std::size_t>::max();
    schedule_.last_pd_tick.reset();
    leg_feedback_initialized_ = false;
    sensor_status_ = {};
    vx_reference_ = yaw_reference_ = 0.0;
    height_reference_ = height_from_ = height_target_ = DeployedPolicyContract::kNominalHeight;
    height_start_ = *timestamp_;
    *enable_request_ = next == State::kPrepare;
}

void WheelLegController::update_state_output_() {
    const bool active = state_ == State::kPrepare || state_ == State::kRl;
    *sensors_valid_output_ = active && sensor_status_.valid && feedback_valid_ && !fault_latched_;
    *sensor_issue_output_ = std::to_underlying(sensor_status_.issue);
    *sensor_mask_output_ = sensor_status_.invalid_mask;
    *motor_age_output_ = sensor_status_.maximum_motor_age_ms;
    *imu_age_output_ = sensor_status_.imu_age_ms;
    *acceleration_age_output_ = sensor_status_.acceleration_age_ms;
    if (active && feedback_valid_)
        publish_model_feedback_();
    for (std::size_t i = 0; i < previous_action_.size(); ++i)
        *effective_action_outputs_[i] =
            unified_full_rl_ ? applied_policy_action_[i] : previous_action_[i];
    *state_output_ = std::to_underlying(state_);
    *fault_output_ = fault_latched_;
}

bool WheelLegController::evaluate_policy_(std::size_t tick) {
    const auto now = Clock::now();
    if (!policy_wait_started_)
        policy_wait_started_ = now;
    if (!(*policy_healthy_input_ > 0.5))
        return false;
    if (*policy_valid_input_ > 0.5 && *policy_sequence_input_ > accepted_policy_sequence_) {
        std::array<double, 6> targets;
        for (std::size_t i = 0; i < targets.size(); ++i)
            targets[i] = *policy_target_inputs_[i];
        const auto processed = process_action_(targets);
        if (!processed) {
            node::error("Policy action rejected: {}", processed.error());
            return false;
        }
        accepted_policy_sequence_ = *policy_sequence_input_;
        accepted_policy_submitted_at_ =
            Clock::time_point{std::chrono::duration_cast<Clock::duration>(
                std::chrono::nanoseconds{*policy_submitted_ns_input_})};
        for (std::size_t i = 0; i < action_outputs_.size(); ++i)
            *action_outputs_[i] = previous_action_[i];
    }
    if (schedule_.last_policy_tick == std::numeric_limits<std::size_t>::max()
        || tick - schedule_.last_policy_tick >= schedule_.policy_divisor) {
        if (!assemble_observation_())
            return false;
        if (unified_full_rl_ && !publish_unified_observation_())
            return false;
        schedule_.last_policy_tick = tick;
    }
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
    *policy_enable_output_ = false;
    if (!schedule_.ready) {
        const double rate = *update_rate_;
        if (unified_full_rl_ && rate != 1000.0)
            throw std::runtime_error("Unified full RL requires a 1000Hz executor");
        if (!almost_integer(rate / inference_frequency_) || !almost_integer(rate / pd_frequency_))
            throw std::runtime_error(
                "RMCS update_rate must be an integer multiple of PD and policy frequencies");
        schedule_.policy_divisor =
            static_cast<std::size_t>(std::llround(rate / inference_frequency_));
        schedule_.pd_divisor = static_cast<std::size_t>(std::llround(rate / pd_frequency_));
        schedule_.ready = true;
    }
    const int requested = *state_command_;
    const bool reset = last_reset_count_ != *reset_count_;
    last_reset_count_ = *reset_count_;
    if (reset) {
        fault_latched_ = false;
        enter_(State::kIdle, true);
        update_state_output_();
        return;
    }
    if (requested == 0) {
        enter_(State::kInit);
        clear_outputs_();
        update_state_output_();
        return;
    }
    // Auto entry is only for simulation / startup. A manual reset never re-arms it.
    const bool automatic = requested == 1 && auto_enter_rl_ && *reset_count_ == 0;
    if (requested == 1 && !automatic) {
        enter_(State::kIdle);
        clear_outputs_();
        update_state_output_();
        return;
    }
    if (fault_latched_) {
        clear_outputs_();
        update_state_output_();
        return;
    }
    if ((requested != 2 && requested != 3 && !automatic) || !policy_ready_ || !calibration_ready_
        || !soft_limits_ready_ || !imu_alignment_ready_ || !read_model_state_()) {
        latch_fault_("invalid command, calibration or feedback");
        update_state_output_();
        return;
    }
    if (requested == 2 || (state_ != State::kPrepare && state_ != State::kRl))
        enter_(State::kPrepare);
    *enable_request_ = true;
    if (!dm_control_ready_.ready() || !*dm_control_ready_) {
        // Keep requesting the normal clear-error/enable handshake. A drive
        // status transition does not require the operator to reset RL.
        enter_(State::kPrepare);
        clear_outputs_();
        update_state_output_();
        return;
    }
    if (requested == 2) {
        clear_outputs_();
        update_state_output_();
        return;
    }
    const std::size_t tick = *update_count_;
    if (!evaluate_policy_(tick)) {
        latch_fault_("policy action invalid or expired");
        update_state_output_();
        return;
    }
    if (!policy_targets_valid_) {
        clear_outputs_();
        update_state_output_();
        return;
    }
    // The policy owns every pose and all six axes from its first valid action.
    enter_(State::kRl);
    const bool pd_due =
        !schedule_.last_pd_tick || tick - *schedule_.last_pd_tick >= schedule_.pd_divisor;
    if (pd_due) {
        const auto pd_start = Clock::now();
        compute_motor_torques_();
        if (state_ == State::kRl) {
            // Only actions that reached PD may enter the next observation.
            if (unified_full_rl_)
                applied_policy_action_ = previous_action_;
            *pd_time_us_ =
                std::chrono::duration<double, std::micro>{Clock::now() - pd_start}.count();
            schedule_.last_pd_tick = tick;
        }
    }
    update_state_output_();
}

} // namespace rmcs_core::controller::chassis

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::WheelLegController, rmcs_executor::Component)
