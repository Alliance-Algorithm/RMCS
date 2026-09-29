#include "rl_controller.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace rmcs::rl {

std::expected<void, std::string>
    RlController::process_action_(const PolicyAction& raw, bool shadow_recovery) {
    PolicyAction clipped;
    for (int i = 0; i < 6; ++i) {
        if (!std::isfinite(raw[i]))
            return std::unexpected{std::string{"ONNX returned a non-finite action"}};
        const float limit = i < 4 ? DeployedPolicyContract::kLegActionLimit
                                  : DeployedPolicyContract::kWheelActionLimit;
        clipped[i] = std::clamp(raw[i], -limit, limit);
    }
    Vector6 targets = policy_targets_;
    for (int i = 0; i < 4; ++i) {
        const double desired = nominal_[i] + DeployedPolicyContract::kLegActionScale * clipped[i];
        targets[i] = q_[i] + std::remainder(desired - q_[i], 2 * std::numbers::pi);
    }
    if (recovery_started_ && recovery_observer_) {
        Eigen::Vector4d legs = targets.head<4>();
        recovery_observer_->constrain_policy_goal(legs, 2.0);
        targets.head<4>() = legs;
    } else {
        for (int side = 0; side < 2; ++side) {
            const int hip = 2 * side, knee = hip + 1;
            // The ordinary RL path retains its calibrated hinge mapping.
            const double a = hinge_coeff_[hip], b = hinge_coeff_[knee];
            const double low =
                (hinge_min_[side] + hinge_margin_ - hinge_bias_[side] - a * targets[hip]) / b;
            const double high =
                (hinge_max_[side] - hinge_margin_ - hinge_bias_[side] - a * targets[hip]) / b;
            targets[knee] = std::clamp(targets[knee], std::min(low, high), std::max(low, high));
        }
    }
    // The last two actions command wheel speeds, not torques.
    targets[4] = DeployedPolicyContract::kWheelActionScale * clipped[4];
    targets[5] = DeployedPolicyContract::kWheelActionScale * clipped[5];
    if (!targets.allFinite())
        return std::unexpected{
            std::string{"Policy action or soft-limit mapping generated a non-finite target"}};
    if (!shadow_recovery)
        previous_action_ = clipped;
    policy_targets_ = targets;
    policy_targets_valid_ = true;
    return {};
}

void RlController::apply_soft_limits_(Eigen::Vector4d& tau) const {
    if (recovery_started_ && recovery_observer_) {
        for (int side = 0; side < 2; ++side) {
            const int hip = 2 * side, knee = hip + 1;
            const double inner = last_recovery_feedback_.inner_knee_deg[side];
            const double slope = last_recovery_feedback_.inner_knee_slope_deg_per_rad[side];
            const double a = -slope, b = slope;
            const double outward = a * tau[hip] + b * tau[knee];
            if ((inner >= 108.0 && outward > 0) || (inner <= 42.0 && outward < 0)) {
                const double multiplier = outward / (a * a + b * b);
                tau[hip] -= a * multiplier;
                tau[knee] -= b * multiplier;
            }
        }
        return;
    }
    for (int side = 0; side < 2; ++side) {
        const int hip = 2 * side, knee = hip + 1;
        const double a = hinge_coeff_[hip], b = hinge_coeff_[knee];
        const double relative = a * q_[hip] + b * q_[knee] + hinge_bias_[side];
        const double outward = a * tau[hip] + b * tau[knee];
        if ((relative >= hinge_max_[side] - hinge_margin_ && outward > 0)
            || (relative <= hinge_min_[side] + hinge_margin_ && outward < 0)) {
            // Project the commanded model effort onto the tangent of the soft
            // constraint. Both hip and knee drives contribute to the hinge.
            const double multiplier = outward / (a * a + b * b);
            tau[hip] -= a * multiplier;
            tau[knee] -= b * multiplier;
        }
    }
}

void RlController::compute_motor_torques_() {
    const bool scripted = state_ == State::kPrepare && recovery_started_;
    const bool blending = scripted && recovery_command_.phase == RecoveryPhase::kBlend;
    Eigen::Vector4d tau = Eigen::Vector4d::Zero();
    Eigen::Vector2d wheel_tau = Eigen::Vector2d::Zero();
    if (scripted) {
        tau = recovery_command_.torque.head<4>();
        wheel_tau = recovery_command_.torque.tail<2>();
    } else if (state_ == State::kPrepare) {
        for (int i = 0; i < 4; ++i)
            tau[i] =
                std::clamp(prepare_kp_ * (targets_[i] - q_[i]) - prepare_kd_ * dq_[i], -40.0, 40.0);
        wheel_tau = -0.2 * dq_.tail<2>();
    }
    if (blending || state_ == State::kRl) {
        if (!policy_targets_valid_) {
            latch_fault_();
            return;
        }
        Eigen::Vector4d policy_tau;
        for (int i = 0; i < 4; ++i)
            policy_tau[i] = std::clamp(
                DeployedPolicyContract::kLegKp * (policy_targets_[i] - q_[i])
                    - DeployedPolicyContract::kLegKd * dq_[i],
                -DeployedPolicyContract::kLegTorqueLimit, DeployedPolicyContract::kLegTorqueLimit);
        const Eigen::Vector2d policy_wheel =
            DeployedPolicyContract::kWheelKp * (policy_targets_.tail<2>() - dq_.tail<2>());
        if (blending) {
            tau = (1.0 - recovery_command_.blend) * tau + recovery_command_.blend * policy_tau;
            wheel_tau = (1.0 - recovery_command_.blend) * wheel_tau
                      + recovery_command_.blend * policy_wheel;
        } else {
            tau = policy_tau;
            wheel_tau = policy_wheel;
        }
    }
    // Enforce the relative hinge limit during PREPARE as well as during RL.
    // Stop on a measured violation beyond the guard band; physical stops are
    // not replaced by this software clamp.
    if (recovery_started_ && recovery_observer_) {
        if (!last_recovery_feedback_.geometry_valid) {
            latch_fault_(RecoveryFailure::kInvalidFeedback);
            return;
        }
    } else {
        for (int side = 0; side < 2; ++side) {
            const int hip = 2 * side;
            const double relative = hinge_coeff_[hip] * q_[hip]
                                  + hinge_coeff_[hip + 1] * q_[hip + 1] + hinge_bias_[side];
            if (!std::isfinite(relative) || relative < hinge_min_[side] - hinge_margin_
                || relative > hinge_max_[side] + hinge_margin_) {
                latch_fault_();
                return;
            }
        }
    }
    apply_soft_limits_(tau);
    // Conditional 24 V output-shaft bound, after the model-side blend.
    // A measured dynamometer curve must replace these provisional endpoints.
    if (recovery_enabled_) {
        // Inference may stall after update() sampled the recovery interval.
        // Reject that same output and measure peak exposure at actuation time.
        const auto now = Clock::now();
        const auto elapsed = recovery_actuation_interval_.sample(now);
        if (!recovery_interval_.fresh(now) || !elapsed) {
            latch_fault_(RecoveryFailure::kInvalidFeedback);
            return;
        }
        for (int i = 0; i < 4; ++i) {
            const double bound = conditional_dm_output_bound(
                dq_[i], recovery_dm_rated_output_rpm_, recovery_dm_rated_torque_nm_,
                recovery_dm_peak_torque_nm_);
            tau[i] = std::clamp(tau[i], -bound, bound);
        }
        recovery_peak_budget_.limit(tau, *elapsed);
    }
    const Eigen::Vector4d motor_tau = leg_jacobian_.transpose() * tau;
    if (!motor_tau.allFinite()) {
        latch_fault_();
        return;
    }
    for (int i = 0; i < 4; ++i)
        *torque_outputs_[i] =
            std::clamp(motor_tau[i], -*max_torque_inputs_[i], *max_torque_inputs_[i]);

    for (int i = 0; i < 2; ++i) {
        // DjiMotor's installed 15.8 ratio converts motor current to wheel-output
        // torque. The model wheel target is already a wheel-output speed.
        const double motor_tau_wheel = wheel_scale_[i] * wheel_tau[i];
        if (!std::isfinite(motor_tau_wheel)) {
            latch_fault_();
            return;
        }
        // DjiMotor provides the installed M3508's own current/ratio torque limit.
        *torque_outputs_[4 + i] =
            std::clamp(motor_tau_wheel, -*max_torque_inputs_[4 + i], *max_torque_inputs_[4 + i]);
    }
}

} // namespace rmcs::rl
