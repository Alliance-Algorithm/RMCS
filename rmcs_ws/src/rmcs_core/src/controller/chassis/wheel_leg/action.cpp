#include "wheel_leg_controller.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "gas_spring_compensation.hpp"
#include "leg_target_winding.hpp"

namespace rmcs_core::controller::chassis {

std::expected<void, std::string> WheelLegController::process_action_(std::span<const double> raw) {
    if (raw.size() != kMotorNames.size())
        return std::unexpected{std::string{"ONNX action size does not match YAML"}};
    Vector6 decoded;
    std::array<double, 6> clipped{};
    for (std::size_t i = 0; i < kMotorNames.size(); ++i) {
        if (!std::isfinite(raw[i]))
            return std::unexpected{std::string{"ONNX returned a non-finite action"}};
        const double target = std::clamp(raw[i], action_clip_min_[i], action_clip_max_[i]);
        decoded[i] = target;
        clipped[i] = target / action_scales_[i];
        if (!std::isfinite(clipped[i]))
            return std::unexpected{
                std::string{"YAML action scale cannot represent action history"}};
    }
    Vector6 targets = policy_targets_;
    const bool v6 = policy_profile_.name == kV6PolicyProfile.name;
    std::array<double, 4> desired{}, current{};
    for (int i = 0; i < 4; ++i) {
        desired[i] = nominal_[i] + decoded[i];
        current[i] = q_[i];
    }
    const auto leg_targets = wind_leg_targets(desired, current, leg_target_winding_);
    for (int i = 0; i < 4; ++i)
        targets[i] = leg_targets[i];
    if (!v6) {
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
    targets[4] = decoded[4];
    targets[5] = decoded[5];
    if (!targets.allFinite())
        return std::unexpected{
            std::string{"Policy action or soft-limit mapping generated a non-finite target"}};
    std::ranges::copy(clipped, previous_action_.begin());
    policy_targets_ = targets;
    policy_targets_valid_ = true;
    return {};
}

bool WheelLegController::apply_soft_limits_(Eigen::Vector4d& tau) const {
    bool at_limit = false;
    for (int side = 0; side < 2; ++side) {
        const int hip = 2 * side, knee = hip + 1;
        const double a = hinge_coeff_[hip], b = hinge_coeff_[knee];
        const double relative = a * q_[hip] + b * q_[knee] + hinge_bias_[side];
        const double outward = a * tau[hip] + b * tau[knee];
        const bool upper = relative >= hinge_max_[side] - hinge_margin_;
        const bool lower = relative <= hinge_min_[side] + hinge_margin_;
        at_limit |= upper || lower;
        if ((upper && outward > 0) || (lower && outward < 0)) {
            // Project the commanded model effort onto the tangent of the soft
            // constraint. Both hip and knee drives contribute to the hinge.
            const double multiplier = outward / (a * a + b * b);
            tau[hip] -= a * multiplier;
            tau[knee] -= b * multiplier;
        }
    }
    return at_limit;
}

void WheelLegController::compute_motor_torques_() {
    if (state_ != State::kRl || !policy_targets_valid_ || !policy_action_fresh_()) {
        latch_fault_("no fresh policy target at actuation");
        return;
    }
    const auto now = Clock::now();
    Eigen::Vector4d tau = policy_profile_.leg_kp * (policy_targets_.head<4>() - q_.head<4>())
                        - policy_profile_.leg_kd * dq_.head<4>();
    if (gas_spring_compensation_enabled_) {
        const auto gas =
            gas_spring_compensation({q_[0], q_[1], q_[2], q_[3]}, gas_spring_compensation_gain_);
        if (!gas.valid) {
            latch_fault_("invalid gas-spring compensation input");
            return;
        }
        const Eigen::Vector4d compensation = Eigen::Map<const Eigen::Vector4d>{gas.torque.data()};
        tau += compensation;
        const Eigen::Vector4d compensation_api = leg_jacobian_.transpose() * compensation;
        for (int i = 0; i < 4; ++i)
            *gas_compensation_torque_outputs_[i] = compensation_api[i];
        for (int side = 0; side < 2; ++side) {
            *gas_beta_outputs_[side] = gas.beta_deg[side];
            *gas_force_outputs_[side] = gas.force_n[side];
            *gas_weight_outputs_[side] = gas.weight[side];
            *gas_extrapolated_outputs_[side] = gas.extrapolated[side] ? 1.0 : 0.0;
        }
    }
    tau = tau.cwiseMax(-DeployedPolicyContract::kLegTorqueLimit)
              .cwiseMin(DeployedPolicyContract::kLegTorqueLimit);
    const Eigen::Vector2d wheel_tau =
        (policy_profile_.wheel_kp * (policy_targets_.tail<2>() - dq_.tail<2>()))
            .cwiseMax(-policy_profile_.wheel_torque_limit)
            .cwiseMin(policy_profile_.wheel_torque_limit);
    // A crossed hinge boundary only removes outward effort. Preserve inward
    // correction and common rotation without leaving the policy-controlled state.
    const bool at_limit = apply_soft_limits_(tau);
    // MIT position PD would otherwise discard the host's compensated effort.
    *joint_torque_override_ = at_limit || gas_spring_compensation_enabled_;
    const Eigen::Vector4d motor_tau = leg_jacobian_.transpose() * tau;
    if (!motor_tau.allFinite() || !wheel_tau.allFinite()) {
        latch_fault_("non-finite actuator effort");
        return;
    }
    // Scale together when actuator caps differ: independent clipping after the
    // hinge projection could recreate an outward relative effort.
    double effort_scale = 1.0;
    for (int i = 0; i < 4; ++i) {
        const double limit =
            std::min(*max_torque_inputs_[i], DeployedPolicyContract::kLegTorqueLimit);
        if (std::abs(motor_tau[i]) > limit)
            effort_scale = std::min(effort_scale, limit / std::abs(motor_tau[i]));
    }
    for (int i = 0; i < 4; ++i)
        *torque_outputs_[i] = effort_scale * motor_tau[i];
    *joint_torque_override_ = *joint_torque_override_ || effort_scale < 1.0;

    if (use_motor_position_pd_) {
        // Preserve the pair's continuous turn in the encoder API frame.
        // Driver reversal/bias is applied only by DmMotor.
        const Eigen::Vector4d delta =
            leg_jacobian_.transpose() * (policy_targets_.head<4>() - q_.head<4>());
        for (int i = 0; i < 4; ++i) {
            const double target = *angle_inputs_[i] + delta[i];
            if (!std::isfinite(target)) {
                latch_fault_("non-finite motor position target");
                return;
            }
            *angle_outputs_[i] = target;
        }
        *joint_command_ns_ = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
    }
    for (int i = 0; i < 2; ++i) {
        // DjiMotor converts motor current to wheel-output torque using its own ratio.
        const double motor_tau_wheel = wheel_scale_[i] * wheel_tau[i];
        if (!std::isfinite(motor_tau_wheel)) {
            latch_fault_("non-finite wheel effort");
            return;
        }
        *torque_outputs_[4 + i] =
            std::clamp(motor_tau_wheel, -*max_torque_inputs_[4 + i], *max_torque_inputs_[4 + i]);
    }
}

} // namespace rmcs_core::controller::chassis
