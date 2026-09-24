#include "rl_controller.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace rmcs::rl {

void RlController::process_action_(const std::array<float, 6>& raw) {
    for (int i = 0; i < 6; ++i) {
        if (!std::isfinite(raw[i]))
            throw std::runtime_error("ONNX returned a non-finite action");
        previous_action_[i] = std::clamp(raw[i], i < 4 ? -3.0f : -9.0f, i < 4 ? 3.0f : 9.0f);
    }
    for (int i = 0; i < 4; ++i) {
        const double desired = nominal_[i] + 0.25 * previous_action_[i];
        targets_[i] = q_[i] + std::remainder(desired - q_[i], 2 * std::numbers::pi);
    }
    for (int side = 0; side < 2; ++side) {
        const int hip = 2 * side, knee = hip + 1;
        // A calibrated hinge row maps the two active coordinates to the true
        // thigh-shank relative angle. It is not assumed to equal q_knee - q_hip.
        const double a = hinge_coeff_[hip], b = hinge_coeff_[knee];
        const double low =
            (hinge_min_[side] + hinge_margin_ - hinge_bias_[side] - a * targets_[hip]) / b;
        const double high =
            (hinge_max_[side] - hinge_margin_ - hinge_bias_[side] - a * targets_[hip]) / b;
        targets_[knee] = std::clamp(targets_[knee], std::min(low, high), std::max(low, high));
    }
    // V5 actor output P[4:6] are wheel speed commands (rad/s), not torque.
    targets_[4] = 10.0 * previous_action_[4];
    targets_[5] = 10.0 * previous_action_[5];
    if (!targets_.allFinite())
        throw std::runtime_error(
            "Policy action or soft-limit mapping generated a non-finite target");
}

void RlController::apply_soft_limits_(Eigen::Vector4d& tau) const {
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
    Eigen::Vector4d tau;
    for (int i = 0; i < 4; ++i) {
        const double kp = state_ == State::kPrepare ? prepare_kp_ : 60.0;
        const double kd = state_ == State::kPrepare ? prepare_kd_ : 2.0;
        tau[i] = std::clamp(kp * (targets_[i] - q_[i]) - kd * dq_[i], -40.0, 40.0);
    }
    // Enforce the relative hinge limit during PREPARE as well as during RL.
    // Stop on a measured violation beyond the guard band; physical stops are
    // not replaced by this software clamp.
    for (int side = 0; side < 2; ++side) {
        const int hip = 2 * side;
        const double relative =
            hinge_coeff_[hip] * q_[hip] + hinge_coeff_[hip + 1] * q_[hip + 1] + hinge_bias_[side];
        if (!std::isfinite(relative) || relative < hinge_min_[side] - hinge_margin_
            || relative > hinge_max_[side] + hinge_margin_) {
            fault_latched_ = true;
            enter_(State::kIdle);
            clear_outputs_();
            return;
        }
    }
    apply_soft_limits_(tau);
    const Eigen::Vector4d motor_tau = leg_jacobian_.transpose() * tau;
    if (!motor_tau.allFinite()) {
        fault_latched_ = true;
        enter_(State::kIdle);
        clear_outputs_();
        return;
    }
    for (int i = 0; i < 4; ++i)
        *torque_outputs_[i] =
            std::clamp(motor_tau[i], -*max_torque_inputs_[i], *max_torque_inputs_[i]);

    for (int i = 0; i < 2; ++i) {
        const double target = state_ == State::kPrepare ? 0.0 : targets_[4 + i];
        const double model_tau = 0.2 * (target - dq_[4 + i]);
        const double motor_tau_wheel = wheel_scale_[i] * model_tau;
        if (!std::isfinite(motor_tau_wheel)) {
            fault_latched_ = true;
            enter_(State::kIdle);
            clear_outputs_();
            return;
        }
        // DjiMotor provides the installed M3508's own current/ratio torque limit.
        *torque_outputs_[4 + i] =
            std::clamp(motor_tau_wheel, -*max_torque_inputs_[4 + i], *max_torque_inputs_[4 + i]);
    }
}

} // namespace rmcs::rl
