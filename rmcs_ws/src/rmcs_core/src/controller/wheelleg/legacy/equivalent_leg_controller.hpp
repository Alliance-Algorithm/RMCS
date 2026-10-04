#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <numbers>
#include <utility>

#include "controller/wheelleg/legacy/leg_kinematics.hpp"
#include "controller/wheelleg/leg_transmission.hpp"

namespace rmcs_core::wheelleg {

// Torque-level control of the equivalent leg (hip H -> axle A), commanded by
// its length L0 and its angle gamma relative to the body plane (forward, up).
//
// The leg is driven by the hip (thigh) and crank motors; the knee is passive
// and follows the crank through the four-bar. For a task wrench
//   Gamma = [F_L (along the leg), T_gamma (about the leg angle)]
// the motor torques follow from virtual work,
//   tau = J^T Gamma,   J = d(L0, gamma) / d(q_hip, q_crank),
// with J obtained by finite differences of the forward map so it stays exact
// across the closed chain (and near the four-bar limits). Joint viscous damping
// is handled implicitly by the model, so only weak task-rate feedback is used.
struct EquivalentLegParams {
    double thigh_length = 0.21;   // L1
    double shank_length = 0.25005; // L2

    // Task-space impedance gains.
    double kp_length = 800.0;
    double kd_length = 3.0;
    double kp_angle = 200.0;
    double kd_angle = 1.0;

    double max_hip_torque = 40.0;
    double max_crank_torque = 40.0;
};

class EquivalentLegController {
public:
    explicit EquivalentLegController(const EquivalentLegParams& params = {}) : p_(params) {
        linkage_ = std::make_unique<LegLinkage>();
    }

    struct TaskState {
        double leg_length = 0.0;       // L0
        double leg_angle_body = 0.0;   // gamma in the body frame
        double beta = 0.0;             // knee angle
        bool feasible = false;
    };

    // Forward map: motor state -> equivalent-leg state.
    [[nodiscard]] TaskState task_from_motors(double q_hip, double q_crank) const {
        TaskState state;
        // V5.0 six-bar: reduced knee depends on the relative crank q_crank - q_hip.
        const double c_eff = q_crank - q_hip;
        const double beta = linkage_->knee_from_crank(c_eff);

        const double L1 = p_.thigh_length;
        const double L2 = p_.shank_length;
        const double g = std::atan2(L2 * std::sin(beta), L1 - L2 * std::cos(beta));

        state.beta = beta;
        state.leg_length = std::sqrt(L1 * L1 + L2 * L2 - 2.0 * L1 * L2 * std::cos(beta));
        state.leg_angle_body = (std::numbers::pi - q_hip) + g;
        state.feasible = true;
        return state;
    }

    struct Torque {
        double hip = 0.0;
        double crank = 0.0;
    };

    // Task wrench -> motor torques (pure task-space impedance, no gravity
    // feed-forward; the leg's weight and the body load are left to the gains).
    [[nodiscard]] Torque compute(
        double leg_length_des, double leg_angle_body_des, double q_hip, double q_crank,
        double dq_hip, double dq_crank) const {
        Torque torque;
        const TaskState state = task_from_motors(q_hip, q_crank);
        if (!state.feasible)
            return torque;

        // Task Jacobian by central differences (exact across the four-bar).
        constexpr double kStep = 1e-6;
        const auto plus = [&](double dq, bool hip) {
            return hip ? task_from_motors(q_hip + dq, q_crank)
                       : task_from_motors(q_hip, q_crank + dq);
        };
        const auto minus = [&](double dq, bool hip) {
            return hip ? task_from_motors(q_hip - dq, q_crank)
                       : task_from_motors(q_hip, q_crank - dq);
        };
        const auto hip_p = plus(kStep, true);
        const auto hip_m = minus(kStep, true);
        const auto crank_p = plus(kStep, false);
        const auto crank_m = minus(kStep, false);
        if (!hip_p.feasible || !hip_m.feasible || !crank_p.feasible || !crank_m.feasible)
            return torque;

        const double j_l0_hip = (hip_p.leg_length - hip_m.leg_length) / (2.0 * kStep);
        const double j_ga_hip =
            (hip_p.leg_angle_body - hip_m.leg_angle_body) / (2.0 * kStep);
        const double j_l0_crank = (crank_p.leg_length - crank_m.leg_length) / (2.0 * kStep);
        const double j_ga_crank =
            (crank_p.leg_angle_body - crank_m.leg_angle_body) / (2.0 * kStep);

        const double d_leg_length = j_l0_hip * dq_hip + j_l0_crank * dq_crank;
        const double d_leg_angle = j_ga_hip * dq_hip + j_ga_crank * dq_crank;

        const double force_length =
            p_.kp_length * (leg_length_des - state.leg_length) - p_.kd_length * d_leg_length;
        const double torque_angle =
            p_.kp_angle * (leg_angle_body_des - state.leg_angle_body) - p_.kd_angle * d_leg_angle;

        torque.hip = j_l0_hip * force_length + j_ga_hip * torque_angle;
        torque.crank = j_l0_crank * force_length + j_ga_crank * torque_angle;

        torque.hip = std::clamp(torque.hip, -p_.max_hip_torque, p_.max_hip_torque);
        torque.crank = std::clamp(torque.crank, -p_.max_crank_torque, p_.max_crank_torque);
        return torque;
    }

    // Inverse (kinematic) command for the bridge references, kept for
    // consistency with leg_kinematics.hpp.
    [[nodiscard]] std::pair<double, double> command_from_body(
        double leg_length_target, double leg_angle_body_target) const {
        const double L1 = p_.thigh_length;
        const double L2 = p_.shank_length;
        const double c = (L1 * L1 + L2 * L2 - leg_length_target * leg_length_target)
                       / (2.0 * L1 * L2);
        if (c < -1.0 || c > 1.0)
            return {0.0, 0.0};
        const double beta = -(std::acos(std::clamp(c, -1.0, 1.0)));
        const double g = std::atan2(L2 * std::sin(beta), L1 - L2 * std::cos(beta));
        return {g - leg_angle_body_target, beta};
    }

private:
    EquivalentLegParams p_;
    std::unique_ptr<LegLinkage> linkage_;
};

} // namespace rmcs_core::wheelleg
