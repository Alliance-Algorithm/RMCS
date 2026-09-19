#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>

namespace rmcs_core::wheelleg {

// Reduced serial-leg kinematics for the sagittal plane, matching
// 执行路线1_数学解析.md §4. Plane coordinates are (forward, up); an angle of a
// vector v is gamma(v) = atan2(up, forward).
//
//   psi1    = pi - theta - q1     (thigh absolute angle)
//   beta    = knee_zero - q2      (knee, straight ~= 0; left leg)
//   L0      = |HA|                = sqrt(L1^2 + L2^2 - 2 L1 L2 cos(beta))
//   alpha_L = gamma(A -> H)       = psi1 + atan2(L2 sin(beta), L1 - L2 cos(beta))
//
// All quantities here are the LEFT-leg convention used by
// hardware::WheelLegSimulation; the right leg is mirrored there. The class is
// the analytic counterpart of the bridge's anchor-based state mapping, and the
// source of the task Jacobian used to reflect task wrenches to motor torques.
struct LegKinematicsParams {
    double thigh_length = 0.21;            // L1: hip H -> knee K
    double shank_length = 0.25005;         // L2: knee K -> axle A (V5.0)
    double wheel_radius = 0.06;            // r
    double hip_zero = 1.568956;            // alpha offset at q1 = 0 (V5.0)
    double knee_zero = 0.784469;           // interior beta at q2 = 0 (V5.0)
};

struct LegTaskState {
    double leg_length = 0.0;   // L0 = |HA|
    double leg_angle = 0.0;    // alpha_L, absolute (ground-relative)
    double knee = 0.0;         // beta
};

// Task Jacobian J = d(L0, alpha_L) / d(q1, q2) at fixed body pitch theta.
struct LegJacobian {
    double d_leg_length_d_q1 = 0.0;
    double d_leg_length_d_q2 = 0.0;
    double d_leg_angle_d_q1 = 0.0;
    double d_leg_angle_d_q2 = 0.0;
};

class LegKinematics {
public:
    explicit LegKinematics(const LegKinematicsParams& params = {}) : p_(params) {}

    [[nodiscard]] const LegKinematicsParams& params() const { return p_; }

    // Reduced knee joint angle -> knee angle beta.
    [[nodiscard]] double knee_from_joint(double q2) const { return p_.knee_zero - q2; }

    // Knee joint angle for a target beta.
    [[nodiscard]] double joint_from_knee(double beta) const { return p_.knee_zero - beta; }

    // Leg length for a knee angle.
    [[nodiscard]] double leg_length(double beta) const {
        const double L1 = p_.thigh_length;
        const double L2 = p_.shank_length;
        return std::sqrt(L1 * L1 + L2 * L2 - 2.0 * L1 * L2 * std::cos(beta));
    }

    // Knee angle for a leg length (straight branch, beta in [0, pi]).
    [[nodiscard]] bool knee_from_leg_length(double leg_length, double& beta) const {
        const double L1 = p_.thigh_length;
        const double L2 = p_.shank_length;
        const double c = (L1 * L1 + L2 * L2 - leg_length * leg_length) / (2.0 * L1 * L2);
        if (c < -1.0 || c > 1.0)
            return false;
        beta = std::acos(std::clamp(c, -1.0, 1.0));
        return true;
    }

    // Forward kinematics: (q1, q2, theta) -> (L0, alpha_L). theta is the body
    // pitch (rad). alpha_L is absolute (includes theta).
    [[nodiscard]] LegTaskState forward(double q1, double q2, double theta) const {
        const double L1 = p_.thigh_length;
        const double L2 = p_.shank_length;
        const double beta = knee_from_joint(q2);
        const double psi1 = std::numbers::pi - theta - q1;

        LegTaskState state;
        state.knee = beta;
        state.leg_length = leg_length(beta);
        state.leg_angle =
            psi1 + std::atan2(L2 * std::sin(beta), L1 - L2 * std::cos(beta));
        return state;
    }

    // Inverse: (L0, alpha_L, theta) -> (q1, q2). Returns false if unreachable.
    [[nodiscard]] bool inverse(
        double leg_length_target, double leg_angle_target, double theta, double& q1,
        double& q2) const {
        const double L1 = p_.thigh_length;
        const double L2 = p_.shank_length;
        double beta = 0.0;
        if (!knee_from_leg_length(leg_length_target, beta))
            return false;
        const double psi1 = leg_angle_target
                          - std::atan2(L2 * std::sin(beta), L1 - L2 * std::cos(beta));
        q1 = std::numbers::pi - theta - psi1;
        q2 = joint_from_knee(beta);
        return true;
    }

    // ---------------------------------------------------------------------
    // Equivalent leg (hip H -> axle A), commanded by its angle relative to the
    // body plane and its length. This is the interface a balance controller
    // reasons in: the leg angle steers the CoM horizontally (static stability)
    // and the leg length sets the height.
    //
    // The body-relative leg angle is gamma = gamma(A -> H) measured in the body
    // frame (forward, up); unlike alpha_L it does NOT include the body pitch, so
    // a command is pitch-independent.
    // ---------------------------------------------------------------------
    [[nodiscard]] double leg_angle_body(double q1, double beta) const {
        const double L1 = p_.thigh_length;
        const double L2 = p_.shank_length;
        // Body-relative thigh angle is (pi - q1); see 执行路线1 §4.
        return (std::numbers::pi - q1)
             + std::atan2(L2 * std::sin(beta), L1 - L2 * std::cos(beta));
    }

    struct BodyTaskState {
        double leg_length = 0.0;      // L0 = |HA|
        double leg_angle_body = 0.0;  // gamma(A->H) in the body frame
        double knee = 0.0;            // beta
    };

    [[nodiscard]] BodyTaskState forward_body(double q1, double q2) const {
        const double beta = knee_from_joint(q2);
        BodyTaskState state;
        state.knee = beta;
        state.leg_length = leg_length(beta);
        state.leg_angle_body = leg_angle_body(q1, beta);
        return state;
    }

    // Reduced hip/knee references expected by WheelLegSimulation
    // (/wheelleg/reference/{hip_angle,knee_angle}); the bridge maps the knee
    // reference to the crank joint through the real four-bar.
    struct LegCommand {
        double hip_angle = 0.0;    // reduced hip reference
        double knee_angle = 0.0;   // reduced knee reference (beta)
        bool feasible = false;
    };

    // Equivalent-leg inverse: (leg length, body-relative leg angle) -> motor
    // references. branch_sign selects the assembly four-bar branch (-1 here).
    [[nodiscard]] LegCommand command_from_body(
        double leg_length_target, double leg_angle_body_target,
        double branch_sign = -1.0) const {
        LegCommand command;
        const double L1 = p_.thigh_length;
        const double L2 = p_.shank_length;
        double beta_abs = 0.0;
        if (!knee_from_leg_length(leg_length_target, beta_abs))
            return command;

        const double beta = branch_sign * beta_abs;
        const double g = std::atan2(L2 * std::sin(beta), L1 - L2 * std::cos(beta));
        // q1 = pi - gamma + g  =>  reduced hip = q1 - pi = g - gamma.
        command.hip_angle = g - leg_angle_body_target;
        command.knee_angle = beta;
        command.feasible = true;
        return command;
    }

    // Task Jacobian at a given knee angle beta (holds theta fixed).
    //   dL0/dbeta   =  L1 L2 sin(beta) / L0
    //   dalpha/dbeta = (L1 L2 cos(beta) - L2^2) / L0^2
    //   dbeta/dq2    = -1
    [[nodiscard]] LegJacobian jacobian(double beta) const {
        const double L1 = p_.thigh_length;
        const double L2 = p_.shank_length;
        const double L0 = std::max(leg_length(beta), 1e-9);
        const double d_leg_length_d_beta = L1 * L2 * std::sin(beta) / L0;
        const double d_leg_angle_d_beta =
            (L1 * L2 * std::cos(beta) - L2 * L2) / (L0 * L0);

        LegJacobian j;
        j.d_leg_length_d_q1 = 0.0;
        j.d_leg_length_d_q2 = d_leg_length_d_beta * (-1.0);
        j.d_leg_angle_d_q1 = -1.0;
        j.d_leg_angle_d_q2 = d_leg_angle_d_beta * (-1.0);
        return j;
    }

    [[nodiscard]] LegJacobian jacobian_from_joint(double q2) const {
        return jacobian(knee_from_joint(q2));
    }

    // Reduced-model CoM horizontal offset from the wheel axle,
    //   d = (C_veh - A)_forward,
    // following 执行路线1_数学解析.md §3 (assembly branch sigma = -1). This is
    // the serial reduced model: single leg x combined masses. For V5.0 the
    // Stephenson six-bar is lumped into the thigh / shank groups (all LL* links
    // with the thigh, the nitrogen-spring mounts with their parent), so the
    // result is approximate; the bridge computes d exactly from MuJoCo instead.
    [[nodiscard]] double com_offset(double leg_length, double leg_angle, double theta)
        const {
        const double L1 = p_.thigh_length;
        const double L2 = p_.shank_length;
        const double L0 = leg_length;
        const double ca = std::cos(leg_angle);
        const double sa = std::sin(leg_angle);

        const double hx = L0 * ca;
        const double hy = L0 * sa;
        const double a = (L0 * L0 + L2 * L2 - L1 * L1) / (2.0 * L0);
        const double b = -std::sqrt(std::max(L2 * L2 - a * a, 0.0));
        const double kx = a * ca - b * sa;
        const double ky = a * sa + b * ca;

        // forward component of R(t) * v.
        auto rot_x = [](double t, double vx, double vy) {
            return std::cos(t) * vx - std::sin(t) * vy;
        };

        // V5.0 reduced parameters (models/wheelleg/calibrate_kinematics.py):
        // thigh group length/COM offset/angle, shank group likewise.
        constexpr double d1 = 0.06623;
        constexpr double beta1 = 0.17659;
        constexpr double d2 = 0.15344;
        constexpr double beta2 = 0.03038;
        // Body COM relative to the hip in the body frame (forward, up).
        constexpr double r_bx = 0.00918;
        constexpr double r_bz = -0.04908;

        const double c1x = hx + d1 * rot_x(beta1, (kx - hx) / L1, (ky - hy) / L1);
        const double c2x = kx + d2 * rot_x(beta2, (0.0 - kx) / L2, (0.0 - ky) / L2);
        // Body CoM forward component uses R_y(theta) applied to r_b.
        const double cbx = hx + std::cos(theta) * r_bx + std::sin(theta) * r_bz;

        constexpr double m_b = 11.62;
        constexpr double m_1 = 0.852;
        constexpr double m_2 = 1.014;
        constexpr double m_w = 0.40;
        constexpr double mass = m_b + m_1 + m_2 + m_w;
        return (m_b * cbx + m_1 * c1x + m_2 * c2x) / mass;
    }

private:
    LegKinematicsParams p_;
};

} // namespace rmcs_core::wheelleg
