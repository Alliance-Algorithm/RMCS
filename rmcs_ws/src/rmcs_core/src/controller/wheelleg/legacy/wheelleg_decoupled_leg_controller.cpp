#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <utility>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "controller/wheelleg/leg_transmission.hpp"

namespace rmcs_core::controller::wheelleg {

// Per-leg task-space computed-torque controller.
//
// The V5.0 six-bar strongly couples the equivalent-leg length L0 and angle
// alpha: the reduced Jacobian gives tau = J^T Gamma with
//   tau_hip   = -a F_L + (1-b) T_a
//   tau_crank =  a F_L +     b T_a
// and b/a ~ 3.7, so two independent SISO loops fight. Instead we invert the
// task-space inertia M_task (measured from the MJCF at S*, ~constant) and issue
//   Gamma = M_task * [a_L*, a_alpha*]^T
// with a_L*, a_alpha* from per-leg PID on (L0, alpha). The integral absorbs the
// gravity/coupling bias. Outputs are per-leg hip/crank torques.
class WheelLegDecoupledLegController
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegDecoupledLegController()
        : Node{get_component_name(), node::options()} {
        load_config_();
        linkage_ = std::make_unique<rmcs_core::wheelleg::LegLinkage>();

        register_input("/wheelleg/state/leg_length_l", leg_length_l_);
        register_input("/wheelleg/state/leg_length_r", leg_length_r_);
        register_input("/wheelleg/state/leg_angle_l", leg_angle_l_);
        register_input("/wheelleg/state/leg_angle_r", leg_angle_r_);
        register_input("/wheelleg/test/leg_length_ref_l", ref_length_l_);
        register_input("/wheelleg/test/leg_length_ref_r", ref_length_r_);
        register_input("/wheelleg/task/alpha_ref", ref_alpha_);

        register_output("/wheelleg/control/hip_torque_l", hip_torque_l_, kNaN);
        register_output("/wheelleg/control/crank_torque_l", crank_torque_l_, kNaN);
        register_output("/wheelleg/control/hip_torque_r", hip_torque_r_, kNaN);
        register_output("/wheelleg/control/crank_torque_r", crank_torque_r_, kNaN);
    }

    void update() override {
        if (!leg_length_l_.ready() || !leg_length_r_.ready() || !leg_angle_l_.ready()
            || !leg_angle_r_.ready() || !ref_length_l_.ready() || !ref_length_r_.ready()
            || !ref_alpha_.ready() || !std::isfinite(*leg_length_l_)
            || !std::isfinite(*leg_length_r_) || !std::isfinite(*leg_angle_l_)
            || !std::isfinite(*leg_angle_r_) || !std::isfinite(*ref_length_l_)
            || !std::isfinite(*ref_length_r_) || !std::isfinite(*ref_alpha_)) {
            output_nan_();
            return;
        }

        const double dt = dt_ > 0.0 ? dt_ : 1e-3;
        const double L0_l = *leg_length_l_;
        const double L0_r = *leg_length_r_;
        const double a_l = *leg_angle_l_;
        const double a_r = *leg_angle_r_;

        if (!started_) {
            prev_L0_l_ = L0_l;
            prev_L0_r_ = L0_r;
            prev_alpha_l_ = a_l;
            prev_alpha_r_ = a_r;
            started_ = true;
        }
        const double dL0_l = (L0_l - prev_L0_l_) / dt;
        const double dL0_r = (L0_r - prev_L0_r_) / dt;
        const double da_l = (a_l - prev_alpha_l_) / dt;
        const double da_r = (a_r - prev_alpha_r_) / dt;
        prev_L0_l_ = L0_l;
        prev_L0_r_ = L0_r;
        prev_alpha_l_ = a_l;
        prev_alpha_r_ = a_r;

        const double alpha_ref = *ref_alpha_;
        leg_state_[0] = {L0_l, a_l, dL0_l, da_l, *ref_length_l_, alpha_ref};
        leg_state_[1] = {L0_r, a_r, dL0_r, da_r, *ref_length_r_, alpha_ref};

        double tau_hip[2];
        double tau_crank[2];
        for (int i = 0; i < 2; ++i)
            solve_leg_(i, tau_hip[i], tau_crank[i]);

        *hip_torque_l_ = limit_hip_(tau_hip[0]);
        *crank_torque_l_ = limit_crank_(tau_crank[0]);
        *hip_torque_r_ = limit_hip_(tau_hip[1]);
        *crank_torque_r_ = limit_crank_(tau_crank[1]);

        if (dbg_count_++ % 500 == 0)
            info(
                "decoupled L0L={:.4f} L0R={:.4f} aL={:.4f} aR={:.4f} aRef={:.3f} "
                "tauHl={:.2f} tauCl={:.2f}",
                L0_l, L0_r, a_l, a_r, alpha_ref, *hip_torque_l_, *crank_torque_l_);
    }

private:
    struct LegState {
        double L0;
        double alpha;
        double dL0;
        double dalpha;
        double ref_length;
        double ref_alpha;
    };

    void solve_leg_(int i, double& tau_hip, double& tau_crank) {
        const LegState& s = leg_state_[i];
        const double e_L = s.ref_length - s.L0;
        const double e_a = s.ref_alpha - s.alpha;

        integral_[i][0] = std::clamp(
            integral_[i][0] + e_L * dt_, -integral_limit_, integral_limit_);
        integral_[i][1] = std::clamp(
            integral_[i][1] + e_a * dt_, -integral_limit_, integral_limit_);

        const double acc_L = kp_L_ * e_L + ki_L_ * integral_[i][0] - kd_L_ * s.dL0;
        const double acc_a = kp_a_ * e_a + ki_a_ * integral_[i][1] - kd_a_ * s.dalpha;

        // Quasi-static gravity generalized force per leg (shared: /2).
        const double m_half = 0.5 * total_mass_;
        const double bias_L = m_half * gravity_ * std::sin(s.alpha);
        const double bias_a = m_half * gravity_ * s.L0 * std::cos(s.alpha);

        const double force_length = m_task_[0][0] * acc_L + m_task_[0][1] * acc_a + bias_L;
        const double torque_alpha = m_task_[1][0] * acc_L + m_task_[1][1] * acc_a + bias_a;

        const auto [a, b] = leg_jacobian_(s.L0);
        double th = hip_torque_sign_ * (-a * force_length + (1.0 - b) * torque_alpha);
        double tc = crank_torque_sign_ * (a * force_length + b * torque_alpha);

        // Command low-pass to avoid exciting the ~5 Hz structural mode.
        if (filter_hz_ > 0.0) {
            const double alpha = 1.0 - std::exp(-2.0 * std::numbers::pi * filter_hz_ * dt_);
            filtered_[i][0] += alpha * (th - filtered_[i][0]);
            filtered_[i][1] += alpha * (tc - filtered_[i][1]);
            th = filtered_[i][0];
            tc = filtered_[i][1];
        }
        tau_hip = th;
        tau_crank = tc;
    }

    std::pair<double, double> leg_jacobian_(double L0) const {
        const double L1 = thigh_length_;
        const double L2 = shank_length_;
        const double cos_beta =
            std::clamp((L1 * L1 + L2 * L2 - L0 * L0) / (2.0 * L1 * L2), -1.0, 1.0);
        const double beta = std::acos(cos_beta);
        const double d_leg_length_d_beta = L1 * L2 * std::sin(beta) / std::max(L0, kEpsilon);
        const double d_gamma_d_beta = (L1 * L2 * cos_beta - L2 * L2) / (L0 * L0);

        double d_beta_d_crank = 0.0;
        constexpr double kDelta = 1e-3;
        double cp = 0.0;
        double cm = 0.0;
        if (linkage_->crank_from_knee(beta + kDelta, cp)
            && linkage_->crank_from_knee(beta - kDelta, cm)
            && std::abs(cp - cm) > kEpsilon)
            d_beta_d_crank = 2.0 * kDelta / (cp - cm);

        return {d_leg_length_d_beta * d_beta_d_crank * jacobian_scale_,
                -d_gamma_d_beta * d_beta_d_crank * jacobian_scale_};
    }

    double limit_hip_(double t) const { return std::clamp(t, -hip_torque_limit_, hip_torque_limit_); }
    double limit_crank_(double t) const {
        return std::clamp(t, -crank_torque_limit_, crank_torque_limit_);
    }

    void output_nan_() {
        *hip_torque_l_ = kNaN;
        *crank_torque_l_ = kNaN;
        *hip_torque_r_ = kNaN;
        *crank_torque_r_ = kNaN;
        started_ = false;
    }

    void load_config_() {
        thigh_length_ = node::param_or("thigh_length", 0.21);
        shank_length_ = node::param_or("shank_length", 0.25005);
        hip_torque_limit_ = node::param_or("hip_torque_limit", 40.0);
        crank_torque_limit_ = node::param_or("crank_torque_limit", 40.0);
        jacobian_scale_ = node::param_or("jacobian_scale", 1.0);
        hip_torque_sign_ = node::param_or("hip_torque_sign", 1.0);
        crank_torque_sign_ = node::param_or("crank_torque_sign", 1.0);
        dt_ = node::param_or("dt", 0.001);

        kp_L_ = node::param_or("kp_L", 25.0);
        ki_L_ = node::param_or("ki_L", 2.0);
        kd_L_ = node::param_or("kd_L", 8.0);
        kp_a_ = node::param_or("kp_a", 6.0);
        ki_a_ = node::param_or("ki_a", 0.5);
        kd_a_ = node::param_or("kd_a", 2.0);
        integral_limit_ = node::param_or("integral_limit", 5.0);
        filter_hz_ = node::param_or("filter_hz", 3.0);
        total_mass_ = node::param_or("total_mass", 13.702);
        gravity_ = node::param_or("gravity", 9.81);

        m_task_[0][0] = node::param_or("m_task_11", 0.664);
        m_task_[0][1] = node::param_or("m_task_12", -0.042);
        m_task_[1][0] = node::param_or("m_task_21", -0.042);
        m_task_[1][1] = node::param_or("m_task_22", 0.067);
    }

    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kEpsilon = 1e-9;

    double thigh_length_;
    double shank_length_;
    double hip_torque_limit_;
    double crank_torque_limit_;
    double jacobian_scale_;
    double hip_torque_sign_;
    double crank_torque_sign_;
    double dt_;

    double kp_L_, ki_L_, kd_L_;
    double kp_a_, ki_a_, kd_a_;
    double integral_limit_;
    double filter_hz_ = 3.0;
    double total_mass_ = 13.702;
    double gravity_ = 9.81;
    double m_task_[2][2] = {{0.664, -0.042}, {-0.042, 0.067}};

    std::unique_ptr<rmcs_core::wheelleg::LegLinkage> linkage_;

    double prev_L0_l_ = 0.0, prev_L0_r_ = 0.0, prev_alpha_l_ = 0.0, prev_alpha_r_ = 0.0;
    bool started_ = false;
    double integral_[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
    double filtered_[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
    LegState leg_state_[2];
    std::uint64_t dbg_count_ = 0;

    InputInterface<double> leg_length_l_;
    InputInterface<double> leg_length_r_;
    InputInterface<double> leg_angle_l_;
    InputInterface<double> leg_angle_r_;
    InputInterface<double> ref_length_l_;
    InputInterface<double> ref_length_r_;
    InputInterface<double> ref_alpha_;

    OutputInterface<double> hip_torque_l_;
    OutputInterface<double> crank_torque_l_;
    OutputInterface<double> hip_torque_r_;
    OutputInterface<double> crank_torque_r_;
};

} // namespace rmcs_core::controller::wheelleg

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::wheelleg::WheelLegDecoupledLegController,
    rmcs_executor::Component)
