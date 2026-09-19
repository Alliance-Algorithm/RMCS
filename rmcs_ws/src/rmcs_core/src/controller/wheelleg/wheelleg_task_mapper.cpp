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

// Per-leg task-space mapper (执行路线2 §13, task-space J^T).
//
// The two coaxial hip motors do not own one task each: the V5.0 crank is
// referenced to the body, so the relative crank c = q_crank - q_hip drives the
// knee (hence the equivalent-leg length L0), and the hip drives the leg angle
// alpha_L. Both motors move both task coordinates. This component collects
// per-leg length forces and per-leg leg-angle forces and maps them to per-leg
// reduced joint torques through the transpose Jacobian
//
//   [tau_hip  ]   [ -a     1 - b ] [F_L]
//   [tau_crank] = [  a     b     ] [T_a]
//
// where a = (dL0/dbeta)(dbeta/dc) and b = (dg/dbeta)(dbeta/dc) are evaluated
// live from each leg's measured equivalent-leg length. Per-leg outputs remove
// the unactuated antisymmetric mode of the shared interface.
class WheelLegTaskMapper
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegTaskMapper()
        : Node{get_component_name(), node::options()} {
        load_config_();
        linkage_ = std::make_unique<rmcs_core::wheelleg::LegLinkage>();

        register_input("/wheelleg/task/force_length_l", force_length_l_);
        register_input("/wheelleg/task/force_length_r", force_length_r_);
        register_input("/wheelleg/task/torque_alpha_l", torque_alpha_l_);
        register_input("/wheelleg/task/torque_alpha_r", torque_alpha_r_);
        register_input("/wheelleg/state/leg_length_l", leg_length_l_);
        register_input("/wheelleg/state/leg_length_r", leg_length_r_);

        register_output("/wheelleg/control/hip_torque_l", hip_torque_l_, kNaN);
        register_output("/wheelleg/control/crank_torque_l", crank_torque_l_, kNaN);
        register_output("/wheelleg/control/hip_torque_r", hip_torque_r_, kNaN);
        register_output("/wheelleg/control/crank_torque_r", crank_torque_r_, kNaN);
    }

    void update() override {
        if (!force_length_l_.ready() || !force_length_r_.ready() || !torque_alpha_l_.ready()
            || !torque_alpha_r_.ready() || !leg_length_l_.ready() || !leg_length_r_.ready()
            || !std::isfinite(*force_length_l_) || !std::isfinite(*force_length_r_)
            || !std::isfinite(*torque_alpha_l_) || !std::isfinite(*torque_alpha_r_)
            || !std::isfinite(*leg_length_l_) || !std::isfinite(*leg_length_r_)) {
            *hip_torque_l_ = kNaN;
            *crank_torque_l_ = kNaN;
            *hip_torque_r_ = kNaN;
            *crank_torque_r_ = kNaN;
            return;
        }

        const double force_l = filter_(force_length_l_filter_, *force_length_l_);
        const double force_r = filter_(force_length_r_filter_, *force_length_r_);
        const double torque_l = filter_(torque_alpha_l_filter_, *torque_alpha_l_);
        const double torque_r = filter_(torque_alpha_r_filter_, *torque_alpha_r_);

        const auto [a_l, b_l] = leg_jacobian_(*leg_length_l_);
        const auto [a_r, b_r] = leg_jacobian_(*leg_length_r_);

        const double tau_hip_l = -a_l * force_l + (1.0 - b_l) * torque_l;
        const double tau_crank_l = a_l * force_l + b_l * torque_l;
        const double tau_hip_r = -a_r * force_r + (1.0 - b_r) * torque_r;
        const double tau_crank_r = a_r * force_r + b_r * torque_r;

        *hip_torque_l_ = std::clamp(
            hip_torque_sign_ * tau_hip_l, -hip_torque_limit_, hip_torque_limit_);
        *crank_torque_l_ = std::clamp(
            crank_torque_sign_ * tau_crank_l, -crank_torque_limit_, crank_torque_limit_);
        *hip_torque_r_ = std::clamp(
            hip_torque_sign_ * tau_hip_r, -hip_torque_limit_, hip_torque_limit_);
        *crank_torque_r_ = std::clamp(
            crank_torque_sign_ * tau_crank_r, -crank_torque_limit_, crank_torque_limit_);

        if (dbg_count_++ % 500 == 0)
            info(
                "mapper L0L={:.4f} L0R={:.4f} aL={:.4f} aR={:.4f} FLl={:.1f} FLr={:.1f} "
                "Tl={:.1f} Tr={:.1f}",
                *leg_length_l_, *leg_length_r_, a_l, a_r, force_l, force_r, torque_l,
                torque_r);
    }

private:
    // Command low-pass that keeps the task loop from exciting the leg's ~5 Hz
    // structural mode. filter_hz <= 0 disables it.
    double filter_(double& state, double raw) const {
        if (filter_hz_ <= 0.0)
            return raw;
        const double alpha = 1.0 - std::exp(-2.0 * std::numbers::pi * filter_hz_ * dt_);
        state += alpha * (raw - state);
        return state;
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
        double crank_plus = 0.0;
        double crank_minus = 0.0;
        if (linkage_->crank_from_knee(beta + kDelta, crank_plus)
            && linkage_->crank_from_knee(beta - kDelta, crank_minus)
            && std::abs(crank_plus - crank_minus) > kEpsilon)
            d_beta_d_crank = 2.0 * kDelta / (crank_plus - crank_minus);

        const double a = d_leg_length_d_beta * d_beta_d_crank * jacobian_scale_;
        const double b = -d_gamma_d_beta * d_beta_d_crank * jacobian_scale_;
        return {a, b};
    }

    void load_config_() {
        thigh_length_ = node::param_or("thigh_length", 0.21);
        shank_length_ = node::param_or("shank_length", 0.25005);
        hip_torque_limit_ = node::param_or("hip_torque_limit", 40.0);
        crank_torque_limit_ = node::param_or("crank_torque_limit", 40.0);
        jacobian_scale_ = node::param_or("jacobian_scale", 1.0);
        hip_torque_sign_ = node::param_or("hip_torque_sign", 1.0);
        crank_torque_sign_ = node::param_or("crank_torque_sign", 1.0);
        filter_hz_ = node::param_or("filter_hz", 0.0);
        dt_ = node::param_or("dt", 0.001);
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
    double filter_hz_;
    double dt_;
    double force_length_l_filter_ = 0.0;
    double force_length_r_filter_ = 0.0;
    double torque_alpha_l_filter_ = 0.0;
    double torque_alpha_r_filter_ = 0.0;

    std::unique_ptr<rmcs_core::wheelleg::LegLinkage> linkage_;

    std::uint64_t dbg_count_ = 0;

    InputInterface<double> force_length_l_;
    InputInterface<double> force_length_r_;
    InputInterface<double> torque_alpha_l_;
    InputInterface<double> torque_alpha_r_;
    InputInterface<double> leg_length_l_;
    InputInterface<double> leg_length_r_;

    OutputInterface<double> hip_torque_l_;
    OutputInterface<double> crank_torque_l_;
    OutputInterface<double> hip_torque_r_;
    OutputInterface<double> crank_torque_r_;
};

} // namespace rmcs_core::controller::wheelleg

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::wheelleg::WheelLegTaskMapper, rmcs_executor::Component)
