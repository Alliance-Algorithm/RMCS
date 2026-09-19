#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

namespace rmcs_core::controller::wheelleg {

// Trajectory generator for the degraded serial wheel-leg model, see
// 执行路线1_数学解析.md. It solves the command equilibrium S* and emits C^2
// reference curves from the current state S0 to S*, then maps the attitude
// space S_A to the joint space S_B (hip/knee/wheel references).
class WheelLegTrajectoryController
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegTrajectoryController()
        : Node{get_component_name(), node::options()} {
        load_config_();

        register_input("/wheelleg/command/velocity", command_velocity_, false);
        register_input("/wheelleg/command/leg_length", command_leg_length_, false);

        register_input("/wheelleg/state/wheel_velocity", state_wheel_velocity_);
        register_input("/wheelleg/state/shank_angle", state_shank_angle_);
        register_input("/wheelleg/state/shank_velocity", state_shank_velocity_);
        register_input("/wheelleg/state/knee_angle", state_knee_angle_);
        register_input("/wheelleg/state/knee_velocity", state_knee_velocity_);
        register_input("/wheelleg/state/pitch", state_pitch_);
        register_input("/wheelleg/state/pitch_velocity", state_pitch_velocity_);
        register_input("/wheelleg/state/com_offset", state_com_offset_);
        register_input("/wheelleg/state/leg_length", state_leg_length_);
        register_input("/wheelleg/state/leg_angle", state_leg_angle_);
        // Outer d-loop correction: a bias added to the hip (leg-angle) reference,
        // used to shift the CoM offset d toward zero (ankle strategy).
        register_input("/wheelleg/task/alpha_bias", alpha_bias_, false);
        // A5: acceleration-mode command (m/s^2) + base position for tracking.
        register_input("/wheelleg/command/acceleration", acceleration_command_, false);
        register_input("/wheelleg/state/com_x", state_com_x_);

        register_output("/wheelleg/reference/wheel_velocity", reference_wheel_velocity_, kNaN);
        register_output("/wheelleg/reference/pitch", reference_pitch_, kNaN);
        register_output("/wheelleg/reference/shank_angle", reference_shank_angle_, kNaN);
        register_output("/wheelleg/reference/knee_angle", reference_knee_angle_, kNaN);
        register_output("/wheelleg/reference/hip_angle", reference_hip_angle_, kNaN);
        // Base forward-position reference (integral of the velocity command), for
        // the outer position loop to track a commanded forward speed.
        register_output("/wheelleg/reference/com_x", reference_com_x_, kNaN);

        register_output("/wheelleg/target/leg_length", target_leg_length_, kNaN);
        register_output("/wheelleg/target/shank_angle", target_shank_angle_, kNaN);
        register_output("/wheelleg/target/knee_angle", target_knee_angle_, kNaN);
        register_output("/wheelleg/target/hip_angle", target_hip_angle_, kNaN);
        register_output("/wheelleg/status/feasible", status_feasible_, false);
        register_output("/wheelleg/task/force_length_ff", force_length_ff_, kNaN);
    }

    void update() override {
        State state;
        if (!read_state_(state)) {
            disable_();
            return;
        }

        // Quasi-static gravity feedforward for the equivalent-leg length,
        // shared by both legs: F_L,ff = (M g sin alpha_L) / 2. Consumed by the
        // length ADRC so its ESO only trims the residual.
        *force_length_ff_ = 0.5 * total_mass_ * gravity_ * std::sin(state.leg_angle);
        measured_pitch_ = state.pitch;

        const double velocity_command =
            command_velocity_.ready() && std::isfinite(*command_velocity_)
                ? *command_velocity_
                : default_velocity_command_;

        double leg_length_command = kNaN;
        if (command_leg_length_.ready() && std::isfinite(*command_leg_length_))
            leg_length_command = *command_leg_length_;

        // Coordinated motion: shorten the leg with forward speed so the CoM
        // stays near the wheel axle (keeps the wheel torque inside the friction
        // limit while accelerating; see the posture map).
        const double base_leg_length =
            std::isfinite(leg_length_command) ? leg_length_command : nominal_leg_length_;
        leg_length_command =
            std::clamp(base_leg_length - leg_motion_coupling_ * velocity_command, 0.22, 0.45);

        auto target = solve_target_(velocity_command, leg_length_command);

        // Static-stability guard: while the CoM offset d exceeds d_max, hold the
        // leg length at its current value and freeze the plan restart, so the
        // height channel does not keep feeding the CoM drift.
        const bool guard = std::abs(state.com_offset) > d_max_;
        if (guard) {
            target.leg_length = state.leg_length;
            target.feasible = false;
        }

        const bool command_changed =
            !initialized_
            || (!guard
                && (std::abs(velocity_command - planned_velocity_) > velocity_epsilon_
                    || std::abs(target.leg_length - planned_leg_length_) > leg_length_epsilon_));

        if (command_changed) {
            start_plan_(state, target, velocity_command);
            planned_velocity_ = velocity_command;
            planned_leg_length_ = target.leg_length;
            elapsed_ = 0.0;
            initialized_ = true;
        } else {
            elapsed_ += dt_;
        }

        // Outer motion loop: while moving, the position reference follows the
        // actual position (position loop neutral) and the pitch setpoint is
        // leaned by the velocity error (and the commanded acceleration).
        const double a_cmd =
            (acceleration_command_.ready() && std::isfinite(*acceleration_command_))
                ? *acceleration_command_ : 0.0;
        const bool moving = std::abs(velocity_command) > 1e-6 || std::abs(a_cmd) > 1e-6;
        if (moving && state_com_x_.ready() && std::isfinite(*state_com_x_))
            com_x_reference_ = *state_com_x_;
        const double v_ref_rad = velocity_command / wheel_radius_;
        const double v_now =
            (state_wheel_velocity_.ready() && std::isfinite(*state_wheel_velocity_))
                ? *state_wheel_velocity_ : 0.0;
        velocity_pitch_bias_ = std::clamp(
            -velocity_pitch_kp_ * (v_now - v_ref_rad) + accel_to_pitch_ * a_cmd,
            -accel_pitch_limit_, accel_pitch_limit_);
        *reference_com_x_ = com_x_reference_;

        publish_reference_(target);
    }

private:
    struct State {
        double wheel_velocity = kNaN;
        double shank = kNaN;
        double shank_velocity = kNaN;
        double knee = kNaN;
        double knee_velocity = kNaN;
        double pitch = kNaN;
        double pitch_velocity = kNaN;
        double com_offset = kNaN;
        double leg_length = kNaN;
        double leg_angle = kNaN;
    };

    struct Target {
        double leg_length = kNaN;
        double beta = kNaN;
        double shank = kNaN;
        double hip = kNaN;
        double wheel_velocity = kNaN;
        bool feasible = false;
    };

    // Quintic curve with matched position/velocity at both ends and zero
    // acceleration at both ends, 执行路线1_数学解析.md §7.2.
    struct Quintic {
        void reset(double y0, double dy0, double y1, double dy1, double duration) {
            duration_ = std::max(duration, kEpsilon);
            a0_ = y0;
            a1_ = duration_ * dy0;

            const double delta = y1 - y0;
            a3_ = 10.0 * delta - 6.0 * duration_ * dy0 - 4.0 * duration_ * dy1;
            a4_ = -15.0 * delta + 8.0 * duration_ * dy0 + 7.0 * duration_ * dy1;
            a5_ = 6.0 * delta - 3.0 * duration_ * dy0 - 3.0 * duration_ * dy1;
        }

        [[nodiscard]] double value(double t) const {
            const double u = std::clamp(t / duration_, 0.0, 1.0);
            const double u2 = u * u;
            const double u3 = u2 * u;
            const double u4 = u3 * u;
            const double u5 = u4 * u;
            return a0_ + a1_ * u + a3_ * u3 + a4_ * u4 + a5_ * u5;
        }

        [[nodiscard]] double derivative(double t) const {
            const double u = std::clamp(t / duration_, 0.0, 1.0);
            const double u2 = u * u;
            const double u3 = u2 * u;
            const double u4 = u3 * u;
            return (a1_ + 3.0 * a3_ * u2 + 4.0 * a4_ * u3 + 5.0 * a5_ * u4) / duration_;
        }

    private:
        double a0_ = 0.0, a1_ = 0.0, a3_ = 0.0, a4_ = 0.0, a5_ = 0.0;
        double duration_ = 1.0;
    };

    void load_config_() {
        dt_ = node::param_or("dt", 0.001);

        thigh_length_ = node::param_or("thigh_length", 0.2);
        shank_length_ = node::param_or("shank_length", 0.2);
        wheel_radius_ = node::param_or("wheel_radius", 0.06);
        nominal_leg_length_ = node::param_or("nominal_leg_length", 0.35);

        body_mass_ = node::param_or("body_mass", 3.0);
        thigh_mass_ = node::param_or("thigh_mass", 0.5);
        shank_mass_ = node::param_or("shank_mass", 0.5);
        thigh_com_offset_ = node::param_or("thigh_com_offset", 0.1);
        shank_com_offset_ = node::param_or("shank_com_offset", 0.1);
        body_com_offset_ = node::param_or("body_com_offset", 0.0);

        transition_time_pitch_ = node::param_or("transition_time_pitch", 0.15);
        transition_time_velocity_ = node::param_or("transition_time_velocity", 1.5);
        transition_time_shank_ = node::param_or("transition_time_shank", 1.5);
        transition_time_knee_ = node::param_or("transition_time_knee", 1.5);

        // Signed knee angle: choose the assembly branch ("knee forward" +1 /
        // "knee backward" -1), see 执行路线1_数学解析.md §6.3/§6.6.
        knee_branch_sign_ = node::param_or("knee_branch_sign", -1.0);
        knee_angle_min_ = node::param_or("knee_angle_min", -std::numbers::pi);
        knee_angle_max_ = node::param_or("knee_angle_max", std::numbers::pi);
        singularity_epsilon_ = node::param_or("singularity_epsilon", 1e-3);

        velocity_epsilon_ = node::param_or("velocity_epsilon", 1e-3);
        leg_length_epsilon_ = node::param_or("leg_length_epsilon", 1e-4);
        default_velocity_command_ = node::param_or("default_velocity_command", 0.0);

        // A5 velocity outer loop: lean the pitch setpoint by the wheel-speed error.
        velocity_pitch_kp_ = node::param_or("velocity_pitch_kp", 0.0);
        velocity_pitch_limit_ = node::param_or("velocity_pitch_limit", 0.3);
        // A5 acceleration mode: lean per unit commanded acceleration.
        accel_to_pitch_ = node::param_or("accel_to_pitch", 0.1);
        accel_pitch_limit_ = node::param_or("accel_pitch_limit", 0.25);
        leg_motion_coupling_ = node::param_or("leg_motion_coupling", 0.0);
        d_max_ = node::param_or("d_max", 0.03);
        total_mass_ = node::param_or("total_mass", 13.702);
        gravity_ = node::param_or("gravity", 9.81);
    }

    bool read_state_(State& state) const {
        state.wheel_velocity = *state_wheel_velocity_;
        state.shank = *state_shank_angle_;
        state.shank_velocity = *state_shank_velocity_;
        state.knee = *state_knee_angle_;
        state.knee_velocity = *state_knee_velocity_;
        state.pitch = *state_pitch_;
        state.pitch_velocity = *state_pitch_velocity_;
        state.com_offset = *state_com_offset_;
        state.leg_length = *state_leg_length_;
        state.leg_angle = *state_leg_angle_;

        return std::isfinite(state.wheel_velocity) && std::isfinite(state.shank)
            && std::isfinite(state.shank_velocity) && std::isfinite(state.knee)
            && std::isfinite(state.knee_velocity) && std::isfinite(state.pitch)
            && std::isfinite(state.pitch_velocity) && std::isfinite(state.com_offset)
            && std::isfinite(state.leg_length) && std::isfinite(state.leg_angle);
    }

    // 执行路线1_数学解析.md §6: command equilibrium S*.
    [[nodiscard]] Target solve_target_(double velocity_command, double leg_length_command) const {
        Target target;

        double leg_length =
            std::isfinite(leg_length_command) ? leg_length_command : nominal_leg_length_;

        const double reach_min = std::abs(thigh_length_ - shank_length_);
        const double reach_max = thigh_length_ + shank_length_;
        const bool reachable = leg_length >= reach_min && leg_length <= reach_max;
        leg_length = std::clamp(leg_length, reach_min, reach_max);

        // §6.3: h_leg* = L0.
        const double cos_beta = std::clamp(
            (thigh_length_ * thigh_length_ + shank_length_ * shank_length_
             - leg_length * leg_length)
                / (2.0 * thigh_length_ * shank_length_),
            -1.0, 1.0);

        double beta = knee_branch_sign_ * std::acos(cos_beta);
        bool within_limits = beta >= knee_angle_min_ && beta <= knee_angle_max_;
        if (!within_limits) {
            beta = std::clamp(beta, knee_angle_min_, knee_angle_max_);
            leg_length = std::sqrt(
                thigh_length_ * thigh_length_ + shank_length_ * shank_length_
                - 2.0 * thigh_length_ * shank_length_ * std::cos(beta));
        }

        // Static balance coefficients, 执行路线1_数学解析.md §6.2.
        const double a1 =
            body_mass_ * thigh_length_ + thigh_mass_ * (thigh_length_ - thigh_com_offset_);
        const double a2 = (body_mass_ + thigh_mass_) * shank_length_
                        + shank_mass_ * (shank_length_ - shank_com_offset_);

        // §6.4: P sin(phi) + Q cos(phi) = m_b c_b, Gamma = sqrt(P^2 + Q^2).
        const double p = -a1 * std::sin(beta);
        const double q = a1 * std::cos(beta) - a2;
        const double gamma = std::sqrt(p * p + q * q);

        double shank = kNaN;
        if (gamma > kEpsilon)
            shank = std::asin(std::clamp(body_mass_ * body_com_offset_ / gamma, -1.0, 1.0))
                  - std::atan2(q, p);

        const bool nonsingular = std::abs(std::sin(beta)) > singularity_epsilon_;

        target.leg_length = leg_length;
        target.beta = beta;
        target.shank = shank;
        // §6.5: alpha* = pi - beta* - phi*.
        target.hip = std::numbers::pi - beta - shank;
        target.wheel_velocity = velocity_command / wheel_radius_;
        target.feasible = reachable && within_limits && nonsingular;
        return target;
    }

    void start_plan_(const State& state, const Target& target, double velocity_command) {
        pitch_plan_.reset(state.pitch, state.pitch_velocity, 0.0, 0.0, transition_time_pitch_);
        shank_plan_.reset(
            state.shank, state.shank_velocity, target.shank, 0.0, transition_time_shank_);
        knee_plan_.reset(state.knee, state.knee_velocity, target.beta, 0.0, transition_time_knee_);

        const double velocity = state.wheel_velocity * wheel_radius_;
        velocity_plan_.reset(velocity, 0.0, velocity_command, 0.0, transition_time_velocity_);
    }

    void publish_reference_(const Target& target) {
        const double pitch_reference = pitch_plan_.value(elapsed_);
        const double shank_reference = shank_plan_.value(elapsed_);
        const double shank_rate = shank_plan_.derivative(elapsed_);
        const double knee_reference = knee_plan_.value(elapsed_);
        const double knee_rate = knee_plan_.derivative(elapsed_);
        const double velocity_reference = velocity_plan_.value(elapsed_);

        // Leg-motion correction, 执行路线1_数学解析.md §7.5:
        //   x_e = L1 cos(beta+phi) - L2 cos(phi)
        //   x_e_dot = (-L1 sin(beta+phi) + L2 sin(phi)) phi_dot - L1 sin(beta+phi) beta_dot
        //   omega_ref = (v_ref + x_e_dot) / r
        const double thigh_projection = thigh_length_ * std::sin(knee_reference + shank_reference);
        const double shank_dx =
            (-thigh_projection + shank_length_ * std::sin(shank_reference)) * shank_rate
            - thigh_projection * knee_rate;

        *reference_wheel_velocity_ = (velocity_reference + shank_dx) / wheel_radius_;
        *reference_pitch_ = pitch_reference + velocity_pitch_bias_;
        *reference_shank_angle_ = shank_reference;
        *reference_knee_angle_ = knee_reference;
        // §7.4: alpha_ref = pi - theta_ref - beta_ref - phi_ref (+ outer d-loop bias).
        const double alpha_bias =
            (alpha_bias_.ready() && std::isfinite(*alpha_bias_)) ? *alpha_bias_ : 0.0;
        *reference_hip_angle_ =
            std::numbers::pi - pitch_reference - knee_reference - shank_reference + alpha_bias;

        *target_leg_length_ = target.leg_length;
        *target_shank_angle_ = target.shank;
        *target_knee_angle_ = target.beta;
        *target_hip_angle_ = target.hip;
        *status_feasible_ = target.feasible;

        if (dbg_count_++ % 500 == 0)
            info(
                "traj tgt L0={:.4f} beta={:.3f} shank={:.3f} hip={:.3f} | ref knee={:.3f} "
                "hip={:.3f} shank={:.3f} | ref_wv={:.3f} state_wv={:.3f}",
                target.leg_length, target.beta, target.shank, target.hip, knee_reference,
                std::numbers::pi - pitch_reference - knee_reference - shank_reference,
                shank_reference, *reference_wheel_velocity_,
                (state_wheel_velocity_.ready() ? *state_wheel_velocity_ : 0.0));
    }

    void disable_() {
        initialized_ = false;
        elapsed_ = 0.0;
        planned_velocity_ = 0.0;
        planned_leg_length_ = kNaN;
        com_x_reference_ = 0.0;

        *reference_com_x_ = kNaN;
        *reference_wheel_velocity_ = kNaN;
        *reference_pitch_ = kNaN;
        *reference_shank_angle_ = kNaN;
        *reference_knee_angle_ = kNaN;
        *reference_hip_angle_ = kNaN;
        *status_feasible_ = false;
        *force_length_ff_ = kNaN;
    }

    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kEpsilon = 1e-9;

    double dt_;

    double thigh_length_;
    double shank_length_;
    double wheel_radius_;
    double nominal_leg_length_;

    double body_mass_;
    double thigh_mass_;
    double shank_mass_;
    double thigh_com_offset_;
    double shank_com_offset_;
    double body_com_offset_;

    double transition_time_pitch_;
    double transition_time_velocity_;
    double transition_time_shank_;
    double transition_time_knee_;

    double knee_branch_sign_;
    double knee_angle_min_;
    double knee_angle_max_;
    double singularity_epsilon_;

    double velocity_epsilon_;
    double leg_length_epsilon_;
    double default_velocity_command_;
    double velocity_pitch_kp_;
    double velocity_pitch_limit_;
    double velocity_pitch_bias_ = 0.0;
    double accel_to_pitch_;
    double accel_pitch_limit_;
    double leg_motion_coupling_;
    double d_max_;
    double total_mass_;
    double gravity_;

    InputInterface<double> command_velocity_;
    InputInterface<double> command_leg_length_;

    InputInterface<double> state_wheel_velocity_;
    InputInterface<double> state_shank_angle_;
    InputInterface<double> state_shank_velocity_;
    InputInterface<double> state_knee_angle_;
    InputInterface<double> state_knee_velocity_;
    InputInterface<double> state_pitch_;
    InputInterface<double> state_pitch_velocity_;
    InputInterface<double> state_com_offset_;
    InputInterface<double> state_leg_length_;
    InputInterface<double> state_leg_angle_;
    InputInterface<double> alpha_bias_;
    InputInterface<double> acceleration_command_;
    InputInterface<double> state_com_x_;

    OutputInterface<double> reference_wheel_velocity_;
    OutputInterface<double> reference_pitch_;
    OutputInterface<double> reference_shank_angle_;
    OutputInterface<double> reference_knee_angle_;
    OutputInterface<double> reference_hip_angle_;
    OutputInterface<double> reference_com_x_;
    double com_x_reference_ = 0.0;

    OutputInterface<double> target_leg_length_;
    OutputInterface<double> target_shank_angle_;
    OutputInterface<double> target_knee_angle_;
    OutputInterface<double> target_hip_angle_;
    OutputInterface<bool> status_feasible_;
    OutputInterface<double> force_length_ff_;

    Quintic pitch_plan_;
    Quintic shank_plan_;
    Quintic knee_plan_;
    Quintic velocity_plan_;

    std::uint64_t dbg_count_ = 0;
    double measured_pitch_ = 0.0;
    bool initialized_ = false;
    double elapsed_ = 0.0;
    double planned_velocity_ = 0.0;
    double planned_leg_length_ = kNaN;
};

} // namespace rmcs_core::controller::wheelleg

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::wheelleg::WheelLegTrajectoryController, rmcs_executor::Component)
