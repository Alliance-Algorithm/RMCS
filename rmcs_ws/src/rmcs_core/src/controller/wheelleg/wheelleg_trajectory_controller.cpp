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
        // Outer d-loop correction: a bias added to the hip (leg-angle) reference,
        // used to shift the CoM offset d toward zero (ankle strategy).
        register_input("/wheelleg/task/alpha_bias", alpha_bias_, false);
        // A5: acceleration-mode command (m/s^2).
        register_input("/wheelleg/command/acceleration", acceleration_command_, false);
        // Body-frame forward speed/distance for the posture-based outer loop.
        register_input("/wheelleg/state/velocity", state_velocity_);
        register_input("/wheelleg/state/distance", state_distance_);
        // Second leg DOF (leg orientation): the shank angle phi. When a producer
        // publishes it, the pose is taken from solve_pose_(L0, phi) instead of
        // the single-DOF "CoM over axle" solve (see update()).
        register_input("/wheelleg/task/shank", shank_command_, false);
        // Recovery leg length (second height source), blended by alpha_r.
        register_input("/wheelleg/task/leg_length", leg_length_recovery_, false);
        // Yaw-rate command, for the turning posture gate.
        register_input("/wheelleg/command/yaw_rate", yaw_rate_command_, false);
        // Task layer (step 2): alpha_r blend (1 = balance, 0 = recovery) set by
        // the recovery source, and the wheel-contact gate.
        register_input("/wheelleg/task/recovery", recovery_command_, false);
        register_input("/wheelleg/state/contact_normal_l", contact_normal_l_, false);
        register_input("/wheelleg/state/contact_normal_r", contact_normal_r_, false);

        register_output("/wheelleg/reference/wheel_velocity", reference_wheel_velocity_, kNaN);
        register_output("/wheelleg/reference/pitch", reference_pitch_, kNaN);
        register_output("/wheelleg/reference/shank_angle", reference_shank_angle_, kNaN);
        register_output("/wheelleg/reference/knee_angle", reference_knee_angle_, kNaN);
        register_output("/wheelleg/reference/hip_angle", reference_hip_angle_, kNaN);
        // Wheel-balance enable: false while recovering (alpha_r = 0) so the
        // balance ADRC does not drive/park the wheels before they are loaded.
        register_output("/wheelleg/balance/enable", balance_enable_, true);
    }

    void update() override {
        State state;
        if (!read_state_(state)) {
            disable_();
            return;
        }

        // Quasi-static gravity feedforward is handled by the leg PD in the
        // bridge; the reduced length channel has no separate feedforward here.

        const double velocity_command_raw =
            command_velocity_.ready() && std::isfinite(*command_velocity_)
                ? *command_velocity_
                : default_velocity_command_;

        // S-curve velocity planner: jerk-limited, acceleration-limited tracking
        // of the raw command. Produces a continuous reference velocity
        // (velocity_ref_) and acceleration (accel_ref_/a_planned), so a step
        // change in the command (e.g. a +/- reversal) becomes a smooth S-profile
        // instead of an instantaneous velocity/acceleration jump. The posture
        // (gait) is driven by this real-time planned acceleration.
        if (!std::isfinite(velocity_ref_)) {
            // Start from rest so the first command is also realised as an
            // S-profile (velocity_ref_ follows via the planner, not a jump).
            velocity_ref_ = 0.0;
            accel_ref_ = 0.0;
        }
        const double a_des = std::clamp(
            command_vloop_kp_ * (velocity_command_raw - velocity_ref_)
                - command_vloop_kd_ * accel_ref_,
            -command_accel_limit_, command_accel_limit_);
        accel_ref_ +=
            std::clamp(a_des - accel_ref_, -command_jerk_limit_ * dt_, command_jerk_limit_ * dt_);
        velocity_ref_ += accel_ref_ * dt_;

        const double velocity_command = velocity_command_raw;
        const double velocity_command_ref = velocity_ref_;
        const double a_planned = accel_ref_;

        // Body-forward distance reference (integral of the commanded speed) for
        // the posture-based outer loop.
        body_distance_reference_ += velocity_command_ref * dt_;

        const double a_explicit =
            (acceleration_command_.ready() && std::isfinite(*acceleration_command_))
                ? *acceleration_command_ : 0.0;

        double leg_length_command = kNaN;
        if (command_leg_length_.ready() && std::isfinite(*command_leg_length_))
            leg_length_command = *command_leg_length_;
        double base_leg_length =
            std::isfinite(leg_length_command) ? leg_length_command : nominal_leg_length_;

        // Posture gate: optionally soften the gait while turning. The old claim
        // that body-frame speed/distance is polluted by omega x offset was not
        // reproducible (CoM-based and wheel-odometry forward speeds agree under
        // yaw), so the gate is disabled by default. Set posture_gate_enabled=true
        // to restore it.
        const double yaw_rate_cmd =
            (yaw_rate_command_.ready() && std::isfinite(*yaw_rate_command_))
                ? *yaw_rate_command_ : 0.0;
        const double yaw_norm = yaw_rate_cmd / std::max(posture_yaw_scale_, kEpsilon);
        const double posture_gate =
            posture_gate_enabled_ ? 1.0 / (1.0 + yaw_norm * yaw_norm) : 1.0;

        // Outer speed/position loop -> desired acceleration, realised as a
        // posture command on the equivalent leg (CoM offset d). This replaces
        // the previous wheel-torque outer loop: the balance ring now owns the
        // common wheel torque, so forward motion is commanded by posture and
        // does not compete with the balance loop for wheel torque. The outer
        // action is gated off while turning (see posture_gate).
        const double v_meas =
            (state_velocity_.ready() && std::isfinite(*state_velocity_)) ? *state_velocity_ : 0.0;
        const double s_meas =
            (state_distance_.ready() && std::isfinite(*state_distance_)) ? *state_distance_ : 0.0;
        const double a_outer = posture_gate
                             * (outer_kv_ * (velocity_command_ref - v_meas)
                                + outer_kx_ * (body_distance_reference_ - s_meas));

        // Planned (S-curve feedforward) + explicit command + gated outer correction.
        const double a_cmd = a_explicit + a_planned + a_outer;

        // Optional coupling of the leg length with forward speed.
        leg_length_command =
            std::clamp(base_leg_length - leg_motion_coupling_ * velocity_command, 0.22, 0.45);

        // ---- Task layer (step 2) ---------------------------------------------
        // The leg has two DOF: length L0 (the height task, above) and orientation
        // (the shank angle phi). The orientation is produced by blending two
        // tasks:
        //   * balance  : phi from the static "CoM over the axle" solve, i.e.
        //                d -> d_ref = a*h/g; only meaningful while the wheels
        //                carry the body (they are the only pitch authority);
        //   * recovery : an externally supplied leg pose (/wheelleg/task/shank),
        //                used while the ground/chassis carries the body and the
        //                wheels have no authority.
        // `/wheelleg/task/recovery` is the blend factor alpha_r (1 = balance,
        // 0 = recovery); it is driven by the recovery source or a hand command.
        const double rec = (recovery_command_.ready() && std::isfinite(*recovery_command_))
                             ? std::clamp(*recovery_command_, 0.0, 1.0)
                             : 1.0;
        // Height task: blend the planner's leg length with the recovery leg
        // length by alpha_r, so the recovery source drives L0 during get-up too.
        const double recovery_length =
            (leg_length_recovery_.ready() && std::isfinite(*leg_length_recovery_))
                ? *leg_length_recovery_ : leg_length_command;
        leg_length_command = std::clamp(
            rec * leg_length_command + (1.0 - rec) * recovery_length, 0.22, 0.45);

        const Target balance_target = solve_target_(velocity_command, leg_length_command);
        const double recovery_shank =
            (shank_command_.ready() && std::isfinite(*shank_command_)) ? *shank_command_
                                                                       : balance_target.shank;
        const double shank_target = rec * balance_target.shank + (1.0 - rec) * recovery_shank;
        // Remember the blend and the measured pitch for the pitch-reference gate
        // in publish_reference_ (hold the measured pitch during recovery so the
        // balance ring does not fight while the wheels have no authority).
        recovery_blend_ = rec;
        last_pitch_ = state.pitch;

        // 2-DOF pose: length from the height task, orientation from the fused
        // orientation task (this is where solve_pose_ replaces the single-DOF
        // solve with a genuinely 2-DOF map).
        Target target = solve_pose_(velocity_command, leg_length_command, shank_target);

        // Posture (gait): d_ref = a*h/g realised as an equivalent-leg
        // inclination offset (leaning the whole leg), which moves the CoM offset
        // d efficiently without crouching. Faded out by alpha_r so the
        // acceleration posture does not fight a recovery pose. The gait is
        // measured relative to the balance target, then added on top of the
        // fused pose.
        gait_dknee_ = 0.0;
        gait_dshank_ = 0.0;
        gait_dhip_ = 0.0;
        {
            const double d_ref = a_cmd * com_height_ / gravity_;
            const double d_err = d_ref - state.com_offset;
            gait_dhip_ = rec * posture_gate * (accel_theta_ff_ * d_ref + accel_kv_ * d_err);
            const double gait_dl =
                rec * posture_gate * (accel_leg_ff_ * d_ref + accel_d_kp_ * d_err);
            const auto gait_target = solve_target_(
                velocity_command, std::clamp(leg_length_command + gait_dl, 0.22, 0.45));
            if (std::isfinite(gait_target.beta) && std::isfinite(balance_target.beta)
                && std::isfinite(gait_target.shank) && std::isfinite(balance_target.shank)) {
                gait_dknee_ = gait_target.beta - balance_target.beta;
                gait_dshank_ = gait_target.shank - balance_target.shank;
            }
        }
        // (the gait offsets are added in publish_reference_, as before, so they
        // are not folded into `target` here.)

        // Extended gating: the static-stability guard applies only in the balance
        // regime (alpha_r = 1) AND while the wheels carry the body. During
        // recovery, or while the wheels are unloaded, the guard is off so the leg
        // is free to move (a fallen start has a large CoM offset and would
        // otherwise freeze the plan immediately). If no contact source is wired,
        // treat the wheels as loaded so the previous behaviour is preserved.
        const bool contact_available =
            contact_normal_l_.ready() || contact_normal_r_.ready();
        const bool supported = !contact_available || state.contact_l > contact_normal_eps_
                            || state.contact_r > contact_normal_eps_;
        const bool guard = rec >= 1.0 && supported && std::abs(state.com_offset) > d_max_;
        if (guard) {
            target.leg_length = state.leg_length;
            target.feasible = false;
        }

        // Balance ADRC enable: ON when the wheels carry the body AND the robot is
        // near upright (the balanceable regime) -- this includes the standing hold.
        // OFF while the wheels are airborne / the body is tumbling (the get-up),
        // where the balance has no authority and its ESO would just spin the
        // wheels.
        *balance_enable_ = supported && std::abs(state.pitch) < balance_enable_tilt_;

        // Restart the joint plan only when the commanded *equilibrium* changes
        // (leg length). The joint targets (beta/shank/hip) do not depend on the
        // velocity command, and the wheel-speed reference follows the S-curve
        // velocity_ref_, so a continuously varying velocity (e.g. a smoothed
        // motion-planner command) must NOT restart the plan: restarting resets
        // elapsed_ to 0 every step, which freezes the pitch reference at the
        // measured pitch and disables the pitch balance loop.
        const bool command_changed =
            !initialized_
            || (!guard && std::abs(target.leg_length - planned_leg_length_) > leg_length_epsilon_);

        if (command_changed) {
            start_plan_(state, target);
            planned_velocity_ = velocity_command;
            planned_leg_length_ = target.leg_length;
            elapsed_ = 0.0;
            initialized_ = true;
        } else {
            elapsed_ += dt_;
        }

        publish_reference_(target);

        // Debug: first few ticks show the task blend and the pitch setpoint.
        if (dbg_task_count_++ < 10)
            info(
                "task rec={:.3f} L0cmd={:.3f} shank={:.3f} blend={:.3f} pitch_ref={:.4f}",
                rec, leg_length_command, shank_target, recovery_blend_, *reference_pitch_);
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
        // Wheel-ground normal force per wheel (contact gate); 0 when absent.
        double contact_l = 0.0;
        double contact_r = 0.0;
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
        transition_time_shank_ = node::param_or("transition_time_shank", 1.5);
        transition_time_knee_ = node::param_or("transition_time_knee", 1.5);

        // Signed knee angle: choose the assembly branch ("knee forward" +1 /
        // "knee backward" -1), see 执行路线1_数学解析.md §6.3/§6.6.
        knee_branch_sign_ = node::param_or("knee_branch_sign", -1.0);
        knee_angle_min_ = node::param_or("knee_angle_min", -std::numbers::pi);
        knee_angle_max_ = node::param_or("knee_angle_max", std::numbers::pi);
        singularity_epsilon_ = node::param_or("singularity_epsilon", 1e-3);

        leg_length_epsilon_ = node::param_or("leg_length_epsilon", 1e-4);
        default_velocity_command_ = node::param_or("default_velocity_command", 0.0);
        command_accel_limit_ = node::param_or("command_accel_limit", 1.0);
        command_jerk_limit_ = node::param_or("command_jerk_limit", 4.0);
        command_vloop_kp_ = node::param_or("command_vloop_kp", 10.0);
        command_vloop_kd_ = node::param_or("command_vloop_kd", 2.0);

        leg_motion_coupling_ = node::param_or("leg_motion_coupling", 0.0);
        outer_kx_ = node::param_or("outer_kx", 0.0);
        outer_kv_ = node::param_or("outer_kv", 0.0);
        // Turning gate scale (rad/s): below it the posture/gait is fully active,
        // above it the gait is disabled so yaw does not deform the leg.
        posture_yaw_scale_ = node::param_or("posture_yaw_scale", 0.15);
        posture_gate_enabled_ = node::param_or("posture_gate_enabled", false);
        // Acceleration mode (gated by the acceleration command): CoM-offset
        // feedforward d* = a*h/g realised via the leg length.
        com_height_ = node::param_or("com_height", 0.26);
        accel_leg_ff_ = node::param_or("accel_leg_ff", 0.0);
        accel_d_kp_ = node::param_or("accel_d_kp", 0.0);
        // d-loop: pitch feedforward per unit CoM offset (posture map: d ~ 0.26
        // per rad of pitch => ~3.85 rad per m).
        accel_theta_ff_ = node::param_or("accel_theta_ff", 3.85);
        accel_kv_ = node::param_or("accel_kv", 1.0);
        d_max_ = node::param_or("d_max", 0.03);
        // Wheel normal force above which the wheel is treated as loaded (contact
        // gate for the static-stability guard / balance regime).
        contact_normal_eps_ = node::param_or("contact_normal_eps", 1.0);
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
        state.contact_l =
            (contact_normal_l_.ready() && std::isfinite(*contact_normal_l_)) ? *contact_normal_l_
                                                                             : 0.0;
        state.contact_r =
            (contact_normal_r_.ready() && std::isfinite(*contact_normal_r_)) ? *contact_normal_r_
                                                                             : 0.0;

        return std::isfinite(state.wheel_velocity) && std::isfinite(state.shank)
            && std::isfinite(state.shank_velocity) && std::isfinite(state.knee)
            && std::isfinite(state.knee_velocity) && std::isfinite(state.pitch)
            && std::isfinite(state.pitch_velocity) && std::isfinite(state.com_offset)
            && std::isfinite(state.leg_length);
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

    // Step 1 of the solver extension: a 2-DOF pose map.
    //
    // The single-DOF solve above fixes the leg ORIENTATION by the hard
    // "CoM over the wheel axle" condition. That is only one point of the leg's
    // two-dimensional configuration space (执行路线1 §1: the independent leg
    // parameters are the length L0 and the hip-wheel line angle alpha_L), and it
    // is only meaningful while the wheels carry the body.
    //
    // This variant takes the second DOF (the shank angle phi, which sets the leg
    // orientation for a given L0) as an INDEPENDENT input, so any pose in the
    // leg's travel is reachable. Conventions match solve_target_ exactly (beta
    // from L0, same branch sign, same hip = pi - beta - phi), so phi = balance
    // shank reproduces the current behaviour and L0 sweeping the reach covers the
    // whole workspace.
    [[nodiscard]] Target
        solve_pose_(double velocity_command, double leg_length, double shank) const {
        Target target;

        const double reach_min = std::abs(thigh_length_ - shank_length_);
        const double reach_max = thigh_length_ + shank_length_;
        const bool reachable = leg_length >= reach_min && leg_length <= reach_max;
        leg_length = std::clamp(leg_length, reach_min, reach_max);

        const double cos_beta = std::clamp(
            (thigh_length_ * thigh_length_ + shank_length_ * shank_length_
             - leg_length * leg_length)
                / (2.0 * thigh_length_ * shank_length_),
            -1.0, 1.0);
        double beta = knee_branch_sign_ * std::acos(cos_beta);
        const bool within_limits = beta >= knee_angle_min_ && beta <= knee_angle_max_;
        if (!within_limits) {
            beta = std::clamp(beta, knee_angle_min_, knee_angle_max_);
            leg_length = std::sqrt(
                thigh_length_ * thigh_length_ + shank_length_ * shank_length_
                - 2.0 * thigh_length_ * shank_length_ * std::cos(beta));
        }
        const bool nonsingular = std::abs(std::sin(beta)) > singularity_epsilon_;

        target.leg_length = leg_length;
        target.beta = beta;
        target.shank = shank;
        // 执行路线1 §1: alpha = pi - theta - beta - phi; the pitch (theta) is set
        // by the plan/publish stage, so the pose here is the theta = 0 reference.
        target.hip = std::numbers::pi - beta - shank;
        target.wheel_velocity = velocity_command / wheel_radius_;
        target.feasible = reachable && within_limits && nonsingular;
        return target;
    }

    void start_plan_(const State& state, const Target& target) {
        pitch_plan_.reset(state.pitch, state.pitch_velocity, 0.0, 0.0, transition_time_pitch_);
        shank_plan_.reset(
            state.shank, state.shank_velocity, target.shank, 0.0, transition_time_shank_);
        knee_plan_.reset(state.knee, state.knee_velocity, target.beta, 0.0, transition_time_knee_);
    }

    void publish_reference_(const Target& target) {
        const double pitch_reference = pitch_plan_.value(elapsed_);
        const double velocity_reference = std::isfinite(velocity_ref_) ? velocity_ref_ : 0.0;

        // Recovery: follow the recovery target DIRECTLY. It is already a smooth,
        // time-parameterised reference, and re-planning it through the quintic
        // would reset the plan every tick (the target moves continuously), which
        // made the leg chatter and never track the recovery pose. The hip uses
        // the measured body pitch during recovery so the leg pose is referenced
        // to the real attitude, not to a planned 0.
        const bool recovering = recovery_blend_ < 1.0 - 1e-6;
        double shank_reference = 0.0, shank_rate = 0.0, knee_reference = 0.0, knee_rate = 0.0;
        if (recovering) {
            shank_reference = target.shank;
            knee_reference = target.beta;
        } else {
            shank_reference = shank_plan_.value(elapsed_);
            shank_rate = shank_plan_.derivative(elapsed_);
            knee_reference = knee_plan_.value(elapsed_);
            knee_rate = knee_plan_.derivative(elapsed_);
        }

        // Leg-motion correction, 执行路线1_数学解析.md §7.5:
        //   x_e = L1 cos(beta+phi) - L2 cos(phi)
        //   x_e_dot = (-L1 sin(beta+phi) + L2 sin(phi)) phi_dot - L1 sin(beta+phi) beta_dot
        //   omega_ref = (v_ref + x_e_dot) / r
        const double thigh_projection = thigh_length_ * std::sin(knee_reference + shank_reference);
        const double shank_dx =
            (-thigh_projection + shank_length_ * std::sin(shank_reference)) * shank_rate
            - thigh_projection * knee_rate;

        *reference_wheel_velocity_ = (velocity_reference + shank_dx) / wheel_radius_;
        // Pitch setpoint: the planned value (-> 0) in the balance regime, the
        // measured pitch during recovery (alpha_r = 0), so the balance ring is
        // not fed a setpoint it cannot achieve while the wheels are unloaded.
        *reference_pitch_ = recovery_blend_ * pitch_reference + (1.0 - recovery_blend_) * last_pitch_;
        // Acceleration posture: direct additive offset on the leg joints.
        const double shank_published = shank_reference + gait_dshank_;
        const double knee_published = knee_reference + gait_dknee_;
        *reference_shank_angle_ = shank_published;
        *reference_knee_angle_ = knee_published;
        // §7.4: alpha_ref = pi - theta - beta_ref - phi_ref (+ outer d-loop bias).
        // During recovery the leg must be referenced to the measured attitude.
        const double hip_pitch = recovering ? last_pitch_ : pitch_reference;
        const double alpha_bias =
            (alpha_bias_.ready() && std::isfinite(*alpha_bias_)) ? *alpha_bias_ : 0.0;
        *reference_hip_angle_ = std::numbers::pi - hip_pitch - knee_published - shank_published
                              + alpha_bias + gait_dhip_;

        if (dbg_count_++ % 500 == 0)
            info(
                "traj tgt L0={:.4f} beta={:.3f} shank={:.3f} hip={:.3f} | ref knee={:.3f} "
                "hip={:.3f} shank={:.3f} | ref_wv={:.3f} state_wv={:.3f}",
                target.leg_length, target.beta, target.shank, target.hip, knee_reference,
                std::numbers::pi - hip_pitch - knee_reference - shank_reference,
                shank_reference, *reference_wheel_velocity_,
                (state_wheel_velocity_.ready() ? *state_wheel_velocity_ : 0.0));
    }

    void disable_() {
        initialized_ = false;
        elapsed_ = 0.0;
        planned_velocity_ = 0.0;
        planned_leg_length_ = kNaN;
        velocity_ref_ = kNaN;
        accel_ref_ = 0.0;
        gait_dknee_ = 0.0;
        gait_dshank_ = 0.0;
        gait_dhip_ = 0.0;

        body_distance_reference_ = 0.0;
        *reference_wheel_velocity_ = kNaN;
        *reference_pitch_ = kNaN;
        *reference_shank_angle_ = kNaN;
        *reference_knee_angle_ = kNaN;
        *reference_hip_angle_ = kNaN;
        *balance_enable_ = true;
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
    double transition_time_shank_;
    double transition_time_knee_;

    double knee_branch_sign_;
    double knee_angle_min_;
    double knee_angle_max_;
    double singularity_epsilon_;

    double leg_length_epsilon_;
    double default_velocity_command_;
    double command_accel_limit_ = 1.0;
    double command_jerk_limit_ = 4.0;
    double command_vloop_kp_ = 10.0;
    double command_vloop_kd_ = 2.0;
    double velocity_ref_ = kNaN;
    double accel_ref_ = 0.0;
    double gait_dknee_ = 0.0;
    double gait_dshank_ = 0.0;
    double gait_dhip_ = 0.0;
    // Recovery blend (alpha_r) and the measured pitch, for the pitch-reference
    // gate in publish_reference_.
    double recovery_blend_ = 1.0;
    double last_pitch_ = 0.0;
    double leg_motion_coupling_;
    double outer_kx_;
    double outer_kv_;
    double com_height_;
    double accel_leg_ff_;
    double accel_d_kp_;
    double accel_theta_ff_;
    double accel_kv_;
    double d_max_;
    double contact_normal_eps_ = 1.0;
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
    InputInterface<double> alpha_bias_;
    InputInterface<double> shank_command_;
    InputInterface<double> leg_length_recovery_;
    InputInterface<double> recovery_command_;
    InputInterface<double> contact_normal_l_;
    InputInterface<double> contact_normal_r_;
    InputInterface<double> acceleration_command_;
    InputInterface<double> state_velocity_;
    InputInterface<double> state_distance_;
    InputInterface<double> yaw_rate_command_;

    OutputInterface<double> reference_wheel_velocity_;
    OutputInterface<double> reference_pitch_;
    OutputInterface<double> reference_shank_angle_;
    OutputInterface<double> reference_knee_angle_;
    OutputInterface<double> reference_hip_angle_;
    OutputInterface<bool> balance_enable_;
    double body_distance_reference_ = 0.0;
    double posture_yaw_scale_ = 0.15;
    bool posture_gate_enabled_ = false;

    Quintic pitch_plan_;
    Quintic shank_plan_;
    Quintic knee_plan_;

    std::uint64_t dbg_count_ = 0;
    std::uint64_t dbg_task_count_ = 0;
    bool initialized_ = false;
    double elapsed_ = 0.0;
    double planned_velocity_ = 0.0;
    double planned_leg_length_ = kNaN;
};

} // namespace rmcs_core::controller::wheelleg

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::wheelleg::WheelLegTrajectoryController, rmcs_executor::Component)
