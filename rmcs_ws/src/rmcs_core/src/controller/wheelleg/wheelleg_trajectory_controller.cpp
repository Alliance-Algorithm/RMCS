#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "identification/wheel_leg_arm_dwell.hpp"

namespace rmcs_core::controller::wheelleg {

using ArmDwell = rmcs_core::controller::identification::WheelLegArmDwell;

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
        // Per-leg measured shape: when present, each side is planned from its own
        // current pose (no left/right averaging).
        register_input("/wheelleg/state/shank_angle_l", state_shank_angle_l_, false);
        register_input("/wheelleg/state/shank_angle_r", state_shank_angle_r_, false);
        register_input("/wheelleg/state/knee_angle_l", state_knee_angle_l_, false);
        register_input("/wheelleg/state/knee_angle_r", state_knee_angle_r_, false);
        register_input("/wheelleg/state/pitch", state_pitch_);
        register_input("/wheelleg/state/pitch_velocity", state_pitch_velocity_);
        register_input("/wheelleg/state/com_offset", state_com_offset_);
        register_input("/wheelleg/state/leg_length", state_leg_length_);
        register_input("/wheelleg/state/base_height", state_base_height_, false);
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

        // Drive-enable gate (all optional): the start-pose quintic must not run
        // while the motors are disabled. The simulation publishes /wheelleg/hold
        // during its start-pose hold; the real robot arms via the remote
        // switches (same ArmDwell logic as the drive bridge). While disabled the
        // reference is pinned to the measured pose and re-planned from it at the
        // enable edge, so "initial target = current pose, then smooth to the true
        // target" happens exactly when torque is released.
        register_input("/wheelleg/hold", hold_, false);
        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);
        register_input("/wheel_leg/dr16_fresh", dr16_fresh_, false);
        // Actual DM enable (both drives enabled + MIT sent + feedback fresh).
        // The arm edge only REQUESTs enable; the drive handshake takes ~100 ms
        // (100 resend cycles), so start the quintic on the real enable edge,
        // otherwise it would advance while the motors are still disabled and
        // the first applied torque would contain a target step.
        register_input("/wheel_leg/dm_control_ready", dm_control_ready_, false);
        register_output("/wheelleg/drive/enabled", drive_enabled_, true);
        // Wheel-balance enable: drive enabled AND the remote is in the
        // left-MIDDLE / right-UP combo (both-MIDDLE = legs only, wheels free).
        // Computed here (upstream of the LQR) so the LQR -> mixer -> drive-bridge
        // chain stays acyclic.
        register_output("/wheelleg/drive/balance_enabled", balance_enabled_, false);

        register_output("/wheelleg/reference/wheel_velocity", reference_wheel_velocity_, kNaN);
        register_output("/wheelleg/reference/pitch", reference_pitch_, kNaN);
        register_output("/wheelleg/reference/shank_angle", reference_shank_angle_, kNaN);
        register_output("/wheelleg/reference/knee_angle", reference_knee_angle_, kNaN);
        register_output("/wheelleg/reference/hip_angle", reference_hip_angle_, kNaN);
        // Per-leg references for the drive bridge (fallback = the averaged ones).
        register_output("/wheelleg/reference/shank_angle_l", reference_shank_angle_l_, kNaN);
        register_output("/wheelleg/reference/shank_angle_r", reference_shank_angle_r_, kNaN);
        register_output("/wheelleg/reference/knee_angle_l", reference_knee_angle_l_, kNaN);
        register_output("/wheelleg/reference/knee_angle_r", reference_knee_angle_r_, kNaN);
        register_output("/wheelleg/reference/hip_angle_l", reference_hip_angle_l_, kNaN);
        register_output("/wheelleg/reference/hip_angle_r", reference_hip_angle_r_, kNaN);
        // Wheel-balance enable: false while recovering (alpha_r = 0) so the
        // balance ADRC does not drive/park the wheels before they are loaded.
        register_output("/wheelleg/balance/enable", balance_enable_, true);
        // Base-velocity outer loop fed to the balance ADRC's feedforward input:
        //   ff = -kv * omega_wheel - kp * integral(omega_wheel)
        // i.e. wheel/base-velocity damping + a slow position term. The ADRC adds
        // it to its output and includes it in last_u_, so the ESO treats it as a
        // known input (no model mismatch). This is a real, measured-state
        // feedback, applied identically on sim and hardware.
        register_output("/wheelleg/balance/feedforward", balance_feedforward_, 0.0);
    }

    void update() override {
        State state;
        if (!read_state_(state)) {
            disable_();
            return;
        }

        // Drive-enable gate: while the motors are disabled (sim start-pose hold,
        // or not yet armed on the real robot) keep `initialized_ = false` so the
        // quintic is rebuilt from the MEASURED pose every tick (elapsed_ = 0 ->
        // reference == measurement). On the enable edge it is rebuilt once more
        // from the pose at release, then the transition to the true target runs.
        const bool drive_enabled = compute_drive_enabled_();
        *drive_enabled_ = drive_enabled;
        {
            using Switch = rmcs_msgs::Switch;
            const bool have_switches = switch_left_.ready() && switch_right_.ready();
            bool balance_armed = true; // sim: no remote -> balance allowed
            if (have_switches) {
                const bool fresh = dr16_fresh_.ready() ? *dr16_fresh_ : true;
                balance_armed =
                    fresh && *switch_left_ == Switch::MIDDLE && *switch_right_ == Switch::UP;
            }
            *balance_enabled_ = drive_enabled && balance_armed;
        }
        if (!drive_enabled || !prev_drive_enabled_) {
            initialized_ = false;
            elapsed_ = 0.0;
            since_enable_ = 0.0;
        } else {
            since_enable_ += dt_;
        }
        prev_drive_enabled_ = drive_enabled;

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

        // Startup settle: right after the drive enables, keep the leg pinned to
        // the measured pose for a short time so the balance can settle (the
        // initial half-metre drift) BEFORE the leg target changes. Applying the
        // new stance during that transient is what folds the leg on the first
        // second when /wheelleg/hold is 0.
        if (since_enable_ < startup_settle_s_) {
            target.leg_length = state.leg_length;
            target.beta = state.knee;
            target.shank = state.shank;
            target.hip = std::numbers::pi - state.pitch - state.knee - state.shank;
            target.feasible = true;
        }

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
            // Acceleration gate: the posture/gait (including the com_offset
            // feedback term) only acts while the planned acceleration is above
            // `accel_gate_min` [m/s^2]. At rest the old `d_ref` term is zero but
            // `accel_kv*d_err = -accel_kv*com_offset` is NOT, so without this
            // gate the (biased) com_offset would move the stance while standing.
            // 0.1 m/s^2 (not 1e-3) so sensor/command noise cannot trigger it.
            const bool accel_mode = std::abs(a_cmd) > accel_gate_min_;
            // Gate: legacy alpha_r fade, or a base-height gate so the posture can
            // also act during the get-up (alpha_r = 0) once the body is high enough
            // for the CoM / leg model to be meaningful.
            double gait_gate = accel_mode ? rec : 0.0;
            if (accel_mode && gait_height_min_ > 0.0) {
                gait_gate = std::isfinite(state.height)
                              ? std::clamp(
                                    (state.height - gait_height_min_)
                                        / std::max(gait_height_span_, kEpsilon),
                                    0.0, 1.0)
                              : 0.0;
            }
            gait_dhip_ = gait_gate * posture_gate * (accel_theta_ff_ * d_ref + accel_kv_ * d_err);
            const double gait_dl =
                gait_gate * posture_gate * (accel_leg_ff_ * d_ref + accel_d_kp_ * d_err);
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
        // A contact source only counts when it publishes a FINITE value. The
        // real-robot estimator has no force sensor and publishes NaN, so the
        // wheels are treated as loaded (its documented default); the MuJoCo
        // bridge publishes real normal forces and takes the normal path.
        const bool contact_available =
            (contact_normal_l_.ready() && std::isfinite(*contact_normal_l_))
            || (contact_normal_r_.ready() && std::isfinite(*contact_normal_r_));
        const bool supported = !contact_available || state.contact_l > contact_normal_eps_
                            || state.contact_r > contact_normal_eps_;
        const bool guard = rec >= 1.0 && supported && std::abs(state.com_offset) > d_max_;
        if (guard) {
            target.leg_length = state.leg_length;
            target.feasible = false;
        }

        // Balance ADRC enable: ON whenever the wheels carry the body. The old
        // |pitch| < tilt gate was a get-up leftover (it disabled the balance as
        // soon as the body tipped past ~45 deg, so a wheel-leg could never
        // recover from a fall). A wheel-leg must keep balancing while the wheels
        // are loaded; the airborne case is already covered by `supported`.
        // Loaded AND actually driving. This is what makes /wheelleg/hold (calib
        // modes) disable the balance loop as well, not just the leg reference.
        *balance_enable_ = supported && drive_enabled;

        // Base-velocity outer loop -> balance feedforward (wheel torque units).
        // The pitch-only balance controls the pitch mode but leaves the base
        // translational mode (x, xdot ~ r*omega) free; this term damps it. It is
        // off during recovery (rec < 1) and while the wheels are unloaded, and
        // the integral is unwound then so it cannot wind up.
        {
            const double omega = state.wheel_velocity;
            const bool active = supported && rec >= 1.0 && std::isfinite(omega);
            if (active)
                balance_wheel_integral_ += omega * dt_;
            else
                balance_wheel_integral_ = 0.0;
            balance_wheel_integral_ = std::clamp(
                balance_wheel_integral_, -balance_integral_limit_, balance_integral_limit_);
            *balance_feedforward_ = active
                ? -balance_velocity_kp_ * omega
                      - balance_position_kp_ * balance_wheel_integral_
                : 0.0;
        }

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
        // Per-leg shape (fallback = the averaged shank/knee when absent).
        double shank_l = kNaN, shank_r = kNaN;
        double knee_l = kNaN, knee_r = kNaN;
        double pitch = kNaN;
        double pitch_velocity = kNaN;
        double com_offset = kNaN;
        double leg_length = kNaN;
        double height = kNaN;
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
        // Unified name: body_com_forward (body CoM forward offset from the hip).
        // Fall back to the legacy body_com_offset so older configs still work.
        body_com_offset_ = node::param_or(
            "body_com_forward", node::param_or("body_com_offset", 0.0));

        transition_time_pitch_ = node::param_or("transition_time_pitch", 0.15);
        transition_time_shank_ = node::param_or("transition_time_shank", 1.5);
        transition_time_knee_ = node::param_or("transition_time_knee", 1.5);

        // Signed knee angle: choose the assembly branch ("knee forward" +1 /
        // "knee backward" -1), see 执行路线1_数学解析.md §6.3/§6.6.
        knee_branch_sign_ = node::param_or("knee_branch_sign", -1.0);
        knee_angle_min_ = node::param_or("knee_angle_min", -std::numbers::pi);
        knee_angle_max_ = node::param_or("knee_angle_max", std::numbers::pi);
        singularity_epsilon_ = node::param_or("singularity_epsilon", 1e-3);

        leg_length_epsilon_ = node::param_or("leg_length_epsilon", 1e-2);
        startup_settle_s_ = std::max(0.0, node::param_or("startup_settle_s", 0.0));
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
        // Height gate for the acceleration posture (gait): once the base height
        // exceeds gait_height_min the posture ramps in over gait_height_span. This
        // lets the posture also act during the get-up (while alpha_r = 0), where
        // the wheels alone cannot absorb the leg motion. gait_height_min <= 0
        // keeps the legacy alpha_r (recovery) fade.
        gait_height_min_ = node::param_or("gait_height_min", -1.0);
        gait_height_span_ = node::param_or("gait_height_span", 0.05);
        accel_gate_min_ = node::param_or("accel_gate_min", 0.1);
        d_max_ = node::param_or("d_max", 0.03);
        // Wheel normal force above which the wheel is treated as loaded (contact
        // gate for the static-stability guard / balance regime).
        contact_normal_eps_ = node::param_or("contact_normal_eps", 1.0);
        // Base-velocity outer loop gains (see /wheelleg/balance/feedforward):
        //   balance_velocity_kp  [Nm per rad/s of wheel speed]  (damping)
        //   balance_position_kp  [Nm per rad of wheel angle]    (drift trim)
        //   balance_integral_limit [rad] clamps the integrated wheel angle.
        balance_velocity_kp_ = node::param_or("balance_velocity_kp", 0.0);
        balance_position_kp_ = node::param_or("balance_position_kp", 0.0);
        balance_integral_limit_ = node::param_or("balance_integral_limit", 5.0);
        gravity_ = node::param_or("gravity", 9.81);
        arm_dwell_s_ = node::param_or("arm_dwell_s", 0.5);
        max_tilt_deg_ = node::param_or("max_tilt_deg", 0.0);
    }

    // True when the wheel-leg motors are actually enabled: no sim start-pose
    // hold, armed on the real remote (same ArmDwell as the drive bridge; absent
    // remote -> always armed, e.g. simulation), and inside the tilt cutoff.
    bool compute_drive_enabled_() {
        using Switch = rmcs_msgs::Switch;
        const bool have_switches = switch_left_.ready() && switch_right_.ready();
        bool armed;
        if (!have_switches) {
            armed = true;
        } else {
            const bool fresh = dr16_fresh_.ready() ? *dr16_fresh_ : true;
            if (arm_.update(
                    ArmDwell::Clock::now(), fresh, *switch_left_, *switch_right_, arm_dwell_s_))
                armed_ = true;
            if (*switch_left_ == Switch::DOWN && *switch_right_ == Switch::DOWN)
                armed_ = false;
            armed = armed_;
        }
        const bool hold = hold_.ready() && *hold_;
        const bool tilt_ok = !(max_tilt_deg_ > 0.0 && std::isfinite(*state_pitch_)
                               && std::abs(*state_pitch_)
                                      > max_tilt_deg_ * (std::numbers::pi / 180.0));
        // On real hardware wait for the DM drives to actually be enabled; the
        // simulation has no such signal and ignores this gate.
        const bool dm_ready = !dm_control_ready_.ready() || *dm_control_ready_;
        return armed && !hold && tilt_ok && dm_ready;
    }

    bool read_state_(State& state) const {
        state.wheel_velocity = *state_wheel_velocity_;
        state.shank = *state_shank_angle_;
        state.shank_velocity = *state_shank_velocity_;
        state.knee = *state_knee_angle_;
        state.knee_velocity = *state_knee_velocity_;
        state.shank_l = (state_shank_angle_l_.ready() && std::isfinite(*state_shank_angle_l_))
                          ? *state_shank_angle_l_
                          : state.shank;
        state.shank_r = (state_shank_angle_r_.ready() && std::isfinite(*state_shank_angle_r_))
                          ? *state_shank_angle_r_
                          : state.shank;
        state.knee_l = (state_knee_angle_l_.ready() && std::isfinite(*state_knee_angle_l_))
                         ? *state_knee_angle_l_
                         : state.knee;
        state.knee_r = (state_knee_angle_r_.ready() && std::isfinite(*state_knee_angle_r_))
                         ? *state_knee_angle_r_
                         : state.knee;
        state.pitch = *state_pitch_;
        state.pitch_velocity = *state_pitch_velocity_;
        state.com_offset = *state_com_offset_;
        state.leg_length = *state_leg_length_;
        state.height =
            (state_base_height_.ready() && std::isfinite(*state_base_height_)) ? *state_base_height_
                                                                              : kNaN;
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
        if (gamma > kEpsilon) {
            // P sin(phi) + Q cos(phi) = m_b c_b has TWO branches. The physically
            // correct one (matching the real-robot encoder / bench convention,
            // axle_line(beta, hip) ~ -90 deg at the nominal stance) is
            //   phi = pi - asin(m_b c_b / Gamma) - atan2(Q, P),
            // NOT asin(...) - atan2(...) (that is the pi-opposite branch, which
            // drove the real leg "straight up"). Wrap into (-pi, pi] so the
            // per-leg plan does not take the long way.
            shank = std::remainder(
                std::numbers::pi
                    - std::asin(
                        std::clamp(body_mass_ * body_com_offset_ / gamma, -1.0, 1.0))
                    - std::atan2(q, p),
                2.0 * std::numbers::pi);
        }

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
        // Plan EACH leg from its own measured pose to the common nominal target.
        // The static target is on the same branch as the measured state (both
        // wrapped), so no 2*pi re-snapping is needed.
        for (std::size_t s = 0; s < 2; ++s) {
            const double shank0 = (s == 0) ? state.shank_l : state.shank_r;
            const double knee0 = (s == 0) ? state.knee_l : state.knee_r;
            shank_plans_[s].reset(
                shank0, state.shank_velocity, target.shank, 0.0, transition_time_shank_);
            knee_plans_[s].reset(
                knee0, state.knee_velocity, target.beta, 0.0, transition_time_knee_);
        }
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
        std::array<double, 2> shank_reference{}, shank_rate{}, knee_reference{}, knee_rate{};
        if (recovering) {
            for (std::size_t s = 0; s < 2; ++s) {
                shank_reference[s] = target.shank;
                knee_reference[s] = target.beta;
            }
        } else {
            for (std::size_t s = 0; s < 2; ++s) {
                shank_reference[s] = shank_plans_[s].value(elapsed_);
                shank_rate[s] = shank_plans_[s].derivative(elapsed_);
                knee_reference[s] = knee_plans_[s].value(elapsed_);
                knee_rate[s] = knee_plans_[s].derivative(elapsed_);
            }
        }

        // §7.4: alpha_ref = pi - theta - beta_ref - phi_ref (+ outer d-loop bias).
        // Only the pitch/attitude terms are common; each leg has its own beta/phi.
        const double hip_pitch = recovering ? last_pitch_ : pitch_reference;
        const double alpha_bias =
            (alpha_bias_.ready() && std::isfinite(*alpha_bias_)) ? *alpha_bias_ : 0.0;
        std::array<double, 2> shank_published{}, knee_published{}, alpha_published{};
        for (std::size_t s = 0; s < 2; ++s) {
            shank_published[s] = shank_reference[s] + gait_dshank_;
            knee_published[s] = knee_reference[s] + gait_dknee_;
            alpha_published[s] = std::numbers::pi - hip_pitch - knee_published[s]
                               - shank_published[s] + alpha_bias + gait_dhip_;
        }

        // Leg-motion correction, 执行路线1_数学解析.md §7.5 (average of both legs
        // drives the common wheel):
        //   x_e_dot = (-L1 sin(beta+phi) + L2 sin(phi)) phi_dot - L1 sin(beta+phi) beta_dot
        //   omega_ref = (v_ref + x_e_dot) / r
        double shank_dx = 0.0;
        for (std::size_t s = 0; s < 2; ++s) {
            const double thigh_projection =
                thigh_length_ * std::sin(knee_reference[s] + shank_reference[s]);
            shank_dx += (-thigh_projection + shank_length_ * std::sin(shank_reference[s]))
                            * shank_rate[s]
                      - thigh_projection * knee_rate[s];
        }
        shank_dx *= 0.5;
        *reference_wheel_velocity_ = (velocity_reference + shank_dx) / wheel_radius_;
        // Pitch setpoint: the planned value (-> 0) in the balance regime, the
        // measured pitch during recovery (alpha_r = 0), so the balance ring is
        // not fed a setpoint it cannot achieve while the wheels are unloaded.
        *reference_pitch_ = recovery_blend_ * pitch_reference + (1.0 - recovery_blend_) * last_pitch_;

        // Per-leg references (drive bridge uses these; the averaged topics are
        // kept for monitoring / backward compatibility).
        *reference_shank_angle_l_ = shank_published[0];
        *reference_shank_angle_r_ = shank_published[1];
        *reference_knee_angle_l_ = knee_published[0];
        *reference_knee_angle_r_ = knee_published[1];
        *reference_hip_angle_l_ = alpha_published[0];
        *reference_hip_angle_r_ = alpha_published[1];
        *reference_shank_angle_ = 0.5 * (shank_published[0] + shank_published[1]);
        *reference_knee_angle_ = 0.5 * (knee_published[0] + knee_published[1]);
        *reference_hip_angle_ = 0.5 * (alpha_published[0] + alpha_published[1]);

        // Only print while the drive is actually enabled (armed, not in a calib
        // hold); keeps the calibration modes quiet.
        if (prev_drive_enabled_ && dbg_count_++ % 500 == 0) {
            constexpr double kRadToDeg = 180.0 / std::numbers::pi;
            const double knee_avg = 0.5 * (knee_reference[0] + knee_reference[1]);
            const double shank_avg = 0.5 * (shank_reference[0] + shank_reference[1]);
            info(
                "traj tgt L0={:.4f} beta={:.2f} shank={:.2f} hip={:.2f} deg | ref knee={:.2f} "
                "hip={:.2f} shank={:.2f} deg | ref_wv={:.3f} state_wv={:.3f}",
                target.leg_length, target.beta * kRadToDeg, target.shank * kRadToDeg,
                target.hip * kRadToDeg, knee_avg * kRadToDeg,
                (std::numbers::pi - hip_pitch - knee_avg - shank_avg) * kRadToDeg,
                shank_avg * kRadToDeg, *reference_wheel_velocity_,
                (state_wheel_velocity_.ready() ? *state_wheel_velocity_ : 0.0));
        }
    }

    void disable_() {
        initialized_ = false;
        elapsed_ = 0.0;
        since_enable_ = 0.0;
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
        *reference_shank_angle_l_ = kNaN;
        *reference_shank_angle_r_ = kNaN;
        *reference_knee_angle_l_ = kNaN;
        *reference_knee_angle_r_ = kNaN;
        *reference_hip_angle_l_ = kNaN;
        *reference_hip_angle_r_ = kNaN;
        *balance_enable_ = false;
        *balance_enabled_ = false;
        // No valid state -> no drive; the gate recomputes on the next valid tick.
        *drive_enabled_ = false;
        prev_drive_enabled_ = false;
        balance_wheel_integral_ = 0.0;
        *balance_feedforward_ = 0.0;
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
    double gait_height_min_ = -1.0;
    double gait_height_span_ = 0.05;
    double accel_gate_min_ = 0.1;
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
    InputInterface<double> state_shank_angle_l_, state_shank_angle_r_;
    InputInterface<double> state_knee_angle_l_, state_knee_angle_r_;
    InputInterface<double> state_pitch_;
    InputInterface<double> state_pitch_velocity_;
    InputInterface<double> state_com_offset_;
    InputInterface<double> state_leg_length_;
    InputInterface<double> state_base_height_;
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
    InputInterface<bool> hold_;
    InputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    InputInterface<bool> dr16_fresh_;
    InputInterface<bool> dm_control_ready_;

    OutputInterface<double> reference_wheel_velocity_;
    OutputInterface<double> reference_pitch_;
    OutputInterface<double> reference_shank_angle_;
    OutputInterface<double> reference_knee_angle_;
    OutputInterface<double> reference_hip_angle_;
    OutputInterface<double> reference_shank_angle_l_, reference_shank_angle_r_;
    OutputInterface<double> reference_knee_angle_l_, reference_knee_angle_r_;
    OutputInterface<double> reference_hip_angle_l_, reference_hip_angle_r_;
    OutputInterface<bool> balance_enable_;
    OutputInterface<bool> drive_enabled_;
    OutputInterface<bool> balance_enabled_;
    OutputInterface<double> balance_feedforward_;
    // Base-velocity outer loop (see /wheelleg/balance/feedforward).
    double balance_velocity_kp_ = 0.0;
    double balance_position_kp_ = 0.0;
    double balance_integral_limit_ = 5.0;
    double balance_wheel_integral_ = 0.0;
    double body_distance_reference_ = 0.0;
    double posture_yaw_scale_ = 0.15;
    bool posture_gate_enabled_ = false;

    // Drive-enable gate (see compute_drive_enabled_).
    ArmDwell arm_;
    bool armed_ = false;
    bool prev_drive_enabled_ = false;
    double since_enable_ = 0.0;
    double startup_settle_s_ = 0.0;
    double arm_dwell_s_ = 0.5;
    double max_tilt_deg_ = 0.0;

    Quintic pitch_plan_;
    // One quintic per leg (no left/right averaging of the start pose).
    std::array<Quintic, 2> shank_plans_;
    std::array<Quintic, 2> knee_plans_;

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
