#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "controller/wheelleg/leg_transmission.hpp"
#include "controller/wheelleg/wheelleg_leg_model.hpp"

namespace rmcs_core::controller::wheelleg {

using LegLinkage = rmcs_core::wheelleg::LegLinkage;

// Self-righting / stand-up controller for the wheel-leg.
//
// This is a "copy the approach" port of the RL self-righting state machine
// (rmcs_rl: JointReferenceRecoveryController / RecoveryController) into the
// V5.0 reduced sagittal model used by this package. It does NOT depend on
// rmcs_rl, does NOT use ADRC, and owns the wheels itself with the same pitch
// law the RL script uses.
//
// Poses come from the frozen V6 recovery profile
// (rmcs_rl/models/wheel_leg/deployment/v6_recovery_profiles_v1.json). The
// profile stores each pose as (hip L_joint1, interior knee deg) which maps
// directly onto our reduced model (hip == raw L_joint1, see 执行路线1 §3):
//
//   pose              hip (L_joint1)   interior knee beta
//   fold              -0.252107        50.000 deg
//   nominal/upright   -0.420000        67.855 deg
//   thrust            -0.749367        95.000 deg
//
// The reduced knee reference published to the bridge is -beta (the controller
// branch sign used by WheelLegTrajectoryController).
//
// Phase machine (sagittal; body on its belly, wheels up):
//   idle -> SELECT -> FOLD -> PLANT -> PREPARE -> BLEND -> DONE
// (upside down)      -> FOLD -> ORBIT -> THRUST -> CAPTURE -> PREPARE -> ...
//
// Wheels are the only external pitch authority; they are only usable once the
// wheels are loaded, so the script is open-loop on the legs against the ground
// until PREPARE/CAPTURE, where a pitch PD (their `efforts_` law) takes over.
//
// It communicates only through executor topics (register_input/register_output)
// so the same component drives either bridge, selected by `backend`:
//   * "simulation": status /wheelleg/state/*, command /wheelleg/reference/* and
//     /wheelleg/control/wheel_torque (MuJoCo bridge, wheelleg_simulation.cpp).
//   * "hardware":  status /wheel_leg/<axis>/{angle,velocity} + IMU,
//     command /wheel_leg/<axis>/{control_angle,control_torque} (real bridge,
//     wheel-leg.cpp). Contact / height come from the recovery observer topics.
class WheelLegStandupController
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegStandupController()
        : Node{get_component_name(), node::options()} {
        load_config_();
        register_io_();
    }

    void update() override {
        // Never act before explicitly triggered (unless start_on_boot).
        const bool triggered = start_on_boot_ || (trigger_.ready() && *trigger_);
        if (!triggered) {
            disable_();
            return;
        }

        State state;
        if (!read_state_(state)) {
            safe_output_();
            return;
        }

        if (phase_ == Phase::kIdle) {
            select_route_(state);
            ref_hip_ = state.hip;
            ref_beta_ = state.beta;
            goal_hip_ = ref_hip_;
            goal_beta_ = ref_beta_;
            // Chassis on the ground (tilt < 70 deg): first rotate the legs down
            // to plant the wheels, then stand. Upside down: tuck (fold) then
            // orbit. Already upright: go straight to the stand-up extension.
            // `start_phase` overrides the automatic choice (e.g. "fold" to run
            // the RL FOLD->PLANT->PREPARE from a self-consistent start pose).
            if (start_phase_ == "fold") {
                route_ = Route::kPlant;
                enter_(Phase::kFold);
            } else if (start_phase_ == "lower") {
                route_ = Route::kPlant;
                enter_(Phase::kLower);
            } else if (start_phase_ == "prepare") {
                enter_(Phase::kPrepare);
            } else if (route_ == Route::kStand) {
                enter_(Phase::kPrepare);
            } else if (route_ == Route::kPlant) {
                enter_(Phase::kLower);
            } else {
                enter_(Phase::kFold);
            }
        }

        phase_elapsed_ += dt_;
        update_goal_(state);
        advance_reference_();
        update_transitions_(state);

        if (dbg_count_++ % 500 == 0)
            info(
                "standup {} route={} hip={:.3f}/{:.3f} beta={:.3f}/{:.3f} pitch={:.4f} "
                "contact={} h={:.3f}",
                std::string_view{phase_name_()}, std::string_view{route_name_()}, ref_hip_, goal_hip_,
                ref_beta_, goal_beta_, state.pitch, state.contact, state.height);

        write_command_(state);
    }

private:
    enum class Phase {
        kIdle,
        kLower,
        kFold,
        kPlant,
        kOrbit,
        kThrust,
        kPrepare,
        kCapture,
        kBlend,
        kDone,
        kFailed
    };
    enum class Route { kStand, kPlant, kOrbitPositive, kOrbitNegative };

    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kEpsilon = 1e-9;

    [[nodiscard]] static constexpr double deg2rad(double deg) {
        return deg * std::numbers::pi / 180.0;
    }

    struct State {
        double hip = 0.0;      // measured reduced hip (== raw L_joint1)
        double beta = 0.0;     // measured interior knee angle
        double pitch = 0.0;    // fore-aft body pitch
        double pitch_velocity = 0.0;
        double wheel_velocity = 0.0;
        double height = 0.0;   // base height above ground
        bool contact = false;  // wheel(s) loaded
        // Raw hardware joints (hardware backend only).
        double hip_l = 0.0, hip_r = 0.0, crank_l = 0.0, crank_r = 0.0;
    };

    // ---- Phase selection (their `start` route choice) ----
    void select_route_(const State& state) {
        const double tilt = std::abs(state.pitch);
        if (tilt < stand_tilt_ && state.height > stand_height_)
            route_ = Route::kStand;
        else if (tilt < plant_tilt_)
            route_ = Route::kPlant;
        else
            route_ = state.pitch >= 0.0 ? Route::kOrbitPositive : Route::kOrbitNegative;
    }

    // ---- Per-phase goal (their `update_goal_` / `update_reference_`) ----
    void update_goal_(const State& state) {
        switch (phase_) {
        case Phase::kLower:
            // Rotate the legs down toward a "legs below" pose while keeping them
            // extended, so the wheels reach the ground and take load early.
            goal_hip_ = lower_hip_;
            goal_beta_ = lower_beta_;
            goal_speed_ = lower_speed_;
            break;
        case Phase::kFold:
            goal_hip_ = fold_hip_;
            goal_beta_ = fold_beta_;
            goal_speed_ = fold_speed_;
            break;
        case Phase::kPlant: {
            // fold pose + a pitch-driven offset so the wheel reaches the ground.
            double offset = 0.0;
            if (std::abs(std::sin(state.pitch)) < 0.5) {
                const double direction =
                    (state.pitch > 0.0) - (state.pitch < 0.0);
                offset = std::clamp(
                    -state.pitch - plant_angle_rad_ * direction, -2.2, 2.2);
            }
            goal_hip_ = fold_hip_ + offset;
            goal_beta_ = fold_beta_;
            goal_speed_ = plant_speed_;
            break;
        }
        case Phase::kOrbit: {
            orbit_angle_ += orbit_speed_ * dt_;
            const double direction = route_ == Route::kOrbitPositive ? 1.0 : -1.0;
            goal_hip_ = orbit_start_hip_ + direction * orbit_angle_;
            goal_beta_ = orbit_start_beta_;
            goal_speed_ = orbit_speed_;
            break;
        }
        case Phase::kThrust:
            // Native: goal = thrust - (pitch - plant_pitch); extend to push the
            // chassis off the ground while holding the planted wheel.
            goal_hip_ = thrust_hip_ - (state.pitch - planted_pitch_);
            goal_beta_ = thrust_beta_;
            goal_speed_ = thrust_speed_;
            break;
        case Phase::kPrepare:
        case Phase::kCapture:
        case Phase::kBlend:
        case Phase::kDone: {
            double hip = nominal_hip_;
            double beta = nominal_beta_;
            const double tilt = std::abs(state.pitch);
            if (state.contact && tilt < deg2rad(85.0)) {
                const double extension =
                    std::clamp((tilt - deg2rad(12.0)) / deg2rad(43.0), 0.0, 1.0);
                hip = nominal_hip_ + extension * (support_hip_ - nominal_hip_) - state.pitch;
                beta = nominal_beta_ + extension * (support_beta_ - nominal_beta_);
            }
            goal_hip_ = hip;
            goal_beta_ = beta;
            goal_speed_ = phase_ == Phase::kCapture ? capture_speed_ : prepare_speed_;
            break;
        }
        default:
            goal_hip_ = ref_hip_;
            goal_beta_ = ref_beta_;
            goal_speed_ = 0.0;
            break;
        }
    }

    // ---- Rate-limited joint reference toward the goal (their advance_reference_) ----
    void advance_reference_() {
        const double max_step = goal_speed_ * dt_;
        ref_hip_ += std::clamp(goal_hip_ - ref_hip_, -max_step, max_step);
        ref_beta_ += std::clamp(goal_beta_ - ref_beta_, -max_step, max_step);
    }

    // ---- Transitions (their `update_phase_`) ----
    void update_transitions_(const State& state) {
        const double tilt = std::abs(state.pitch);
        const bool ref_ready = std::abs(goal_hip_ - ref_hip_) < 0.02
                            && std::abs(goal_beta_ - ref_beta_) < 0.02;

        switch (phase_) {
        case Phase::kLower:
            // Legs are down (reference reached); hand over to the stand-up
            // extension (PREPARE), which extends toward the nominal stance.
            if (ref_ready && phase_elapsed_ >= lower_min_seconds_)
                enter_(Phase::kPrepare);
            break;
        case Phase::kFold:
            if (ref_ready && phase_elapsed_ >= fold_min_seconds_) {
                if (route_ == Route::kOrbitPositive || route_ == Route::kOrbitNegative) {
                    orbit_start_hip_ = ref_hip_;
                    orbit_start_beta_ = ref_beta_;
                    orbit_angle_ = 0.0;
                    enter_(Phase::kOrbit);
                } else {
                    enter_(Phase::kPlant);
                }
            }
            break;
        case Phase::kPlant:
            contact_elapsed_ = state.contact ? contact_elapsed_ + dt_ : 0.0;
            if (ref_ready && contact_elapsed_ >= plant_contact_seconds_)
                enter_(Phase::kPrepare);
            else if (phase_elapsed_ > plant_timeout_)
                enter_(Phase::kFailed);
            break;
        case Phase::kOrbit:
            if (state.contact && tilt < deg2rad(30.0) && state.height > 0.22)
                enter_(Phase::kPrepare);
            else if (
                state.contact && tilt < deg2rad(145.0) && state.height > 0.12
                && orbit_angle_ > orbit_contact_angle_) {
                planted_pitch_ = state.pitch;
                enter_(Phase::kThrust);
            } else if (orbit_angle_ >= 2.0 * std::numbers::pi * orbit_turns_) {
                enter_(Phase::kFailed);
            }
            break;
        case Phase::kThrust:
            if (tilt < deg2rad(65.0) && state.height > 0.24)
                enter_(Phase::kCapture);
            break;
        case Phase::kPrepare:
        case Phase::kCapture: {
            const bool ready = state.contact && tilt < handover_tilt_
                            && state.height > handover_h_min_ && state.height < handover_h_max_;
            ready_elapsed_ = ready ? ready_elapsed_ + dt_ : 0.0;
            if (ready_elapsed_ >= handover_seconds_)
                enter_(Phase::kBlend);
            break;
        }
        case Phase::kBlend:
            if (phase_elapsed_ >= blend_seconds_)
                enter_(Phase::kDone);
            break;
        default:
            break;
        }

        // Lost the upright while trusting the wheels.
        if ((phase_ == Phase::kBlend || phase_ == Phase::kDone) && tilt > deg2rad(45.0))
            enter_(Phase::kFailed);
    }

    // ---- Wheel torque (their pitch law; plus a hold during kLower) ----
    [[nodiscard]] double balance_torque_(const State& state) const {
        if (phase_ == Phase::kFailed || phase_ == Phase::kIdle)
            return 0.0;
        if (phase_ == Phase::kLower) {
            // Hold the wheels (brake) while the legs rotate down, so the leg
            // swing pivots the body over the wheel axle instead of free-rolling
            // the base along the ground.
            return std::clamp(
                -lower_brake_kp_ * state.wheel_velocity, -wheel_torque_limit_,
                wheel_torque_limit_);
        }
        // Wheels balance only once the body is being supported by them
        // (PREPARE onward).
        const bool balancing =
            (phase_ == Phase::kPlant || phase_ == Phase::kPrepare
             || phase_ == Phase::kCapture || phase_ == Phase::kBlend || phase_ == Phase::kDone)
            && std::abs(state.pitch) < balance_tilt_;
        if (!balancing || !state.contact)
            return 0.0;
        const double effort = pitch_kp_ * state.pitch + pitch_kd_ * state.pitch_velocity;
        const double torque = effort * wheel_axis_sign_ - wheel_damping_ * state.wheel_velocity;
        return std::clamp(torque, -wheel_torque_limit_, wheel_torque_limit_);
    }

    void enter_(Phase next) {
        phase_ = next;
        phase_elapsed_ = 0.0;
        contact_elapsed_ = 0.0;
        ready_elapsed_ = 0.0;
    }

    [[nodiscard]] const char* phase_name_() const {
        switch (phase_) {
        case Phase::kLower: return "lower";
        case Phase::kFold: return "fold";
        case Phase::kPlant: return "plant";
        case Phase::kOrbit: return "orbit";
        case Phase::kThrust: return "thrust";
        case Phase::kPrepare: return "prepare";
        case Phase::kCapture: return "capture";
        case Phase::kBlend: return "blend";
        case Phase::kDone: return "done";
        case Phase::kFailed: return "failed";
        default: return "idle";
        }
    }

    [[nodiscard]] const char* route_name_() const {
        switch (route_) {
        case Route::kStand: return "stand";
        case Route::kPlant: return "plant";
        case Route::kOrbitPositive: return "orbit+";
        case Route::kOrbitNegative: return "orbit-";
        default: return "?";
        }
    }

    void register_io_() {
        if (backend_ == "hardware") {
            for (std::size_t i = 0; i < kJointNames.size(); ++i) {
                const std::string prefix = std::string{"/wheel_leg/"} + kJointNames[i];
                register_input(prefix + "/angle", hw_angle_[i], false);
                register_input(prefix + "/velocity", hw_velocity_[i], false);
            }
            register_input("/wheel_leg/imu/sensor_gravity/x", hw_gravity_[0], false);
            register_input("/wheel_leg/imu/sensor_gravity/y", hw_gravity_[1], false);
            register_input("/wheel_leg/imu/sensor_gravity/z", hw_gravity_[2], false);
            register_input("/wheel_leg/imu/sensor_gyro/y", hw_gyro_y_, false);
            register_input("/wheel_leg/left_wheel/velocity", hw_wheel_velocity_[0], false);
            register_input("/wheel_leg/right_wheel/velocity", hw_wheel_velocity_[1], false);
            // Contact / height from the recovery observer (optional).
            register_input("/wheel_leg/recovery/support", hw_support_, false);
            register_input("/wheel_leg/recovery/height", hw_height_, false);

            for (std::size_t i = 0; i < kJointNames.size(); ++i) {
                const std::string prefix = std::string{"/wheel_leg/"} + kJointNames[i];
                register_output(prefix + "/control_angle", hw_control_angle_[i], kNaN);
                register_output(prefix + "/control_torque", hw_control_torque_[i], 0.0);
            }
            register_output("/wheel_leg/left_wheel/control_torque", hw_wheel_torque_[0], 0.0);
            register_output("/wheel_leg/right_wheel/control_torque", hw_wheel_torque_[1], 0.0);
        } else {
            register_input("/wheelleg/state/knee_angle", sim_knee_angle_);
            register_input("/wheelleg/state/knee_velocity", sim_knee_velocity_);
            register_input("/wheelleg/state/pitch", sim_pitch_);
            register_input("/wheelleg/state/pitch_velocity", sim_pitch_velocity_);
            register_input("/wheelleg/state/wheel_velocity", sim_wheel_velocity_);
            register_input("/wheelleg/state/hip_angle", sim_hip_state_);
            register_input("/wheelleg/state/base_height", sim_base_height_);
            register_input("/wheelleg/state/contact_normal_l", sim_contact_l_);
            register_input("/wheelleg/state/contact_normal_r", sim_contact_r_);

            register_output("/wheelleg/reference/hip_angle", sim_hip_angle_, kNaN);
            register_output("/wheelleg/reference/knee_angle", sim_knee_angle_out_, kNaN);
            register_output("/wheelleg/reference/wheel_velocity", sim_wheel_velocity_out_, kNaN);
            register_output("/wheelleg/control/wheel_torque", sim_wheel_torque_, 0.0);
        }

        register_input(trigger_topic_, trigger_, false);
    }

    bool read_state_(State& state) const {
        if (backend_ == "hardware") {
            if (!hw_angle_[0].ready() || !hw_gravity_[2].ready())
                return false;
            state.hip_l = *hw_angle_[kLeftHip];
            state.crank_l = *hw_angle_[kLeftCrank];
            state.hip_r = *hw_angle_[kRightHip];
            state.crank_r = *hw_angle_[kRightCrank];

            const double gy = *hw_gravity_[1], gz = *hw_gravity_[2];
            state.pitch = std::atan2(gy, -gz);
            state.pitch_velocity = *hw_gyro_y_;
            state.wheel_velocity = 0.5 * (*hw_wheel_velocity_[0] + *hw_wheel_velocity_[1]);

            const double beta_l = knee_from_hardware_(*hw_angle_[kLeftCrank], *hw_angle_[kLeftHip], 0);
            const double beta_r =
                knee_from_hardware_(*hw_angle_[kRightCrank], *hw_angle_[kRightHip], 1);
            state.beta = 0.5 * (beta_l + beta_r);
            state.hip = 0.5 * (state.hip_l + state.hip_r);
            state.contact = hw_support_.ready() && *hw_support_;
            state.height = hw_height_.ready() ? *hw_height_ : 0.0;
            return std::isfinite(state.pitch) && std::isfinite(state.beta);
        }

        if (!sim_pitch_.ready() || !sim_knee_angle_.ready())
            return false;
        state.pitch = *sim_pitch_;
        state.pitch_velocity = sim_pitch_velocity_.ready() ? *sim_pitch_velocity_ : 0.0;
        state.beta = *sim_knee_angle_;  // state knee is the interior beta (unified)
        state.hip = sim_hip_state_.ready() ? *sim_hip_state_ : 0.0;
        state.wheel_velocity = sim_wheel_velocity_.ready() ? *sim_wheel_velocity_ : 0.0;
        state.height = sim_base_height_.ready() ? *sim_base_height_ : 0.0;
        const double cn_l = sim_contact_l_.ready() ? *sim_contact_l_ : 0.0;
        const double cn_r = sim_contact_r_.ready() ? *sim_contact_r_ : 0.0;
        state.contact = cn_l > contact_force_eps_ || cn_r > contact_force_eps_;
        return std::isfinite(state.pitch) && std::isfinite(state.beta);
    }

    // Reduced knee angle from hardware joint angles, through the SHARED model
    // (single source of truth) + the canonical (real-robot bench) calibration.
    [[nodiscard]] double knee_from_hardware_(double crank, double hip, int side) const {
        return leg_model_
            ->forward(hip, crank, leg_calib_[static_cast<std::size_t>(side)])
            .beta;
    }

    void write_command_(const State& state) {
        const double wheel_torque = balance_torque_(state);
        if (backend_ == "hardware") {
            const double beta =
                std::clamp(ref_beta_, linkage_->knee_min(), linkage_->knee_max());
            double hip_cad_l = 0.0, crank_cad_l = 0.0, hip_cad_r = 0.0, crank_cad_r = 0.0;
            if (!leg_model_->joint_targets(
                    ref_hip_, beta, leg_calib_[0], hip_cad_l, crank_cad_l)
                || !leg_model_->joint_targets(
                    ref_hip_, beta, leg_calib_[1], hip_cad_r, crank_cad_r)) {
                safe_output_();
                return;
            }

            // Mod 2*pi: the DM encoders are multi-turn; snap each target to the
            // turn nearest the measured joint so the servo / PD does not resolve
            // to a wrong turn (the model targets are single-turn).
            const auto align = [](double target, double measured) {
                return target
                     + 2.0 * std::numbers::pi
                         * std::round((measured - target) / (2.0 * std::numbers::pi));
            };
            *hw_control_angle_[kLeftHip] = align(hip_cad_l, *hw_angle_[kLeftHip]);
            *hw_control_angle_[kRightHip] = align(hip_cad_r, *hw_angle_[kRightHip]);
            *hw_control_angle_[kLeftCrank] = align(crank_cad_l, *hw_angle_[kLeftCrank]);
            *hw_control_angle_[kRightCrank] = align(crank_cad_r, *hw_angle_[kRightCrank]);
            for (auto& torque : hw_control_torque_)
                *torque = 0.0;
            *hw_wheel_torque_[0] = wheel_axis_sign_ * wheel_torque;
            *hw_wheel_torque_[1] = -wheel_axis_sign_ * wheel_torque;
            return;
        }

        *sim_hip_angle_ = ref_hip_;
        *sim_knee_angle_out_ = ref_beta_;
        *sim_wheel_velocity_out_ = target_wheel_velocity_;
        *sim_wheel_torque_ = wheel_torque;
    }

    void safe_output_() {
        if (backend_ == "hardware") {
            for (auto& angle : hw_control_angle_)
                *angle = kNaN;
            for (auto& torque : hw_control_torque_)
                *torque = 0.0;
            *hw_wheel_torque_[0] = 0.0;
            *hw_wheel_torque_[1] = 0.0;
        } else {
            *sim_hip_angle_ = kNaN;
            *sim_knee_angle_out_ = kNaN;
            *sim_wheel_velocity_out_ = kNaN;
            *sim_wheel_torque_ = 0.0;
        }
    }

    void disable_() {
        phase_ = Phase::kIdle;
        phase_elapsed_ = 0.0;
        contact_elapsed_ = 0.0;
        ready_elapsed_ = 0.0;
        orbit_angle_ = 0.0;
        safe_output_();
    }

    void load_config_() {
        dt_ = node::param_or("dt", 0.001);
        backend_ = node::param_or<std::string>("backend", std::string{"simulation"});
        if (backend_ != "simulation" && backend_ != "hardware")
            throw std::runtime_error{"standup backend must be 'simulation' or 'hardware'"};

        trigger_topic_ =
            node::param_or<std::string>("trigger_topic", std::string{"/wheelleg/standup/trigger"});
        start_on_boot_ = node::param_or("start_on_boot", false);
        // "auto" | "fold" | "lower" | "prepare": force the first phase.
        start_phase_ = node::param_or<std::string>("start_phase", std::string{"auto"});

        // Geometry / mass (must match the model; used for the hardware knee map).
        thigh_length_ = node::param_or("thigh_length", 0.21);
        shank_length_ = node::param_or("shank_length", 0.25005);
        body_mass_ = node::param_or("body_mass", 11.62);
        thigh_mass_ = node::param_or("thigh_mass", 0.426);
        shank_mass_ = node::param_or("shank_mass", 0.507);
        thigh_com_offset_ = node::param_or("thigh_com_offset", 0.06623);
        shank_com_offset_ = node::param_or("shank_com_offset", 0.15344);
        // Unified name: body_com_forward; legacy body_com_offset fallback.
        body_com_offset_ = node::param_or(
            "body_com_forward", node::param_or("body_com_offset", 0.00918));

        // Poses (our reduced convention: hip == raw L_joint1, beta == interior
        // knee). Defaults are the frozen V6 recovery poses.
        fold_hip_ = node::param_or("fold_hip", -0.252107);
        fold_beta_ = node::param_or("fold_beta", deg2rad(50.0));
        nominal_hip_ = node::param_or("nominal_hip", -0.42);
        nominal_beta_ = node::param_or("nominal_beta", deg2rad(67.85497));
        thrust_hip_ = node::param_or("thrust_hip", -0.749367);
        thrust_beta_ = node::param_or("thrust_beta", deg2rad(95.0));
        support_hip_ = node::param_or("support_hip", nominal_hip_);
        support_beta_ = node::param_or("support_beta", nominal_beta_);
        // "Lower the legs" first phase (chassis grounded, legs up): rotate the
        // legs down with the leg near full extension so the wheels reach the
        // ground and load early, before the stand-up extension.
        lower_hip_ = node::param_or("lower_hip", -0.42);
        lower_beta_ = node::param_or("lower_beta", deg2rad(105.0));
        lower_speed_ = node::param_or("lower_speed", 0.6);
        lower_min_seconds_ = node::param_or("lower_min_seconds", 0.5);
        // Wheel hold (brake) gain during kLower (Nm per rad/s).
        lower_brake_kp_ = node::param_or("lower_brake_kp", 20.0);

        // Reference speeds (native V6 JointReferenceRecoveryConfig).
        fold_speed_ = node::param_or("fold_speed", 1.0);
        plant_speed_ = node::param_or("plant_speed", 1.0);
        orbit_speed_ = node::param_or("orbit_speed", 4.0);
        thrust_speed_ = node::param_or("thrust_speed", 4.0);
        prepare_speed_ = node::param_or("prepare_speed", 1.0);
        capture_speed_ = node::param_or("capture_speed", 4.0);
        blend_seconds_ = node::param_or("blend_seconds", 0.2);
        plant_angle_rad_ = node::param_or("plant_angle_rad", std::numbers::pi / 9.0);

        // Route thresholds and dwell.
        stand_tilt_ = node::param_or("stand_tilt_deg", 15.0) * std::numbers::pi / 180.0;
        stand_height_ = node::param_or("stand_height", 0.20);
        plant_tilt_ = node::param_or("plant_tilt_deg", 70.0) * std::numbers::pi / 180.0;
        fold_min_seconds_ = node::param_or("fold_min_seconds", 0.3);
        plant_contact_seconds_ = node::param_or("plant_contact_seconds", 0.06);
        plant_timeout_ = node::param_or("plant_timeout", 2.5);
        orbit_turns_ = node::param_or("orbit_turns", 1.0);
        orbit_contact_angle_ = node::param_or("orbit_contact_angle", 3.7);
        handover_seconds_ = node::param_or("handover_seconds", 0.1);
        handover_tilt_ = node::param_or("handover_tilt_deg", 8.0) * std::numbers::pi / 180.0;
        handover_h_min_ = node::param_or("handover_height_min", 0.27);
        handover_h_max_ = node::param_or("handover_height_max", 0.36);

        // Wheel pitch balance (their `efforts_` pitch law; no ADRC).
        balance_tilt_ = node::param_or("balance_tilt_deg", 55.0) * std::numbers::pi / 180.0;
        pitch_kp_ = node::param_or("pitch_kp", 8.0);
        pitch_kd_ = node::param_or("pitch_kd", 1.5);
        wheel_damping_ = node::param_or("wheel_damping", 0.2);
        wheel_torque_limit_ = node::param_or("wheel_torque_limit", 4.88);
        wheel_axis_sign_ = node::param_or("wheel_axis_sign", 1.0);
        contact_force_eps_ = node::param_or("contact_force_eps", 1.0);
        target_wheel_velocity_ = node::param_or("target_wheel_velocity", 0.0);

        // Hardware joint<->reduced mapping, per side [left, right].
        hardware_hip_sign_ = node::param_or<std::vector<double>>("hardware_hip_sign", {-1.0, 1.0});
        hardware_hip_zero_ = node::param_or<std::vector<double>>(
            "hardware_hip_zero", {-std::numbers::pi / 2.0, std::numbers::pi / 2.0});
        hardware_crank_sign_ =
            node::param_or<std::vector<double>>("hardware_crank_sign", {-1.0, -1.0});
        if (hardware_hip_sign_.size() != 2 || hardware_hip_zero_.size() != 2
            || hardware_crank_sign_.size() != 2)
            throw std::runtime_error{"hardware_hip_*/hardware_crank_* need exactly [left, right]"};

        linkage_ = std::make_unique<LegLinkage>();
        leg_model_ = std::make_unique<rmcs_core::wheelleg::WheelLegLegModel>(
            thigh_length_, shank_length_, linkage_->knee_min(), linkage_->knee_max(), *linkage_);
        for (std::size_t i = 0; i < 2; ++i)
            leg_calib_[i] = rmcs_core::wheelleg::LegCalibration{
                hardware_hip_sign_[i], hardware_hip_zero_[i], hardware_crank_sign_[i]};
    }

    enum JointIndex : std::size_t {
        kLeftHip,
        kLeftCrank,
        kLeftWheel,
        kRightHip,
        kRightCrank,
        kRightWheel,
    };
    static constexpr std::array<const char*, 6> kJointNames = {
        "left_hip_joint", "left_knee_joint", "left_wheel",
        "right_hip_joint", "right_knee_joint", "right_wheel"};

    double dt_;
    std::string backend_;
    std::string trigger_topic_;
    bool start_on_boot_ = false;
    std::string start_phase_ = "auto";

    double thigh_length_;
    double shank_length_;
    double body_mass_;
    double thigh_mass_;
    double shank_mass_;
    double thigh_com_offset_;
    double shank_com_offset_;
    double body_com_offset_;

    // Poses.
    double fold_hip_, fold_beta_, nominal_hip_, nominal_beta_, thrust_hip_, thrust_beta_,
        support_hip_, support_beta_;
    // "Lower legs" first phase.
    double lower_hip_, lower_beta_, lower_speed_, lower_min_seconds_, lower_brake_kp_;
    // Reference speeds / timing.
    double fold_speed_, plant_speed_, orbit_speed_, thrust_speed_, prepare_speed_, capture_speed_,
        blend_seconds_;
    double plant_angle_rad_;
    double stand_tilt_, stand_height_, plant_tilt_;
    double fold_min_seconds_, plant_contact_seconds_, plant_timeout_, orbit_turns_,
        orbit_contact_angle_, handover_seconds_, handover_tilt_, handover_h_min_, handover_h_max_;
    // Wheel balance.
    double balance_tilt_, pitch_kp_, pitch_kd_, wheel_damping_, wheel_torque_limit_,
        wheel_axis_sign_, contact_force_eps_, target_wheel_velocity_;

    std::vector<double> hardware_hip_sign_;
    std::vector<double> hardware_hip_zero_;
    std::vector<double> hardware_crank_sign_;

    std::unique_ptr<LegLinkage> linkage_;
    std::unique_ptr<rmcs_core::wheelleg::WheelLegLegModel> leg_model_;
    std::array<rmcs_core::wheelleg::LegCalibration, 2> leg_calib_{};

    // Script state.
    Phase phase_ = Phase::kIdle;
    Route route_ = Route::kStand;
    double phase_elapsed_ = 0.0;
    double contact_elapsed_ = 0.0;
    double ready_elapsed_ = 0.0;
    double ref_hip_ = 0.0;
    double ref_beta_ = 0.0;
    double goal_hip_ = 0.0;
    double goal_beta_ = 0.0;
    double goal_speed_ = 0.0;
    double orbit_start_hip_ = 0.0;
    double orbit_start_beta_ = 0.0;
    double orbit_angle_ = 0.0;
    double planted_pitch_ = 0.0;

    InputInterface<bool> trigger_;

    InputInterface<double> sim_knee_angle_;
    InputInterface<double> sim_knee_velocity_;
    InputInterface<double> sim_pitch_;
    InputInterface<double> sim_pitch_velocity_;
    InputInterface<double> sim_wheel_velocity_;
    InputInterface<double> sim_hip_state_;
    InputInterface<double> sim_base_height_;
    InputInterface<double> sim_contact_l_;
    InputInterface<double> sim_contact_r_;

    OutputInterface<double> sim_hip_angle_;
    OutputInterface<double> sim_knee_angle_out_;
    OutputInterface<double> sim_wheel_velocity_out_;
    OutputInterface<double> sim_wheel_torque_;

    std::array<InputInterface<double>, 6> hw_angle_;
    std::array<InputInterface<double>, 6> hw_velocity_;
    std::array<InputInterface<double>, 3> hw_gravity_;
    InputInterface<double> hw_gyro_y_;
    std::array<InputInterface<double>, 2> hw_wheel_velocity_;
    InputInterface<bool> hw_support_;
    InputInterface<double> hw_height_;
    std::array<OutputInterface<double>, 6> hw_control_angle_;
    std::array<OutputInterface<double>, 6> hw_control_torque_;
    std::array<OutputInterface<double>, 2> hw_wheel_torque_;

    std::uint64_t dbg_count_ = 0;
};

} // namespace rmcs_core::controller::wheelleg

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::wheelleg::WheelLegStandupController, rmcs_executor::Component)
