#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "controller/wheelleg/leg_transmission.hpp"
#include "controller/wheelleg/wheelleg_leg_model.hpp"

namespace rmcs_core::controller::wheelleg {

using LegLinkage = rmcs_core::wheelleg::LegLinkage;

// Single shared leg model (see wheelleg_leg_model.hpp): the solver used here is
// exactly the one the estimator / bridge / simulation use.
using LegModel = rmcs_core::wheelleg::WheelLegLegModel;
using LegShape = rmcs_core::wheelleg::LegShape;
using LegCalibration = rmcs_core::wheelleg::LegCalibration;

[[nodiscard]] inline double wrap_to_pi(double angle) {
    return std::remainder(angle, 2.0 * std::numbers::pi);
}

// Bench commissioning assembly for the real V5.0 wheel-leg. It replaces the
// whole traditional chain (stand/move/motion-planner/trajectory/ADRC/mixer/
// drive-bridge) with a single self-contained component so the three bring-up
// tests do not depend on the not-yet-verified balance stack.
//
// `mode` (yaml) selects exactly one behaviour:
//   1  wheel-only: legs stay DISABLED the whole time (`wheel_only = true`);
//      the right-stick X axis commands a common wheel speed through a P loop,
//      so the wheel-torque / wheel-velocity / chassis-forward sign can be
//      measured. Torque is applied on the wheel output shaft.
//   2  passive: NO drive is enabled (`enable_request = false`); the operator
//      poses the legs by hand and the component prints the solver's leg shape
//      for both sides (interior thigh-shank angle and the hip->axle line angle
//      to horizontal, plus length/reduced angles). Body is suspended level.
//   3  pose: the interior knee is driven to an ABSOLUTE target
//      (`target_inner_knee_deg`, 40..105 deg) while the hip->axle line angle to
//      horizontal only moves by a SMALL relative `delta_leg_angle_deg` from the
//      measured value, so the leg barely swings. A quintic ramp (from the
//      measured pose, started once the hardware enables) drives it; the joint
//      targets are aligned to the multi-turn encoders. Wheels held at zero torque.
//   4  attitude: everything is DISABLED (`enable_request = false`); the
//      component only accumulates and prints the rolling mean/std of the
//      attitude signal (pitch/roll/yaw from /wheelleg/state/*) so IMU noise
//      can be characterised with the robot held still. The std is published
//      under /wheelleg/bench/{pitch,roll,yaw}_std.
//   5  teleop: joystick jog. The left-stick X integrates the interior knee and
//      the left-stick Y the hip->axle line angle, from the measured pose
//      (started once enabled), at `teleop_sensitivity` rad per 1 kHz step.
//      `teleop_side` selects left/right/both; the other leg holds. Wheels zero.
//
// Geometry mirrors WheelLegStateEstimator (same LegLinkage LUT and reduced-FK
// conventions) so mode 2 verifies the same solver. Attitude is assumed level:
// the body frame's +x is the horizontal reference, so the hip->axle angle is
// reported in the body sagittal plane.
//
// Enable is gated by the hardware (`wheel_leg_control.hpp`): both remote
// switches must be MIDDLE, DR16 must be fresh, and `/wheel_leg/feedback_fresh`
// must be true. Modes 1 and 3 request enable; modes 2 and 4 never do.
class WheelLegBenchTestController
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    using node = rmcs_utility::NodeMixin;

    WheelLegBenchTestController()
        : Node{get_component_name(), node::options()} {
        load_config_();
        register_interfaces_();
        publish_zero_();
    }

    void update() override {
        solve_leg_shape_();
        publish_leg_shape_();

        switch (mode_) {
        case 1: update_wheel_only_(); break;
        case 2: update_passive_(); break;
        case 3: update_pose_(); break;
        case 4: update_imu_test_(); break;
        case 5: update_teleop_(); break;
        default: update_passive_(); break;
        }

        maybe_log_();
    }

private:
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr std::size_t kNumJoints = 4; // [Lhip, Lknee, Rhip, Rknee]

    // ------------------------------------------------------------------ config
    void load_config_() {
        mode_ = get_parameter_or("mode", 1);

        thigh_length_ = get_parameter_or("thigh_length", 0.21);
        shank_length_ = get_parameter_or("shank_length", 0.25045);
        wheel_radius_ = get_parameter_or("wheel_radius", 0.06);
        knee_limit_min_ = get_parameter_or("knee_limit_min", 0.6981317007977318);
        knee_limit_max_ = get_parameter_or("knee_limit_max", 1.8325957145940461);

        hardware_hip_sign_ = get_parameter("hardware_hip_sign").as_double_array();
        hardware_hip_zero_ = get_parameter("hardware_hip_zero").as_double_array();
        hardware_crank_sign_ = get_parameter("hardware_crank_sign").as_double_array();
        if (hardware_hip_sign_.size() != 2 || hardware_hip_zero_.size() != 2
            || hardware_crank_sign_.size() != 2)
            throw std::invalid_argument(
                "hardware_hip_sign/zero/crank_sign require two values");

        // Mode 1 wheel loop (mirrors WheelLegWheelIdentificationController's P).
        wheel_velocity_kp_ = get_parameter_or("wheel_velocity_kp", 0.3);
        wheel_torque_limit_ = get_parameter_or("wheel_torque_limit", 4.88);
        vx_max_ = get_parameter_or("vx_max", 0.5);
        deadzone_ = get_parameter_or("deadzone", 0.08);
        wheel_command_sign_ =
            get_parameter_or("wheel_command_sign", std::vector<double>{1.0, 1.0});
        if (wheel_command_sign_.size() != 2)
            throw std::invalid_argument("wheel_command_sign requires two values [left, right]");

        // Mode 3 host-side leg PD (torque mode drives, zero-Kp MIT).
        leg_kp_ = get_parameter_or("leg_kp", 160.0);
        leg_kd_ = get_parameter_or("leg_kd", 2.5);
        leg_torque_limit_ = get_parameter_or("leg_torque_limit", 40.0);

        const double deg = std::numbers::pi / 180.0;
        target_inner_knee_ = get_parameter_or("target_inner_knee_deg", 90.0) * deg;
        ramp_seconds_ = get_parameter_or("ramp_seconds", 3.0);
        dt_ = get_parameter_or("dt", 0.001);
        delta_leg_angle_ = get_parameter_or("delta_leg_angle_deg", 0.0) * deg;
        // Mode 5 joystick teleop (left stick: X -> knee, Y -> leg angle).
        teleop_side_ = get_parameter_or("teleop_side", 2); // 0=left, 1=right, 2=both
        teleop_sensitivity_ = get_parameter_or("teleop_sensitivity", 0.006); // rad/step
        teleop_knee_sign_ = get_parameter_or("teleop_knee_sign", 1.0);
        teleop_leg_sign_ = get_parameter_or("teleop_leg_sign", 1.0);
        teleop_leg_min_ = get_parameter_or("teleop_leg_min_deg", -170.0) * deg;
        teleop_leg_max_ = get_parameter_or("teleop_leg_max_deg", -20.0) * deg;
        if (teleop_side_ < 0 || teleop_side_ > 2)
            throw std::invalid_argument("teleop_side must be 0 (left), 1 (right) or 2 (both)");
        if (ramp_seconds_ <= 0.0 || dt_ <= 0.0)
            throw std::invalid_argument("ramp_seconds and dt must be positive");
        if (target_inner_knee_ < knee_limit_min_ || target_inner_knee_ > knee_limit_max_)
            throw std::invalid_argument(
                "target_inner_knee_deg outside the mechanical knee limits");
        if (mode_ < 1 || mode_ > 5)
            throw std::invalid_argument(
                "mode must be 1 (wheel), 2 (passive), 3 (pose), 4 (imu) or 5 (teleop)");

        log_every_cycles_ = std::max<int>(1, get_parameter_or("log_every_cycles", 200));
        stats_period_cycles_ = std::max<int>(1, get_parameter_or("stats_period_cycles", 1000));

        model_ = std::make_unique<LegModel>(
            thigh_length_, shank_length_, knee_limit_min_, knee_limit_max_, linkage_);
        leg_calib_[0] =
            LegCalibration{hardware_hip_sign_[0], hardware_hip_zero_[0], hardware_crank_sign_[0]};
        leg_calib_[1] =
            LegCalibration{hardware_hip_sign_[1], hardware_hip_zero_[1], hardware_crank_sign_[1]};
    }

    void register_interfaces_() {
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);

        constexpr std::array kJointTopics{
            "left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint"};
        for (std::size_t i = 0; i < kNumJoints; ++i) {
            const std::string prefix = std::string{"/wheel_leg/"} + kJointTopics[i];
            register_input(prefix + "/angle", joint_angle_[i], false);
            register_input(prefix + "/velocity", joint_velocity_[i], false);
            register_output(prefix + "/control_torque", joint_torque_[i], 0.0);
        }
        constexpr std::array kWheelNames{"left_wheel", "right_wheel"};
        for (std::size_t i = 0; i < 2; ++i) {
            const std::string prefix = std::string{"/wheel_leg/"} + kWheelNames[i];
            register_input(prefix + "/velocity", wheel_velocity_[i], false);
            register_output(prefix + "/control_torque", wheel_torque_[i], 0.0);
        }

        // Optional cross-check against the production solver.
        register_input("/wheelleg/state/knee_angle", state_knee_, false);
        register_input("/wheelleg/state/leg_angle_l", state_leg_angle_l_, false);
        register_input("/wheelleg/state/leg_angle_r", state_leg_angle_r_, false);
        register_input("/wheelleg/state/leg_length_l", state_leg_length_l_, false);
        register_input("/wheelleg/state/leg_length_r", state_leg_length_r_, false);

        // Attitude (mode 4 pitch/roll/yaw noise characterisation).
        register_input("/wheelleg/state/pitch", state_pitch_, false);
        register_input("/wheelleg/state/roll", state_roll_, false);
        register_input("/wheelleg/state/yaw", state_yaw_, false);

        // Legs actually enabled (status-side output = both DM pairs enabled and
        // ready). Read from the STATUS side, not the partner Command's
        // /wheel_leg/identification/enable_requested, because this component
        // also produces /wheel_leg/enable_request which that Command consumes
        // (reading its output would be a dependency cycle).
        register_input("/wheel_leg/dm_control_ready", legs_enabled_, false);

        register_output("/wheel_leg/enable_request", enable_request_, false);
        register_output("/wheel_leg/identification/wheel_only", wheel_only_, true);

        register_output("/wheelleg/bench/mode", bench_mode_, 0);
        register_output("/wheelleg/bench/inner_knee_l", bench_inner_knee_l_, kNaN);
        register_output("/wheelleg/bench/inner_knee_r", bench_inner_knee_r_, kNaN);
        register_output("/wheelleg/bench/inner_knee_raw_l", bench_inner_knee_raw_l_, kNaN);
        register_output("/wheelleg/bench/inner_knee_raw_r", bench_inner_knee_raw_r_, kNaN);
        register_output("/wheelleg/bench/leg_axle_angle_l", bench_axle_angle_l_, kNaN);
        register_output("/wheelleg/bench/leg_axle_angle_r", bench_axle_angle_r_, kNaN);
        register_output("/wheelleg/bench/leg_length_l", bench_length_l_, kNaN);
        register_output("/wheelleg/bench/leg_length_r", bench_length_r_, kNaN);
        register_output("/wheelleg/bench/leg_angle_l", bench_leg_angle_l_, kNaN);
        register_output("/wheelleg/bench/leg_angle_r", bench_leg_angle_r_, kNaN);
        register_output("/wheelleg/bench/q_hip_l", bench_q_hip_l_, kNaN);
        register_output("/wheelleg/bench/q_hip_r", bench_q_hip_r_, kNaN);
        register_output("/wheelleg/bench/q_crank_l", bench_q_crank_l_, kNaN);
        register_output("/wheelleg/bench/q_crank_r", bench_q_crank_r_, kNaN);
        register_output("/wheelleg/bench/crank_rel_l", bench_crank_rel_l_, kNaN);
        register_output("/wheelleg/bench/crank_rel_r", bench_crank_rel_r_, kNaN);

        // Mode-4 attitude noise: rolling std of pitch/roll/yaw.
        register_output("/wheelleg/bench/pitch_std", bench_pitch_std_, kNaN);
        register_output("/wheelleg/bench/roll_std", bench_roll_std_, kNaN);
        register_output("/wheelleg/bench/yaw_std", bench_yaw_std_, kNaN);
    }

    // -------------------------------------------------------------- wheel loop
    void publish_zero_() {
        for (auto& torque : joint_torque_)
            *torque = 0.0;
        for (auto& torque : wheel_torque_)
            *torque = 0.0;
        *enable_request_ = false;
        *wheel_only_ = true;
    }

    double read_channel_(double value) const {
        if (!std::isfinite(value) || std::abs(value) < deadzone_)
            return 0.0;
        return std::clamp(value, -1.0, 1.0);
    }

    void update_wheel_only_() {
        *wheel_only_ = true;
        *enable_request_ = true;

        // Legs stay disabled by the hardware (wheel_only gate); hold zero torque.
        for (auto& torque : joint_torque_)
            *torque = 0.0;

        const double channel = read_channel_(joystick_right_->x());
        const double target_speed = (vx_max_ / wheel_radius_) * channel; // physical-forward rad/s
        for (std::size_t i = 0; i < 2; ++i) {
            const double measured =
                wheel_velocity_[i].ready() && std::isfinite(*wheel_velocity_[i])
                    ? *wheel_velocity_[i]
                    : 0.0;
            // wheel_command_sign_[i] maps this wheel's physical-forward direction
            // to its API frame (the two mounts are mirrored), so both wheels drive
            // the same physical way for the same stick input.
            const double sign = wheel_command_sign_[i];
            const double torque = sign * wheel_velocity_kp_ * (target_speed - sign * measured);
            *wheel_torque_[i] = std::clamp(torque, -wheel_torque_limit_, wheel_torque_limit_);
        }
    }

    void update_passive_() {
        *enable_request_ = false;
        *wheel_only_ = false;
        for (auto& torque : joint_torque_)
            *torque = 0.0;
        for (auto& torque : wheel_torque_)
            *torque = 0.0;
    }

    // Mode 4: full-body disable; only observe attitude (pitch/roll/yaw). We
    // accumulate mean/std over `stats_period_cycles_` and print/publish the std
    // so IMU noise is visible live.
    struct Accumulator {
        double sum = 0.0, sumsq = 0.0;
        long long n = 0;
        void add(double value) {
            if (std::isfinite(value)) {
                sum += value;
                sumsq += value * value;
                ++n;
            }
        }
        void reset() { sum = sumsq = 0.0; n = 0; }
        double mean() const { return n > 0 ? sum / n : kNaN; }
        double stddev() const {
            if (n < 2)
                return kNaN;
            const double m = sum / n;
            return std::sqrt(std::max(0.0, sumsq / n - m * m));
        }
    };

    void update_imu_test_() {
        *enable_request_ = false;
        *wheel_only_ = false;
        for (auto& torque : joint_torque_)
            *torque = 0.0;
        for (auto& torque : wheel_torque_)
            *torque = 0.0;

        const std::array<InputInterface<double>*, 3> attitude{
            &state_pitch_, &state_roll_, &state_yaw_};
        for (std::size_t i = 0; i < 3; ++i)
            if (attitude[i]->ready())
                attitude_acc_[i].add(**attitude[i]);

        if (++stats_cycles_ < stats_period_cycles_)
            return;
        stats_cycles_ = 0;

        *bench_pitch_std_ = attitude_acc_[0].stddev();
        *bench_roll_std_ = attitude_acc_[1].stddev();
        *bench_yaw_std_ = attitude_acc_[2].stddev();
        const double deg = 180.0 / std::numbers::pi;
        RCLCPP_INFO(
            get_logger(),
            "[attitude] std (n=%lld): pitch=%.4f roll=%.4f yaw=%.4f rad | "
            "pitch=%.4f roll=%.4f yaw=%.4f deg",
            attitude_acc_[0].n, *bench_pitch_std_, *bench_roll_std_, *bench_yaw_std_,
            *bench_pitch_std_ * deg, *bench_roll_std_ * deg, *bench_yaw_std_ * deg);
        RCLCPP_INFO(
            get_logger(), "[attitude] mean: pitch=%.5f roll=%.5f yaw=%.5f rad",
            attitude_acc_[0].mean(), attitude_acc_[1].mean(), attitude_acc_[2].mean());

        for (auto& acc : attitude_acc_)
            acc.reset();
    }

    // Quintic reference ramp (same form as WheelLegTrajectoryController): matched
    // position/velocity at both ends, zero acceleration at both ends.
    struct Quintic {
        void reset(double y0, double dy0, double y1, double dy1, double duration) {
            duration_ = std::max(duration, 1e-6);
            a0_ = y0;
            a1_ = duration_ * dy0;
            const double delta = y1 - y0;
            a3_ = 10.0 * delta - 6.0 * duration_ * dy0 - 4.0 * duration_ * dy1;
            a4_ = -15.0 * delta + 8.0 * duration_ * dy0 + 7.0 * duration_ * dy1;
            a5_ = 6.0 * delta - 3.0 * duration_ * dy0 - 3.0 * duration_ * dy1;
        }
        [[nodiscard]] double value(double t) const {
            const double u = std::clamp(t / duration_, 0.0, 1.0);
            const double u2 = u * u, u3 = u2 * u, u4 = u3 * u, u5 = u4 * u;
            return a0_ + a1_ * u + a3_ * u3 + a4_ * u4 + a5_ * u5;
        }

    private:
        double a0_ = 0, a1_ = 0, a3_ = 0, a4_ = 0, a5_ = 0, duration_ = 1;
    };

    void update_pose_() {
        *wheel_only_ = false;
        *enable_request_ = true;
        for (auto& torque : wheel_torque_)
            *torque = 0.0;

        constexpr double kDeg = 180.0 / std::numbers::pi;

        // Axle line angle (hip->wheel-axle, to horizontal) for a given interior
        // knee beta and reduced hip alpha: alpha_L = alpha_L(beta,0) - alpha.
        const auto axle_line = [&](double beta, double alpha) {
            return model_->axle_line(beta, alpha);
        };

        // Safety: the interior knee goes to an ABSOLUTE target (40..105 deg) from
        // the yaml; the hip->axle line angle only moves by a SMALL relative
        // delta_leg_angle from the MEASURED value. A quintic ramp drives it,
        // started only once the hardware has actually enabled the legs.
        const bool enabled = legs_enabled_.ready() && *legs_enabled_;
        if (!pose_ramp_started_ && enabled) {
            const double beta0 = 0.5 * (*bench_inner_knee_raw_l_ + *bench_inner_knee_raw_r_);
            const double alpha0 = 0.5 * (*bench_leg_angle_l_ + *bench_leg_angle_r_);
            const double b0 = std::isfinite(beta0) ? beta0 : target_inner_knee_;
            const double a0 = std::isfinite(alpha0) ? alpha0 : -std::numbers::pi / 2.0;
            const double alpha_l0 = axle_line(b0, a0);
            const double beta_t =
                std::clamp(target_inner_knee_, knee_limit_min_, knee_limit_max_);
            const double alpha_l_t = alpha_l0 + delta_leg_angle_;
            pose_beta_plan_.reset(b0, 0.0, beta_t, 0.0, ramp_seconds_);
            pose_alpha_l_plan_.reset(alpha_l0, 0.0, alpha_l_t, 0.0, ramp_seconds_);
            pose_ramp_elapsed_ = 0.0;
            pose_ramp_started_ = true;
            RCLCPP_INFO(
                get_logger(),
                "[pose] armed: knee %.2f->%.2f deg, axle-line %.2f->%.2f deg over %.1fs",
                b0 * kDeg, beta_t * kDeg, alpha_l0 * kDeg, alpha_l_t * kDeg, ramp_seconds_);
        }

        double beta_ref = 0.5 * (*bench_inner_knee_raw_l_ + *bench_inner_knee_raw_r_);
        double alpha_ref = 0.5 * (*bench_leg_angle_l_ + *bench_leg_angle_r_);
        if (pose_ramp_started_) {
            pose_ramp_elapsed_ += dt_;
            beta_ref = pose_beta_plan_.value(pose_ramp_elapsed_);
            const double alpha_l = pose_alpha_l_plan_.value(pose_ramp_elapsed_);
            alpha_ref = model_->axle_line(beta_ref, alpha_l);
        }

        for (std::size_t side = 0; side < 2; ++side)
            drive_leg_(side, alpha_ref, beta_ref);
    }

    // Hip->axle line angle (to horizontal) for a given interior knee beta and
    // reduced hip alpha: alpha_L = alpha_L(beta,0) - alpha.
    [[nodiscard]] double axle_line_(double beta, double alpha) const {
        return model_->axle_line(beta, alpha);
    }

    // Drive one side's two coaxial motors to a reduced (alpha, beta) target.
    // The DM encoder is multi-turn, so targets are picked nearest the measured
    // joint to keep the PD error small.
    bool drive_leg_(std::size_t side, double alpha, double beta) {
        double hip_target = 0.0, crank_target = 0.0;
        if (!model_->joint_targets(alpha, beta, leg_calib_[side], hip_target, crank_target))
            return false;
        const double q_hip = joint_angle_[2 * side].ready() ? *joint_angle_[2 * side] : 0.0;
        const double q_crank =
            joint_angle_[2 * side + 1].ready() ? *joint_angle_[2 * side + 1] : 0.0;
        hip_target += 2.0 * std::numbers::pi
            * std::round((q_hip - hip_target) / (2.0 * std::numbers::pi));
        crank_target += 2.0 * std::numbers::pi
            * std::round((q_crank - crank_target) / (2.0 * std::numbers::pi));
        leg_hip_target_[side] = hip_target;
        leg_crank_target_[side] = crank_target;
        apply_joint_pd_(2 * side, hip_target);
        apply_joint_pd_(2 * side + 1, crank_target);
        return true;
    }

    // Mode 5: joystick teleop. Left-stick X integrates the interior knee, left-
    // stick Y integrates the hip->axle line angle, from the measured pose (once
    // enabled). Small per-update sensitivity so it moves slowly at 1 kHz.
    void update_teleop_() {
        *wheel_only_ = false;
        *enable_request_ = true;
        for (auto& torque : wheel_torque_)
            *torque = 0.0;

        const bool enabled = legs_enabled_.ready() && *legs_enabled_;
        const bool measured_valid =
            std::isfinite(*bench_inner_knee_raw_l_) && std::isfinite(*bench_inner_knee_raw_r_)
            && std::isfinite(*bench_leg_angle_l_) && std::isfinite(*bench_leg_angle_r_);
        const auto seed_from_measured = [&] {
            teleop_beta_ = 0.5 * (*bench_inner_knee_raw_l_ + *bench_inner_knee_raw_r_);
            teleop_alpha_l_ =
                axle_line_(teleop_beta_, 0.5 * (*bench_leg_angle_l_ + *bench_leg_angle_r_));
        };

        if (!enabled) {
            // Unarmed: the target tracks the MEASURED pose from power-on, so the
            // first target is the machine's current value, never a default.
            if (measured_valid)
                seed_from_measured();
        } else {
            if (!teleop_started_ && measured_valid) {
                seed_from_measured();
                teleop_started_ = true;
                RCLCPP_INFO(
                    get_logger(),
                    "[teleop] armed: knee=%.1fdeg leg=%.1fdeg (side=%s sens=%.4f rad/step)",
                    teleop_beta_ * 180.0 / std::numbers::pi,
                    teleop_alpha_l_ * 180.0 / std::numbers::pi,
                    teleop_side_ == 0 ? "left" : (teleop_side_ == 1 ? "right" : "both"),
                    teleop_sensitivity_);
            }
            const double knee_axis = read_channel_(joystick_left_->x());
            const double leg_axis = read_channel_(joystick_left_->y());
            teleop_beta_ = std::clamp(
                teleop_beta_ + teleop_sensitivity_ * teleop_knee_sign_ * knee_axis,
                knee_limit_min_, knee_limit_max_);
            teleop_alpha_l_ = std::clamp(
                teleop_alpha_l_ + teleop_sensitivity_ * teleop_leg_sign_ * leg_axis,
                teleop_leg_min_, teleop_leg_max_);
        }

        for (std::size_t side = 0; side < 2; ++side) {
            const bool active = teleop_side_ == 2 || teleop_side_ == static_cast<int>(side);
            double beta_ref, alpha_ref;
            if (active) {
                beta_ref = teleop_beta_;
                alpha_ref = axle_line_(teleop_beta_, 0.0) - teleop_alpha_l_;
            } else {
                // Hold the non-selected leg at its measured pose.
                beta_ref = side == 0 ? *bench_inner_knee_raw_l_ : *bench_inner_knee_raw_r_;
                const double alpha_l = axle_line_(
                    beta_ref, side == 0 ? *bench_leg_angle_l_ : *bench_leg_angle_r_);
                alpha_ref = axle_line_(beta_ref, 0.0) - alpha_l;
            }
            if (std::isfinite(beta_ref) && std::isfinite(alpha_ref))
                drive_leg_(side, alpha_ref, beta_ref);
        }

        if (teleop_cycles_++ % log_every_cycles_ != 0)
            return;
        const double deg = 180.0 / std::numbers::pi;
        const auto q = [&](std::size_t i) {
            return joint_angle_[i].ready() && std::isfinite(*joint_angle_[i]) ? *joint_angle_[i]
                                                                              : kNaN;
        };
        RCLCPP_INFO(
            get_logger(),
            "[teleop] knee=%.1f axle=%.1f deg | L hip tgt=%.3f meas=%.3f err=%.3f | L crank "
            "tgt=%.3f meas=%.3f err=%.3f | R hip tgt=%.3f meas=%.3f err=%.3f | R crank tgt=%.3f "
            "meas=%.3f err=%.3f",
            teleop_beta_ * deg, teleop_alpha_l_ * deg, leg_hip_target_[0], q(0),
            leg_hip_target_[0] - q(0), leg_crank_target_[0], q(1), leg_crank_target_[0] - q(1),
            leg_hip_target_[1], q(2), leg_hip_target_[1] - q(2), leg_crank_target_[1], q(3),
            leg_crank_target_[1] - q(3));
    }

    void apply_joint_pd_(std::size_t index, double target) {
        const double angle = joint_angle_[index].ready() && std::isfinite(*joint_angle_[index])
                               ? *joint_angle_[index]
                               : target;
        const double velocity =
            joint_velocity_[index].ready() && std::isfinite(*joint_velocity_[index])
                ? *joint_velocity_[index]
                : 0.0;
        const double torque = leg_kp_ * (target - angle) - leg_kd_ * velocity;
        *joint_torque_[index] = std::clamp(torque, -leg_torque_limit_, leg_torque_limit_);
    }

    // ------------------------------------------------------------- leg solver
    void solve_leg_shape_() {
        for (std::size_t side = 0; side < 2; ++side) {
            const double q_hip = joint_angle_[2 * side].ready()
                                   ? *joint_angle_[2 * side]
                                   : kNaN;
            const double q_crank = joint_angle_[2 * side + 1].ready()
                                     ? *joint_angle_[2 * side + 1]
                                     : kNaN;
            double inner_knee = kNaN, inner_knee_raw = kNaN, axle_angle = kNaN, leg_length = kNaN,
                   leg_angle = kNaN;
            double crank_rel = kNaN;

            if (std::isfinite(q_hip) && std::isfinite(q_crank)) {
                const LegShape shape = model_->forward(q_hip, q_crank, leg_calib_[side]);
                crank_rel = shape.crank_rel;
                inner_knee_raw = shape.beta_raw;
                inner_knee = shape.beta;
                leg_angle = shape.alpha;
                axle_angle = shape.axle_angle;
                leg_length = shape.leg_length;
            }

            if (side == 0) {
                *bench_inner_knee_l_ = inner_knee;
                *bench_inner_knee_raw_l_ = inner_knee_raw;
                *bench_axle_angle_l_ = axle_angle;
                *bench_length_l_ = leg_length;
                *bench_leg_angle_l_ = leg_angle;
                *bench_q_hip_l_ = q_hip;
                *bench_q_crank_l_ = q_crank;
                *bench_crank_rel_l_ = crank_rel;
            } else {
                *bench_inner_knee_r_ = inner_knee;
                *bench_inner_knee_raw_r_ = inner_knee_raw;
                *bench_axle_angle_r_ = axle_angle;
                *bench_length_r_ = leg_length;
                *bench_leg_angle_r_ = leg_angle;
                *bench_q_hip_r_ = q_hip;
                *bench_q_crank_r_ = q_crank;
                *bench_crank_rel_r_ = crank_rel;
            }
        }
    }

    void publish_leg_shape_() { *bench_mode_ = mode_; }

    void maybe_log_() {
        if (mode_ != 2 || cycle_++ % log_every_cycles_ != 0)
            return;
        const double deg = 180.0 / std::numbers::pi;
        RCLCPP_INFO(
            get_logger(),
            "[passive raw] L: q_hip=%.4f q_crank=%.4f c=%.4f (%.2fdeg) | "
            "R: q_hip=%.4f q_crank=%.4f c=%.4f (%.2fdeg)",
            *bench_q_hip_l_, *bench_q_crank_l_, *bench_crank_rel_l_, *bench_crank_rel_l_ * deg,
            *bench_q_hip_r_, *bench_q_crank_r_, *bench_crank_rel_r_, *bench_crank_rel_r_ * deg);
        RCLCPP_INFO(
            get_logger(),
            "[passive] L: inner_knee=%.2f raw=%.2f deg axle=%.2fdeg len=%.4f | "
            "R: inner_knee=%.2f raw=%.2f deg axle=%.2fdeg len=%.4f  "
            "(est knee=%.2fdeg legL=%.2fdeg legR=%.2fdeg)",
            *bench_inner_knee_l_ * deg, *bench_inner_knee_raw_l_ * deg, *bench_axle_angle_l_ * deg,
            *bench_length_l_, *bench_inner_knee_r_ * deg, *bench_inner_knee_raw_r_ * deg,
            *bench_axle_angle_r_ * deg, *bench_length_r_,
            state_knee_.ready() ? *state_knee_ * deg : kNaN,
            state_leg_angle_l_.ready() ? *state_leg_angle_l_ * deg : kNaN,
            state_leg_angle_r_.ready() ? *state_leg_angle_r_ * deg : kNaN);
    }

    // ------------------------------------------------------------------ fields
    LegLinkage linkage_;
    std::unique_ptr<LegModel> model_;
    std::array<LegCalibration, 2> leg_calib_{};

    int mode_ = 1;
    double thigh_length_ = 0.21, shank_length_ = 0.25045, wheel_radius_ = 0.06;
    double knee_limit_min_ = 0.6981317007977318, knee_limit_max_ = 1.8325957145940461;
    std::vector<double> hardware_hip_sign_{1.0, 1.0};
    std::vector<double> hardware_hip_zero_{0.0, 0.0};
    std::vector<double> hardware_crank_sign_{1.0, 1.0};

    double wheel_velocity_kp_ = 0.3, wheel_torque_limit_ = 4.88;
    double vx_max_ = 0.5, deadzone_ = 0.08;
    std::vector<double> wheel_command_sign_{1.0, 1.0}; // [left, right] physical-forward sign
    double leg_kp_ = 160.0, leg_kd_ = 2.5, leg_torque_limit_ = 40.0;
    double target_inner_knee_ = 1.5708;
    double ramp_seconds_ = 3.0, dt_ = 0.001;
    double delta_leg_angle_ = 0.0;
    bool pose_ramp_started_ = false;
    double pose_ramp_elapsed_ = 0.0;
    Quintic pose_alpha_l_plan_, pose_beta_plan_;
    int teleop_side_ = 2;
    double teleop_sensitivity_ = 0.006;
    double teleop_knee_sign_ = 1.0, teleop_leg_sign_ = 1.0;
    double teleop_leg_min_ = -2.967, teleop_leg_max_ = -0.349;
    bool teleop_started_ = false;
    // Seeded from the MEASURED pose on the first valid sample (these defaults
    // are only a placeholder that is overwritten before any command is applied).
    double teleop_beta_ = 0.0, teleop_alpha_l_ = -1.5708;
    // Mode-5 debug: last commanded joint targets (per side), for the print.
    std::array<double, 2> leg_hip_target_{kNaN, kNaN}, leg_crank_target_{kNaN, kNaN};
    long long teleop_cycles_ = 0;
    int log_every_cycles_ = 200;
    long long cycle_ = 0;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    std::array<InputInterface<double>, kNumJoints> joint_angle_, joint_velocity_;
    std::array<InputInterface<double>, 2> wheel_velocity_;
    InputInterface<double> state_knee_, state_leg_angle_l_, state_leg_angle_r_,
        state_leg_length_l_, state_leg_length_r_;

    InputInterface<double> state_pitch_, state_roll_, state_yaw_;
    InputInterface<bool> legs_enabled_;
    std::array<Accumulator, 3> attitude_acc_;
    int stats_period_cycles_ = 1000;
    long long stats_cycles_ = 0;

    std::array<OutputInterface<double>, kNumJoints> joint_torque_;
    std::array<OutputInterface<double>, 2> wheel_torque_;
    OutputInterface<bool> enable_request_, wheel_only_;
    OutputInterface<int> bench_mode_;
    OutputInterface<double> bench_inner_knee_l_, bench_inner_knee_r_;
    OutputInterface<double> bench_inner_knee_raw_l_, bench_inner_knee_raw_r_;
    OutputInterface<double> bench_axle_angle_l_, bench_axle_angle_r_;
    OutputInterface<double> bench_length_l_, bench_length_r_;
    OutputInterface<double> bench_leg_angle_l_, bench_leg_angle_r_;
    OutputInterface<double> bench_q_hip_l_, bench_q_hip_r_;
    OutputInterface<double> bench_q_crank_l_, bench_q_crank_r_;
    OutputInterface<double> bench_crank_rel_l_, bench_crank_rel_r_;
    OutputInterface<double> bench_pitch_std_, bench_roll_std_, bench_yaw_std_;
};

} // namespace rmcs_core::controller::wheelleg

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::wheelleg::WheelLegBenchTestController, rmcs_executor::Component)
