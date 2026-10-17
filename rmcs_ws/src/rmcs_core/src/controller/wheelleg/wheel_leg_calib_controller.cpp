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
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "controller/wheelleg/leg_transmission.hpp"
#include "controller/wheelleg/wheelleg_leg_model.hpp"

namespace rmcs_core::controller::wheelleg {

using LegLinkage = rmcs_core::wheelleg::LegLinkage;
using LegModel = rmcs_core::wheelleg::WheelLegLegModel;
using LegShape = rmcs_core::wheelleg::LegShape;
using LegCalibration = rmcs_core::wheelleg::LegCalibration;

// Calibration / commissioning modes for the REAL wheel-leg, living INSIDE the
// stand graph so the bench calibration and the standing test share one yaml,
// one state estimator and one hardware owner.
//
// The drive bridge is the only component that writes /wheel_leg/*/control_torque,
// so this component never touches the hardware directly: it computes the joint /
// wheel torques and publishes them on /wheelleg/calib/*; the drive bridge passes
// them through when /wheelleg/calib/active is true.
//
// mode 0  off        : inactive; the normal stand chain runs.
// mode 1  wheel-only : legs zero torque, right-stick X -> common wheel speed P
//                      loop (same as the bench mode 1); measures the wheel /
//                      forward sign. request_enable = true.
// mode 2  passive    : request_enable = false; pose the legs by hand and read the
//                      solved leg shape (interior knee, hip->axle angle, length).
// mode 4  attitude   : request_enable = false; rolling pitch/roll/yaw std.
// mode 5  teleop     : left-stick X/Y jog the interior knee / hip->axle angle from
//                      the measured pose (same as the bench mode 5), host leg PD.
//                      request_enable = true.
//
// While mode != 0 this component publishes /wheelleg/hold = true, so the
// trajectory controller freezes its reference and re-plans from the measured
// pose when the mode returns to 0. request_enable is false for the passive modes,
// so the drive bridge keeps the hardware disabled.
class WheelLegCalibController
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    using node = rmcs_utility::NodeMixin;

    WheelLegCalibController()
        : Node{get_component_name(), node::options()} {
        load_config_();
        register_interfaces_();
        publish_zero_();
        RCLCPP_INFO(
            get_logger(),
            "wheelleg_calib: mode=%d (0=off, 1=wheel, 2=passive, 4=imu, 5=teleop)",
            mode_);
    }

    void update() override {
        solve_leg_shape_();
        *calib_mode_ = mode_;
        *calib_active_ = mode_ != 0;
        *calib_request_enable_ = false;
        for (auto& torque : joint_torque_)
            *torque = 0.0;
        for (auto& torque : wheel_torque_)
            *torque = 0.0;

        switch (mode_) {
        case 1: update_wheel_only_(); break;
        case 2: update_passive_(); break;
        case 4: update_imu_test_(); break;
        case 5: update_teleop_(); break;
        default: break; // 0 = off
        }

        // Freeze the production trajectory while calibrating; it re-plans from
        // the measured pose when mode returns to 0.
        *hold_ = mode_ != 0;
        // Mode 1: legs stay disabled (wheel-only), exactly like bench mode 1.
        *wheel_only_ = mode_ == 1;
        maybe_log_();
    }

private:
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr std::size_t kNumJoints = 4; // [Lhip, Lknee(crank), Rhip, Rknee(crank)]

    void load_config_() {
        mode_ = get_parameter_or("mode", 0);
        if (!(mode_ == 0 || mode_ == 1 || mode_ == 2 || mode_ == 4 || mode_ == 5))
            throw std::invalid_argument(
                "wheelleg_calib mode must be 0 (off), 1 (wheel), 2 (passive), 4 (imu) "
                "or 5 (teleop)");

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
            throw std::invalid_argument("hardware_hip_sign/zero/crank_sign require two values");

        wheel_velocity_kp_ = get_parameter_or("wheel_velocity_kp", 0.3);
        wheel_torque_limit_ = get_parameter_or("wheel_torque_limit", 4.88);
        vx_max_ = get_parameter_or("vx_max", 0.5);
        deadzone_ = get_parameter_or("deadzone", 0.08);
        wheel_command_sign_ =
            get_parameter_or("wheel_command_sign", std::vector<double>{1.0, 1.0});
        if (wheel_command_sign_.size() != 2)
            throw std::invalid_argument("wheel_command_sign requires two values [left, right]");

        leg_kp_ = get_parameter_or("leg_kp", 160.0);
        leg_kd_ = get_parameter_or("leg_kd", 2.5);
        leg_torque_limit_ = get_parameter_or("leg_torque_limit", 40.0);
        dt_ = get_parameter_or("dt", 0.001);

        teleop_side_ = get_parameter_or("teleop_side", 2); // 0=left, 1=right, 2=both
        teleop_sensitivity_ = get_parameter_or("teleop_sensitivity", 0.003);
        teleop_knee_sign_ = get_parameter_or("teleop_knee_sign", 1.0);
        teleop_leg_sign_ = get_parameter_or("teleop_leg_sign", 1.0);
        const double deg = std::numbers::pi / 180.0;
        teleop_leg_min_ = get_parameter_or("teleop_leg_min_deg", -170.0) * deg;
        teleop_leg_max_ = get_parameter_or("teleop_leg_max_deg", -20.0) * deg;
        teleop_ramp_s_ = std::max(0.0, get_parameter_or("teleop_ramp_s", 0.3));
        // PRINT-ONLY standard stand-pose reference (fixed nominal). Used by the
        // DISARMED log to show the measured pose relative to the standard stance,
        // so the forward/inverse solving can be checked without enabling.
        stand_ref_leg_length_ = get_parameter_or("stand_ref_leg_length", 0.334);
        stand_ref_alpha_l_ = get_parameter_or("stand_ref_alpha_l_deg", -90.0) * deg;
        if (teleop_side_ < 0 || teleop_side_ > 2)
            throw std::invalid_argument("teleop_side must be 0 (left), 1 (right) or 2 (both)");
        if (dt_ <= 0.0)
            throw std::invalid_argument("dt must be positive");

        log_every_cycles_ = std::max<int>(1, get_parameter_or("log_every_cycles", 200));
        stats_period_cycles_ = std::max<int>(1, get_parameter_or("stats_period_cycles", 1000));

        model_ = std::make_unique<LegModel>(
            thigh_length_, shank_length_, knee_limit_min_, knee_limit_max_, linkage_);
        leg_calib_[0] =
            LegCalibration{hardware_hip_sign_[0], hardware_hip_zero_[0], hardware_crank_sign_[0]};
        leg_calib_[1] =
            LegCalibration{hardware_hip_sign_[1], hardware_hip_zero_[1], hardware_crank_sign_[1]};
        // Fixed standard-stand reference (print only).
        stand_ref_beta_ = model_->knee_from_leg_length(stand_ref_leg_length_);
    }

    void register_interfaces_() {
        register_input("/remote/joystick/right", joystick_right_, false);
        register_input("/remote/joystick/left", joystick_left_, false);

        constexpr std::array kJointTopics{
            "left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint"};
        for (std::size_t i = 0; i < kNumJoints; ++i) {
            const std::string hw = std::string{"/wheel_leg/"} + kJointTopics[i];
            register_input(hw + "/angle", joint_angle_[i], false);
            register_input(hw + "/velocity", joint_velocity_[i], false);
            const std::string out = std::string{"/wheelleg/calib/"} + kJointTopics[i] + "/torque";
            register_output(out, joint_torque_[i], 0.0);
        }
        constexpr std::array kWheelNames{"left_wheel", "right_wheel"};
        for (std::size_t i = 0; i < 2; ++i) {
            const std::string hw = std::string{"/wheel_leg/"} + kWheelNames[i];
            register_input(hw + "/velocity", wheel_velocity_[i], false);
            const std::string out = std::string{"/wheelleg/calib/"} + kWheelNames[i] + "/torque";
            register_output(out, wheel_torque_[i], 0.0);
        }

        // Production estimator cross-check (mode 2) and attitude (mode 4).
        register_input("/wheelleg/state/knee_angle", state_knee_, false);
        register_input("/wheelleg/state/leg_angle_l", state_leg_angle_l_, false);
        register_input("/wheelleg/state/leg_angle_r", state_leg_angle_r_, false);
        register_input("/wheelleg/state/leg_length_l", state_leg_length_l_, false);
        register_input("/wheelleg/state/leg_length_r", state_leg_length_r_, false);
        register_input("/wheelleg/state/pitch", state_pitch_, false);
        register_input("/wheelleg/state/roll", state_roll_, false);
        register_input("/wheelleg/state/yaw", state_yaw_, false);
        // Legs actually enabled (hardware status).
        register_input("/wheel_leg/dm_control_ready", legs_enabled_, false);

        register_output("/wheelleg/calib/mode", calib_mode_, 0);
        register_output("/wheelleg/calib/active", calib_active_, false);
        register_output("/wheelleg/calib/request_enable", calib_request_enable_, false);
        // Reuse the trajectory controller's start-pose gate.
        register_output("/wheelleg/hold", hold_, false);
        // Mode 1 must leave the LEG drives disabled (same as bench mode 1): the
        // wheel-only gate makes the hardware enable only the wheel motors.
        register_output("/wheel_leg/identification/wheel_only", wheel_only_, false);

        register_output("/wheelleg/calib/inner_knee_l", bench_inner_knee_l_, kNaN);
        register_output("/wheelleg/calib/inner_knee_r", bench_inner_knee_r_, kNaN);
        register_output("/wheelleg/calib/inner_knee_raw_l", bench_inner_knee_raw_l_, kNaN);
        register_output("/wheelleg/calib/inner_knee_raw_r", bench_inner_knee_raw_r_, kNaN);
        register_output("/wheelleg/calib/leg_axle_angle_l", bench_axle_angle_l_, kNaN);
        register_output("/wheelleg/calib/leg_axle_angle_r", bench_axle_angle_r_, kNaN);
        register_output("/wheelleg/calib/leg_length_l", bench_length_l_, kNaN);
        register_output("/wheelleg/calib/leg_length_r", bench_length_r_, kNaN);
        register_output("/wheelleg/calib/leg_angle_l", bench_leg_angle_l_, kNaN);
        register_output("/wheelleg/calib/leg_angle_r", bench_leg_angle_r_, kNaN);
        register_output("/wheelleg/calib/crank_rel_l", bench_crank_rel_l_, kNaN);
        register_output("/wheelleg/calib/crank_rel_r", bench_crank_rel_r_, kNaN);
        register_output("/wheelleg/calib/pitch_std", bench_pitch_std_, kNaN);
        register_output("/wheelleg/calib/roll_std", bench_roll_std_, kNaN);
        register_output("/wheelleg/calib/yaw_std", bench_yaw_std_, kNaN);
    }

    void publish_zero_() {
        for (auto& torque : joint_torque_)
            *torque = 0.0;
        for (auto& torque : wheel_torque_)
            *torque = 0.0;
        *calib_active_ = false;
        *calib_request_enable_ = false;
        *hold_ = false;
        *wheel_only_ = false;
    }

    double read_channel_(double value) const {
        if (!std::isfinite(value) || std::abs(value) < deadzone_)
            return 0.0;
        return std::clamp(value, -1.0, 1.0);
    }
    double joystick_x_(const InputInterface<Eigen::Vector2d>& joy) const {
        return joy.ready() ? joy->x() : 0.0;
    }
    double joystick_y_(const InputInterface<Eigen::Vector2d>& joy) const {
        return joy.ready() ? joy->y() : 0.0;
    }

    // ---- mode 1: wheel-only P loop (identical to bench mode 1) --------------
    void update_wheel_only_() {
        *calib_request_enable_ = true;
        const double channel = read_channel_(joystick_x_(joystick_right_));
        const double target_speed = (vx_max_ / wheel_radius_) * channel; // physical-forward rad/s
        for (std::size_t i = 0; i < 2; ++i) {
            const double measured =
                wheel_velocity_[i].ready() && std::isfinite(*wheel_velocity_[i])
                    ? *wheel_velocity_[i]
                    : 0.0;
            const double sign = wheel_command_sign_[i];
            const double torque = sign * wheel_velocity_kp_ * (target_speed - sign * measured);
            *wheel_torque_[i] = std::clamp(torque, -wheel_torque_limit_, wheel_torque_limit_);
        }
    }

    // ---- mode 2: passive ----------------------------------------------------
    void update_passive_() { *calib_request_enable_ = false; }

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

    // ---- mode 4: attitude noise --------------------------------------------
    void update_imu_test_() {
        *calib_request_enable_ = false;
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
            "[calib imu] std: pitch=%.4f roll=%.4f yaw=%.4f deg (mean %.5f/%.5f/%.5f rad)",
            *bench_pitch_std_ * deg, *bench_roll_std_ * deg, *bench_yaw_std_ * deg,
            attitude_acc_[0].mean(), attitude_acc_[1].mean(), attitude_acc_[2].mean());
        for (auto& acc : attitude_acc_)
            acc.reset();
    }

    // Quintic: matched position/velocity at both ends, zero accel at both ends.
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

    // Per-side teleop state. NO left/right averaging: each side is seeded from
    // its own measured pose and planned to its own target with a quintic on the
    // arm edge, then follows the target (option A).
    struct TeleopSide {
        double beta_tgt = 0.0, alpha_l_tgt = 0.0; // target (joystick integral)
        double beta_ref = 0.0, alpha_l_ref = 0.0; // smoother output
        Quintic beta_plan, alpha_l_plan;
        double elapsed = 0.0;
        bool started = false; // arm edge seen and seeded
        bool done = false;    // arm ramp finished -> follow the target
    };

    // ---- mode 5: joystick teleop (per-side, quintic from current to target) --
    void update_teleop_() {
        *calib_request_enable_ = true;
        const bool enabled = legs_enabled_.ready() && *legs_enabled_;
        teleop_enabled_ = enabled;
        const double knee_axis = read_channel_(joystick_x_(joystick_left_));
        const double leg_axis = read_channel_(joystick_y_(joystick_left_));
        const double d_beta = teleop_sensitivity_ * teleop_knee_sign_ * knee_axis;
        const double d_alpha_l = teleop_sensitivity_ * teleop_leg_sign_ * leg_axis;

        bool logged_armed = false;
        for (std::size_t side = 0; side < 2; ++side) {
            TeleopSide& t = teleop_[side];
            const double meas_beta =
                side == 0 ? *bench_inner_knee_raw_l_ : *bench_inner_knee_raw_r_;
            const double meas_alpha = side == 0 ? *bench_leg_angle_l_ : *bench_leg_angle_r_;
            const bool valid = std::isfinite(meas_beta) && std::isfinite(meas_alpha);
            const bool active = teleop_side_ == 2 || teleop_side_ == static_cast<int>(side);

            if (!enabled) {
                // Track the measured pose (no command) and drop the arm state, so
                // the next arm seeds from the pose at release.
                if (valid) {
                    t.beta_tgt = t.beta_ref = meas_beta;
                    t.alpha_l_tgt = t.alpha_l_ref = axle_line_(meas_beta, meas_alpha);
                }
                t.started = false;
                t.done = false;
                t.elapsed = 0.0;
                continue;
            }

            if (!t.started) {
                if (!valid)
                    continue; // wait for a valid measurement, never drive a default
                // Arm edge: plan from the MEASURED pose to the target. The target
                // is seeded to the measured pose, so the ramp starts flat (no
                // jump); it becomes meaningful if the target is pre-set.
                t.beta_tgt = t.beta_ref = meas_beta;
                t.alpha_l_tgt = t.alpha_l_ref = axle_line_(meas_beta, meas_alpha);
                t.beta_plan.reset(t.beta_ref, 0.0, t.beta_tgt, 0.0, teleop_ramp_s_);
                t.alpha_l_plan.reset(t.alpha_l_ref, 0.0, t.alpha_l_tgt, 0.0, teleop_ramp_s_);
                t.elapsed = 0.0;
                t.started = true;
                t.done = teleop_ramp_s_ <= 0.0;
                if (!logged_armed) {
                    logged_armed = true;
                    RCLCPP_INFO(
                        get_logger(),
                        "[calib teleop] armed: L knee=%.1f R knee=%.1f deg ramp=%.2fs "
                        "(side=%s sens=%.4f rad/step)",
                        (*bench_inner_knee_raw_l_) * 180.0 / std::numbers::pi,
                        (*bench_inner_knee_raw_r_) * 180.0 / std::numbers::pi, teleop_ramp_s_,
                        teleop_side_ == 0 ? "left" : (teleop_side_ == 1 ? "right" : "both"),
                        teleop_sensitivity_);
                }
            }

            if (!t.done) {
                // Arm ramp: target fixed; reference follows the quintic.
                t.elapsed += dt_;
                t.beta_ref = t.beta_plan.value(t.elapsed);
                t.alpha_l_ref = t.alpha_l_plan.value(t.elapsed);
                if (t.elapsed >= teleop_ramp_s_) {
                    t.done = true;
                    t.beta_ref = t.beta_tgt;
                    t.alpha_l_ref = t.alpha_l_tgt;
                }
            } else {
                // Follow the integrated target (only the selected side moves).
                if (active) {
                    t.beta_tgt =
                        std::clamp(t.beta_tgt + d_beta, knee_limit_min_, knee_limit_max_);
                    t.alpha_l_tgt =
                        std::clamp(t.alpha_l_tgt + d_alpha_l, teleop_leg_min_, teleop_leg_max_);
                }
                t.beta_ref = t.beta_tgt;
                t.alpha_l_ref = t.alpha_l_tgt;
            }

            const double beta_ref = t.beta_ref;
            const double alpha_ref = axle_line_(beta_ref, 0.0) - t.alpha_l_ref;
            if (std::isfinite(beta_ref) && std::isfinite(alpha_ref))
                drive_leg_(side, alpha_ref, beta_ref);
        }
    }

    [[nodiscard]] double axle_line_(double beta, double alpha) const {
        return model_->axle_line(beta, alpha);
    }

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

    // ---- shared leg solver (same conventions as the estimator) -------------
    void solve_leg_shape_() {
        for (std::size_t side = 0; side < 2; ++side) {
            const double q_hip =
                joint_angle_[2 * side].ready() ? *joint_angle_[2 * side] : kNaN;
            const double q_crank =
                joint_angle_[2 * side + 1].ready() ? *joint_angle_[2 * side + 1] : kNaN;
            double inner_knee = kNaN, inner_knee_raw = kNaN, axle_angle = kNaN, leg_length = kNaN,
                   leg_angle = kNaN, crank_rel = kNaN;

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
                *bench_crank_rel_l_ = crank_rel;
            } else {
                *bench_inner_knee_r_ = inner_knee;
                *bench_inner_knee_raw_r_ = inner_knee_raw;
                *bench_axle_angle_r_ = axle_angle;
                *bench_length_r_ = leg_length;
                *bench_leg_angle_r_ = leg_angle;
                *bench_crank_rel_r_ = crank_rel;
            }
        }
    }

    // Print the current measured pose / nominal joint targets against the FIXED
    // standard stand pose (stand_ref_leg_length / stand_ref_alpha_l_deg).
    // PRINT ONLY: nothing here is fed to joint_targets/PD.
    void log_stand_ref_() {
        constexpr double kRadToDeg = 180.0 / std::numbers::pi;
        const double beta_ref = stand_ref_beta_;
        const double alpha_l_ref = stand_ref_alpha_l_;
        if (!std::isfinite(beta_ref) || !std::isfinite(alpha_l_ref))
            return;
        const double alpha_ref = axle_line_(beta_ref, 0.0) - alpha_l_ref;
        for (std::size_t s = 0; s < 2; ++s) {
            const double meas_beta =
                (s == 0) ? *bench_inner_knee_raw_l_ : *bench_inner_knee_raw_r_;
            const double meas_alpha_l =
                (s == 0) ? *bench_axle_angle_l_ : *bench_axle_angle_r_;
            double hip_tgt = kNaN, crank_tgt = kNaN;
            if (!model_->joint_targets(alpha_ref, beta_ref, leg_calib_[s], hip_tgt, crank_tgt))
                continue;
            const double qh = joint_angle_[2 * s].ready() && std::isfinite(*joint_angle_[2 * s])
                                ? *joint_angle_[2 * s]
                                : kNaN;
            const double qc = joint_angle_[2 * s + 1].ready()
                                && std::isfinite(*joint_angle_[2 * s + 1])
                                ? *joint_angle_[2 * s + 1]
                                : kNaN;
            hip_tgt += 2.0 * std::numbers::pi
                     * std::round((qh - hip_tgt) / (2.0 * std::numbers::pi));
            crank_tgt += 2.0 * std::numbers::pi
                       * std::round((qc - crank_tgt) / (2.0 * std::numbers::pi));
            RCLCPP_INFO(
                get_logger(),
                "[calib STAND REF] %s: beta ref/meas/err=%.1f/%.1f/%.1f deg | "
                "axle ref/meas/err=%.1f/%.1f/%.1f deg | hip tgt/meas/err=%.3f/%.3f/%.3f rad | "
                "crank tgt/meas/err=%.3f/%.3f/%.3f rad",
                s == 0 ? "L" : "R", beta_ref * kRadToDeg, meas_beta * kRadToDeg,
                (beta_ref - meas_beta) * kRadToDeg, alpha_l_ref * kRadToDeg,
                meas_alpha_l * kRadToDeg, (alpha_l_ref - meas_alpha_l) * kRadToDeg, hip_tgt, qh,
                hip_tgt - qh, crank_tgt, qc, crank_tgt - qc);
        }
    }

    void maybe_log_() {
        const bool log_now = (cycle_++ % log_every_cycles_ == 0);
        if (!log_now)
            return;
        if (mode_ == 2) {
            const double deg = 180.0 / std::numbers::pi;
            const auto q_hip = [&](std::size_t i) {
                return joint_angle_[i].ready() && std::isfinite(*joint_angle_[i])
                         ? *joint_angle_[i]
                         : kNaN;
            };
            RCLCPP_INFO(
                get_logger(),
                "[calib passive] L0 L/R=%.4f/%.4f m | knee_raw L/R=%.2f/%.2f deg | axle "
                "L/R=%.2f/%.2f deg | q_hip L/R=%.2f/%.2f deg",
                *bench_length_l_, *bench_length_r_, *bench_inner_knee_raw_l_ * deg,
                *bench_inner_knee_raw_r_ * deg, *bench_axle_angle_l_ * deg,
                *bench_axle_angle_r_ * deg, q_hip(0) * deg, q_hip(2) * deg);
        } else if (mode_ == 1) {
            const double channel = read_channel_(joystick_x_(joystick_right_));
            const double cmd_fwd = (vx_max_ / wheel_radius_) * channel; // physical-forward
            const double vl = wheel_velocity_[0].ready() ? *wheel_velocity_[0] : kNaN;
            const double vr = wheel_velocity_[1].ready() ? *wheel_velocity_[1] : kNaN;
            RCLCPP_INFO(
                get_logger(),
                "[calib wheel] fwd_cmd=%.2f rad/s | L fwd=%.2f (api %.2f) | R fwd=%.2f (api "
                "%.2f) | torq L=%.2f R=%.2f",
                cmd_fwd, wheel_command_sign_[0] * vl, vl, wheel_command_sign_[1] * vr, vr,
                *wheel_torque_[0], *wheel_torque_[1]);
        } else if (mode_ == 5) {
            const double deg = 180.0 / std::numbers::pi;
            const auto q = [&](std::size_t i) -> double {
                return joint_angle_[i].ready() && std::isfinite(*joint_angle_[i])
                         ? *joint_angle_[i]
                         : kNaN;
            };
            if (!teleop_enabled_) {
                log_stand_ref_();
                return;
            }
            RCLCPP_INFO(
                get_logger(),
                "[calib teleop] L knee tgt/ref/meas=%.1f/%.1f/%.1f | axle tgt=%.1f | "
                "R knee tgt/ref/meas=%.1f/%.1f/%.1f | axle tgt=%.1f deg | "
                "hip err L/R=%.2f/%.2f crank err L/R=%.2f/%.2f deg",
                teleop_[0].beta_tgt * deg, teleop_[0].beta_ref * deg,
                *bench_inner_knee_raw_l_ * deg, teleop_[0].alpha_l_tgt * deg,
                teleop_[1].beta_tgt * deg, teleop_[1].beta_ref * deg,
                *bench_inner_knee_raw_r_ * deg, teleop_[1].alpha_l_tgt * deg,
                (leg_hip_target_[0] - q(0)) * deg, (leg_hip_target_[1] - q(2)) * deg,
                (leg_crank_target_[0] - q(1)) * deg, (leg_crank_target_[1] - q(3)) * deg);
        } else if (mode_ == 0) {
            // Mode 0: same "both-down" standard-pose check as mode 5. Print only
            // while the drives are disabled (not armed); when armed the stand
            // chain's own trajectory/drive-bridge logs take over.
            const bool enabled = legs_enabled_.ready() && *legs_enabled_;
            if (!enabled)
                log_stand_ref_();
        }
    }

    // ---- fields ------------------------------------------------------------
    LegLinkage linkage_;
    std::unique_ptr<LegModel> model_;
    std::array<LegCalibration, 2> leg_calib_{};

    int mode_ = 0;
    double thigh_length_ = 0.21, shank_length_ = 0.25045, wheel_radius_ = 0.06;
    double knee_limit_min_ = 0.6981317007977318, knee_limit_max_ = 1.8325957145940461;
    std::vector<double> hardware_hip_sign_{1.0, 1.0};
    std::vector<double> hardware_hip_zero_{0.0, 0.0};
    std::vector<double> hardware_crank_sign_{1.0, 1.0};

    double wheel_velocity_kp_ = 0.3, wheel_torque_limit_ = 4.88;
    double vx_max_ = 0.5, deadzone_ = 0.08;
    std::vector<double> wheel_command_sign_{1.0, 1.0};
    double leg_kp_ = 160.0, leg_kd_ = 2.5, leg_torque_limit_ = 40.0;
    double dt_ = 0.001;

    int teleop_side_ = 2;
    double teleop_sensitivity_ = 0.003;
    double teleop_knee_sign_ = 1.0, teleop_leg_sign_ = 1.0;
    double teleop_leg_min_ = -2.967, teleop_leg_max_ = -0.349;
    double teleop_ramp_s_ = 0.3;
    // Per-side teleop target/reference/smoother (no left/right averaging).
    std::array<TeleopSide, 2> teleop_{};
    bool teleop_enabled_ = false;
    // PRINT-ONLY standard stand-pose reference (fixed nominal from the params).
    // Never used by the control path (targets/seed/PD still use the measured).
    double stand_ref_leg_length_ = 0.334;
    double stand_ref_alpha_l_ = -1.5707963267948966;
    double stand_ref_beta_ = kNaN;
    // Last commanded joint targets (per side) for the mode-5 print.
    std::array<double, 2> leg_hip_target_{kNaN, kNaN}, leg_crank_target_{kNaN, kNaN};

    std::array<Accumulator, 3> attitude_acc_;
    int stats_period_cycles_ = 1000;
    long long stats_cycles_ = 0;
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

    std::array<OutputInterface<double>, kNumJoints> joint_torque_;
    std::array<OutputInterface<double>, 2> wheel_torque_;
    OutputInterface<int> calib_mode_;
    OutputInterface<bool> calib_active_, calib_request_enable_, hold_, wheel_only_;
    OutputInterface<double> bench_inner_knee_l_, bench_inner_knee_r_;
    OutputInterface<double> bench_inner_knee_raw_l_, bench_inner_knee_raw_r_;
    OutputInterface<double> bench_axle_angle_l_, bench_axle_angle_r_;
    OutputInterface<double> bench_length_l_, bench_length_r_;
    OutputInterface<double> bench_leg_angle_l_, bench_leg_angle_r_;
    OutputInterface<double> bench_crank_rel_l_, bench_crank_rel_r_;
    OutputInterface<double> bench_pitch_std_, bench_roll_std_, bench_yaw_std_;
};

} // namespace rmcs_core::controller::wheelleg

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::wheelleg::WheelLegCalibController, rmcs_executor::Component)
