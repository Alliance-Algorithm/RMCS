#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <numbers>
#include <stdexcept>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"
#include "deformable_joint_layer.hpp"

namespace rmcs_core::controller::chassis {

enum class SuspensionPhase : uint8_t { kInactive, kArming, kActive, kReleasing };

struct AttitudeBias {
    double pitch_force = 0.0;
    double roll_force = 0.0;
};

struct LegControlState {
    SuspensionPhase phase = SuspensionPhase::kInactive;
    double support_force = 0.0;
    double contact_confidence = 1.0;
    double filtered_contact_confidence = 1.0;
    double phase_elapsed = 0.0;
    bool requested_deploy = false;
    bool output_active = false;
    bool contact_latched = false;
};

struct LegCommand {
    double requested_target_angle = std::numeric_limits<double>::quiet_NaN();
    double final_target_angle = std::numeric_limits<double>::quiet_NaN();
    double target_velocity = 0.0;
    double target_acceleration = 0.0;
    bool suspension_mode = false;
    double suspension_torque = std::numeric_limits<double>::quiet_NaN();
};

struct AttitudePidAxis {
    double kp = 20.0;
    double ki = 0.0;
    double kd = 0.0;
    double integral = 0.0;
    double integral_limit = std::numeric_limits<double>::infinity();
    double output_limit = std::numeric_limits<double>::infinity();

    void reset() { integral = 0.0; }

    double update(double error, double rate, double dt) {
        if (!std::isfinite(error) || !std::isfinite(rate) || !std::isfinite(dt) || dt <= 0.0) {
            reset();
            return std::numeric_limits<double>::quiet_NaN();
        }
        integral = std::clamp(integral + error * dt, -integral_limit, integral_limit);
        return std::clamp(kp * error + ki * integral - kd * rate, -output_limit, output_limit);
    }
};

struct SuspensionParams {
    double mass, rod_length, Kz, Kp, pitch_ki, Dp, Kr, roll_ki, Dr, D_leg;
    double com_height, wheel_base_half_x, wheel_base_half_y;
    double gravity_comp_gain, control_acceleration_limit;
    double preload_angle, entry_offset, ride_height_offset, hold_travel;
    double activation_velocity_threshold;
    double target_physical_velocity_limit, target_physical_acceleration_limit;
    double torque_limit;
    double pitch_angle_diff_limit, roll_angle_diff_limit, pid_integral_limit;
};

// --- ChassisVelocityControl ---
struct ChassisVelocityControl {
    static constexpr double kTranslationalVelocityMax = 10.0;
    static constexpr double kAngularVelocityMax = 10.0;

    void configure(double spin_ratio) {
        spin_ratio_ = std::clamp(spin_ratio, 0.0, 1.0);
        following_velocity_controller_.output_max = kAngularVelocityMax;
        following_velocity_controller_.output_min = -kAngularVelocityMax;
    }

    void set_spin_forward(bool forward) { spinning_forward_ = forward; }

    Eigen::Vector2d compute_translational(
        const Eigen::Vector2d& joystick_right, const rmcs_msgs::Keyboard& keyboard,
        double gimbal_yaw_angle) {
        Eigen::Vector2d tv =
            Eigen::Rotation2Dd{gimbal_yaw_angle}
            * (joystick_right + Eigen::Vector2d{keyboard.w - keyboard.s, keyboard.a - keyboard.d});
        if (tv.norm() > 1.0)
            tv.normalize();
        return tv * kTranslationalVelocityMax;
    }

    struct AngularResult {
        double angular_velocity = 0.0;
        double chassis_angle = std::numeric_limits<double>::quiet_NaN();
        double chassis_control_angle = std::numeric_limits<double>::quiet_NaN();
    };

    AngularResult compute_angular(
        rmcs_msgs::ChassisMode mode, double gimbal_yaw_angle, double gimbal_yaw_angle_error,
        bool apply_toggle_forward) {
        AngularResult result;
        switch (mode) {
        case rmcs_msgs::ChassisMode::AUTO: break;
        case rmcs_msgs::ChassisMode::SPIN:
            if (apply_toggle_forward)
                spinning_forward_ = !spinning_forward_;
            result.angular_velocity = std::clamp(
                spin_ratio_ * (spinning_forward_ ? kAngularVelocityMax : -kAngularVelocityMax),
                -kAngularVelocityMax, kAngularVelocityMax);
            break;
        case rmcs_msgs::ChassisMode::STEP_DOWN:
            result.angular_velocity = following_velocity_controller_.update(calc_angle_err_(
                result.chassis_control_angle, gimbal_yaw_angle_error, gimbal_yaw_angle,
                std::numbers::pi));
            break;
        case rmcs_msgs::ChassisMode::LAUNCH_RAMP: {
            double e = calc_angle_err_(
                result.chassis_control_angle, gimbal_yaw_angle_error, gimbal_yaw_angle,
                2 * std::numbers::pi);
            if (e > std::numbers::pi)
                e -= 2 * std::numbers::pi;
            result.angular_velocity = following_velocity_controller_.update(e);
            break;
        }
        default: break;
        }
        result.chassis_angle = 2 * std::numbers::pi - gimbal_yaw_angle;
        return result;
    }

    void update_acceleration_estimate(
        const Eigen::Vector2d& translational_velocity, double dt, double limit) {
        if (!translational_velocity.array().isFinite().all()) {
            control_acceleration_estimate_.setZero();
            last_translational_velocity_.setZero();
            last_valid_ = false;
            return;
        }
        if (!last_valid_) {
            last_translational_velocity_ = translational_velocity;
            control_acceleration_estimate_.setZero();
            last_valid_ = true;
            return;
        }
        const Eigen::Vector2d cap = Eigen::Vector2d::Constant(limit);
        control_acceleration_estimate_ =
            ((translational_velocity - last_translational_velocity_) / dt)
                .cwiseMax(-cap)
                .cwiseMin(cap);
        last_translational_velocity_ = translational_velocity;
    }

    void reset_acceleration_estimate() {
        control_acceleration_estimate_.setZero();
        last_translational_velocity_.setZero();
        last_valid_ = false;
    }

    Eigen::Vector2d control_acceleration_estimate() const { return control_acceleration_estimate_; }
    bool spinning_forward() const { return spinning_forward_; }

private:
    double spin_ratio_ = 1.0;
    bool spinning_forward_ = true;
    pid::PidCalculator following_velocity_controller_{10.0, 0.0, 0.0};
    Eigen::Vector2d control_acceleration_estimate_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d last_translational_velocity_ = Eigen::Vector2d::Zero();
    bool last_valid_ = false;

    double
        calc_angle_err_(double& cca, double yaw_error, double yaw_angle, double alignment) const {
        cca = yaw_error;
        if (cca < 0)
            cca += 2 * std::numbers::pi;
        double e = cca + yaw_angle;
        if (e >= 2 * std::numbers::pi)
            e -= 2 * std::numbers::pi;
        while (e > alignment / 2) {
            cca -= alignment;
            if (cca < 0)
                cca += 2 * std::numbers::pi;
            e -= alignment;
        }
        return e;
    }
};

// --- ActiveSuspension ---
struct ActiveSuspension {
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kGravity = 9.81;
    static constexpr double kMaxAttitudeRad = 30.0 * std::numbers::pi / 180.0;
    static constexpr double kMinForceArmSin = 0.1;
    static constexpr double kContactConfidenceEnterThreshold = 0.55;
    static constexpr double kContactConfidenceExitThreshold = 0.35;
    static constexpr double kContactConfidenceFilterAlpha = 0.25;
    static constexpr double kMinimumArmingTime = 0.02;
    static constexpr std::array<double, kJointCount> kPitchSigns = {-1.0, 1.0, 1.0, -1.0};
    static constexpr std::array<double, kJointCount> kRollSigns = {1.0, 1.0, -1.0, -1.0};

    void load_params(rclcpp::Node& node, double min_angle, double max_angle) {
        params_ = SuspensionParams{
            .mass = node.get_parameter_or("active_suspension_mass", 22.5),
            .rod_length = node.get_parameter_or("active_suspension_rod_length", 0.150),
            .Kz = node.get_parameter_or("active_suspension_Kz", 150.0),
            .Kp = node.get_parameter_or("active_suspension_Kp", 200.0),
            .pitch_ki = node.get_parameter_or("active_suspension_pitch_ki", 0.0),
            .Dp = node.get_parameter_or("active_suspension_Dp", 20.0),
            .Kr = node.get_parameter_or("active_suspension_Kr", 200.0),
            .roll_ki = node.get_parameter_or("active_suspension_roll_ki", 0.0),
            .Dr = node.get_parameter_or("active_suspension_Dr", 20.0),
            .D_leg = node.get_parameter_or("active_suspension_D_leg", 10.0),
            .com_height = node.get_parameter_or("active_suspension_com_height", 0.15),
            .wheel_base_half_x = node.get_parameter_or(
                "active_suspension_wheel_base_half_x", 0.2341741 / std::numbers::sqrt2),
            .wheel_base_half_y = node.get_parameter_or(
                "active_suspension_wheel_base_half_y", 0.2341741 / std::numbers::sqrt2),
            .gravity_comp_gain = node.get_parameter_or("active_suspension_gravity_comp_gain", 1.0),
            .control_acceleration_limit = std::abs(
                node.get_parameter_or("active_suspension_control_acceleration_limit", 6.0)),
            .preload_angle =
                std::abs(node.get_parameter_or("active_suspension_preload_angle_deg", 8.0))
                * std::numbers::pi / 180.0,
            .entry_offset =
                std::abs(node.get_parameter_or(
                    "active_suspension_entry_offset_deg",
                    node.get_parameter_or("active_suspension_enter_deploy_tolerance_deg", 1.5)))
                * std::numbers::pi / 180.0,
            .ride_height_offset =
                std::abs(node.get_parameter_or("active_suspension_ride_height_offset_deg", 0.0))
                * std::numbers::pi / 180.0,
            .hold_travel =
                std::abs(node.get_parameter_or(
                    "active_suspension_hold_travel_deg",
                    node.get_parameter_or("active_suspension_exit_deploy_tolerance_deg", 3.0)))
                * std::numbers::pi / 180.0,
            .activation_velocity_threshold =
                node.get_parameter_or("active_suspension_activation_velocity_threshold_deg", 15.0)
                * std::numbers::pi / 180.0,
            .target_physical_velocity_limit =
                std::max(
                    node.get_parameter_or(
                        "active_suspension_target_velocity_limit_deg",
                        node.get_parameter_or("target_physical_velocity_limit", 180.0)),
                    1e-6)
                * std::numbers::pi / 180.0,
            .target_physical_acceleration_limit =
                std::max(
                    node.get_parameter_or(
                        "active_suspension_target_acceleration_limit_deg",
                        node.get_parameter_or("target_physical_acceleration_limit", 720.0)),
                    1e-6)
                * std::numbers::pi / 180.0,
            .torque_limit = std::abs(node.get_parameter_or("active_suspension_torque_limit", 80.0)),
            .pitch_angle_diff_limit =
                std::abs(node.get_parameter_or(
                    "active_suspension_pitch_angle_diff_limit_deg", max_angle - min_angle))
                * std::numbers::pi / 180.0,
            .roll_angle_diff_limit =
                std::abs(node.get_parameter_or(
                    "active_suspension_roll_angle_diff_limit_deg", max_angle - min_angle))
                * std::numbers::pi / 180.0,
            .pid_integral_limit =
                std::abs(node.get_parameter_or(
                    "active_suspension_pid_integral_limit_deg", max_angle - min_angle))
                * std::numbers::pi / 180.0,
        };
        pitch_pid_.kp = params_.Kp;
        pitch_pid_.ki = params_.pitch_ki;
        pitch_pid_.kd = params_.Dp;
        pitch_pid_.integral_limit = params_.pid_integral_limit;
        pitch_pid_.output_limit = params_.pitch_angle_diff_limit;
        roll_pid_.kp = params_.Kr;
        roll_pid_.ki = params_.roll_ki;
        roll_pid_.kd = params_.Dr;
        roll_pid_.integral_limit = params_.pid_integral_limit;
        roll_pid_.output_limit = params_.roll_angle_diff_limit;
        enabled_ = node.get_parameter_or("active_suspension_enable", false);
        calib_wait_ = std::max(node.get_parameter_or("chassis_imu_calibration_wait_s", 2.0), 0.0);
        calib_sample_ =
            std::max(node.get_parameter_or("chassis_imu_calibration_sample_s", 3.0), 1e-6);
    }

    bool enabled() const { return enabled_; }
    void set_enabled(bool v) { enabled_ = v; }

    void update(
        const JointFeedbackFrame& fb, double imu_pitch, double imu_roll, double imu_pitch_rate,
        double imu_roll_rate, double dt, bool requested, double min_angle_deg, double max_angle_deg,
        const Eigen::Vector2d& control_accel,
        std::array<double, kJointCount>& current_target_physical_angles,
        std::array<double, kJointCount>& out_suspension_mode,
        std::array<double, kJointCount>& out_suspension_torque) {
        static constexpr auto deg_to_rad = [](double d) { return d * std::numbers::pi / 180.0; };

        clear_outputs_(out_suspension_mode, out_suspension_torque);
        prepare_commands_();

        if (!requested) {
            reset_state_();
            current_target_physical_angles = requested_target_angles_;
            publish_outputs_(out_suspension_mode, out_suspension_torque);
            return;
        }

        const double deploy = deg_to_rad(min_angle_deg);
        const double entry = deploy + params_.entry_offset;
        const double ride_height =
            std::clamp(deploy + params_.ride_height_offset, deploy, deg_to_rad(max_angle_deg));
        const double support_zero_angle = deploy - params_.preload_angle;
        const double release = ride_height + params_.hold_travel;

        AttitudeBias bias =
            compute_attitude_bias_(imu_pitch, imu_roll, imu_pitch_rate, imu_roll_rate, dt);
        if (!std::isfinite(bias.pitch_force) || !std::isfinite(bias.roll_force)) {
            reset_state_();
            current_target_physical_angles.fill(deploy);
            publish_outputs_(out_suspension_mode, out_suspension_torque);
            return;
        }

        update_leg_contact_estimates_(fb);
        update_leg_states_(fb, entry, release, dt);
        compute_leg_support_intents_(fb, bias, support_zero_angle, ride_height, control_accel);
        current_target_physical_angles = target_angles_;
        publish_outputs_(out_suspension_mode, out_suspension_torque);
    }

    void update_imu_calibration(
        bool symmetric_targets, double imu_pitch, double imu_roll, double dt) {
        if (!symmetric_targets) {
            calib_hold_ = 0.0;
            calib_count_ = 0;
            calib_pitch_sum_ = 0.0;
            calib_roll_sum_ = 0.0;
            calib_done_ = false;
            return;
        }
        if (!std::isfinite(imu_pitch) || !std::isfinite(imu_roll))
            return;
        calib_hold_ += dt;
        if (calib_hold_ < calib_wait_)
            return;
        double end = calib_wait_ + calib_sample_;
        if (calib_hold_ < end) {
            calib_pitch_sum_ += imu_pitch;
            calib_roll_sum_ += imu_roll;
            ++calib_count_;
            return;
        }
        if (calib_done_)
            return;
        calib_done_ = true;
        if (calib_count_ == 0)
            return;
        imu_pitch_offset_ = calib_pitch_sum_ / static_cast<double>(calib_count_);
        imu_roll_offset_ = calib_roll_sum_ / static_cast<double>(calib_count_);
    }

    void reset_calibration() {
        calib_hold_ = 0.0;
        calib_count_ = 0;
        calib_pitch_sum_ = 0.0;
        calib_roll_sum_ = 0.0;
        calib_done_ = false;
    }

    void reset_all() {
        reset_state_();
        reset_calibration();
    }

    double target_vel_limit() const { return params_.target_physical_velocity_limit; }
    double target_accel_limit() const { return params_.target_physical_acceleration_limit; }
    double control_accel_limit() const { return params_.control_acceleration_limit; }

private:
    SuspensionParams params_{};
    bool enabled_ = false;
    AttitudePidAxis pitch_pid_, roll_pid_;
    double imu_pitch_offset_ = 0.0, imu_roll_offset_ = 0.0;
    double calib_wait_ = 0.0, calib_sample_ = 0.0;
    double calib_hold_ = 0.0;
    size_t calib_count_ = 0;
    double calib_pitch_sum_ = 0.0, calib_roll_sum_ = 0.0;
    bool calib_done_ = false;
    std::array<bool, kJointCount> suspension_active_{};
    std::array<LegControlState, kJointCount> leg_states_{};
    std::array<LegCommand, kJointCount> leg_commands_{};
    std::array<double, kJointCount> requested_target_angles_{};
    std::array<double, kJointCount> target_angles_{};

    static double deg_to_rad_(double d) { return d * std::numbers::pi / 180.0; }

    void clear_outputs_(
        std::array<double, kJointCount>& modes, std::array<double, kJointCount>& torques) {
        modes.fill(false);
        torques.fill(kNaN);
    }
    void publish_outputs_(
        std::array<double, kJointCount>& modes, std::array<double, kJointCount>& torques) {
        for (size_t i = 0; i < kJointCount; ++i) {
            modes[i] = leg_commands_[i].suspension_mode;
            torques[i] = leg_commands_[i].suspension_torque;
        }
    }

    void reset_state_() {
        suspension_active_.fill(false);
        for (size_t i = 0; i < kJointCount; ++i) {
            leg_states_[i] = LegControlState{};
            leg_commands_[i] = LegCommand{};
        }
    }

    void prepare_commands_() {
        for (size_t i = 0; i < kJointCount; ++i) {
            leg_commands_[i] = LegCommand{
                .requested_target_angle = requested_target_angles_[i],
                .final_target_angle = target_angles_[i],
            };
            leg_states_[i].output_active = false;
            leg_states_[i].support_force = 0.0;
        }
    }

    static LegFeedback leg_feedback_at_(const JointFeedbackFrame& fb, size_t i) {
        return {
            .motor_angle = fb.motor_angles[i],
            .physical_angle = fb.physical_angles[i],
            .physical_velocity = fb.physical_velocities[i],
            .joint_torque = fb.joint_torques[i],
            .eso_z2 = fb.eso_z2[i],
            .eso_z3 = fb.eso_z3[i]};
    }

    double estimate_contact_(const LegFeedback& lf) const {
        double c = 1.0;
        if (std::isfinite(lf.eso_z3))
            c -= std::clamp(std::abs(lf.eso_z3) / 80.0, 0.0, 0.5);
        if (std::isfinite(lf.joint_torque))
            c += std::clamp(std::abs(lf.joint_torque) / 20.0, 0.0, 0.3);
        if (std::isfinite(lf.physical_velocity))
            c -= std::clamp(std::abs(lf.physical_velocity) / 10.0, 0.0, 0.2);
        return std::clamp(c, 0.0, 1.0);
    }

    bool contact_ready_(const LegControlState& s) const {
        return s.contact_latched
            || s.filtered_contact_confidence >= kContactConfidenceEnterThreshold;
    }

    void update_leg_contact_estimates_(const JointFeedbackFrame& fb) {
        for (size_t i = 0; i < kJointCount; ++i) {
            auto& s = leg_states_[i];
            s.contact_confidence = estimate_contact_(leg_feedback_at_(fb, i));
            s.filtered_contact_confidence = std::clamp(
                (1.0 - kContactConfidenceFilterAlpha) * s.filtered_contact_confidence
                    + kContactConfidenceFilterAlpha * s.contact_confidence,
                0.0, 1.0);
            if (s.filtered_contact_confidence >= kContactConfidenceEnterThreshold)
                s.contact_latched = true;
            else if (s.filtered_contact_confidence <= kContactConfidenceExitThreshold)
                s.contact_latched = false;
        }
    }

    void update_leg_states_(const JointFeedbackFrame& fb, double entry, double release, double dt) {
        for (size_t i = 0; i < kJointCount; ++i) {
            auto lf = leg_feedback_at_(fb, i);
            bool rd =
                std::isfinite(requested_target_angles_[i]) && requested_target_angles_[i] <= entry;
            update_suspension_state_(i, lf, rd, entry, release, dt);
        }
    }

    void compute_leg_support_intents_(
        const JointFeedbackFrame& fb, const AttitudeBias& bias, double support_zero_angle,
        double ride_height, const Eigen::Vector2d& control_accel) {
        for (size_t i = 0; i < kJointCount; ++i) {
            auto lf = leg_feedback_at_(fb, i);
            auto& s = leg_states_[i];
            if (s.phase != SuspensionPhase::kActive)
                continue;
            s.support_force =
                compute_leg_support_force_(i, lf, bias, support_zero_angle, control_accel);
            leg_commands_[i].final_target_angle = ride_height;
            leg_commands_[i].suspension_mode = true;
            leg_commands_[i].suspension_torque =
                leg_force_to_torque_(s.support_force, lf.physical_angle);
        }
    }

    AttitudeBias compute_attitude_bias_(
        double pitch, double roll, double pitch_rate, double roll_rate, double dt) {
        double corrected_pitch =
            std::clamp(pitch - imu_pitch_offset_, -kMaxAttitudeRad, kMaxAttitudeRad);
        double corrected_roll =
            std::clamp(roll - imu_roll_offset_, -kMaxAttitudeRad, kMaxAttitudeRad);
        double pitch_force = pitch_pid_.update(-corrected_pitch, pitch_rate, dt);
        double roll_force = roll_pid_.update(corrected_roll, -roll_rate, dt);
        if (!std::isfinite(pitch_force) || !std::isfinite(roll_force))
            return {kNaN, kNaN};
        return {pitch_force, roll_force};
    }

    double compute_leg_support_force_(
        size_t i, const LegFeedback& lf, const AttitudeBias& bias, double support_zero_angle,
        const Eigen::Vector2d& control_accel) const {
        double f = params_.gravity_comp_gain * params_.mass * kGravity / 4.0
                 + params_.Kz * (lf.physical_angle - support_zero_angle)
                 + params_.D_leg * lf.physical_velocity;
        f += kPitchSigns[i] * bias.pitch_force + kRollSigns[i] * bias.roll_force;
        if (params_.com_height > 0.0 && params_.wheel_base_half_x > 1e-6
            && params_.wheel_base_half_y > 1e-6) {
            f += kPitchSigns[i] * params_.mass * control_accel.x() * params_.com_height
               / (4.0 * params_.wheel_base_half_x);
            f += kRollSigns[i] * params_.mass * control_accel.y() * params_.com_height
               / (4.0 * params_.wheel_base_half_y);
        }
        return std::max(f, 0.0);
    }

    double leg_force_to_torque_(double force, double angle) const {
        return std::clamp(
            force * params_.rod_length * std::max(std::sin(angle), kMinForceArmSin),
            -params_.torque_limit, params_.torque_limit);
    }

    void update_suspension_state_(
        size_t i, const LegFeedback& lf, bool rd, double entry, double release, double dt) {
        auto& s = leg_states_[i];
        s.phase_elapsed += (s.requested_deploy != rd) ? 0.0 : dt;
        s.requested_deploy = rd;
        if (!rd || !std::isfinite(lf.physical_angle) || !std::isfinite(lf.physical_velocity)) {
            s.phase = SuspensionPhase::kInactive;
            suspension_active_[i] = false;
            s.output_active = false;
            s.phase_elapsed = 0.0;
            s.contact_latched = false;
            return;
        }
        bool eok = lf.physical_angle <= entry;
        bool vok = std::abs(lf.physical_velocity) <= params_.activation_velocity_threshold;
        bool cok = contact_ready_(s);
        switch (s.phase) {
        case SuspensionPhase::kInactive:
            suspension_active_[i] = false;
            s.output_active = false;
            if (eok) {
                s.phase = SuspensionPhase::kArming;
                s.phase_elapsed = 0.0;
            }
            break;
        case SuspensionPhase::kArming:
            suspension_active_[i] = false;
            s.output_active = false;
            if (!eok) {
                s.phase = SuspensionPhase::kInactive;
                s.phase_elapsed = 0.0;
                break;
            }
            if (vok && std::isfinite(lf.motor_angle)
                && (cok || s.phase_elapsed >= kMinimumArmingTime)) {
                suspension_active_[i] = true;
                s.phase = SuspensionPhase::kActive;
                s.output_active = true;
                s.phase_elapsed = 0.0;
            }
            break;
        case SuspensionPhase::kActive:
            if (lf.physical_angle > release || (!cok && s.phase_elapsed >= kMinimumArmingTime)) {
                suspension_active_[i] = false;
                s.phase = SuspensionPhase::kReleasing;
                s.output_active = false;
                s.phase_elapsed = 0.0;
                break;
            }
            suspension_active_[i] = true;
            s.output_active = true;
            break;
        case SuspensionPhase::kReleasing:
            suspension_active_[i] = false;
            s.output_active = false;
            if (!eok || s.phase_elapsed >= kMinimumArmingTime) {
                s.phase = SuspensionPhase::kInactive;
                s.phase_elapsed = 0.0;
            }
            break;
        }
    }
};

// ============================================================
// DeformableChassis — thin orchestrator
// ============================================================
class DeformableChassis
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DeformableChassis()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        const double spin_ratio = std::clamp(get_parameter_or("spin_ratio", 0.6), 0.0, 1.0);
        const double min_angle = get_parameter_or("min_angle", 15.0);
        const double max_angle = get_parameter_or("max_angle", 55.0);
        const double vel_limit = std::max(
            std::abs(get_parameter_or("target_physical_velocity_limit", 180.0)) * std::numbers::pi
                / 180.0,
            1e-6);
        const double accel_limit = std::max(
            std::abs(get_parameter_or("target_physical_acceleration_limit", 720.0))
                * std::numbers::pi / 180.0,
            1e-6);

        velocity_control_.configure(spin_ratio);
        suspension_.load_params(*this, min_angle, max_angle);
        trajectory_.init(min_angle, max_angle, vel_limit, accel_limit);

        for (size_t i = 0; i < kJointCount; ++i)
            joint_offsets_[i] =
                get_parameter_or(std::string(kJointNames[i]) + "_joint_offset", 0.0);

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/rotary_knob", rotary_knob_);
        register_input("/predefined/update_rate", update_rate_);
        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_, false);
        register_input("/gimbal/yaw/control_angle_error", gimbal_yaw_angle_error_, false);

        auto reg_joint_input = [this](size_t i) {
            const auto& name = kJointNames[i];
            auto& j = joints_[i];
            register_input(joint_path_(name, "angle"), j.angle, false);
            register_input(joint_path_(name, "physical_angle"), j.physical_angle, false);
            register_input(joint_path_(name, "physical_velocity"), j.physical_velocity, false);
            register_input(joint_path_(name, "torque"), j.torque, false);
            register_input(joint_path_(name, "encoder_angle"), j.encoder_angle, false);
            register_input(joint_path_(name, "eso_z2"), j.eso_z2, false);
            register_input(joint_path_(name, "eso_z3"), j.eso_z3, false);
        };
        for (size_t i = 0; i < kJointCount; ++i)
            reg_joint_input(i);

        register_input("/chassis/imu/pitch", chassis_imu_pitch_, false);
        register_input("/chassis/imu/roll", chassis_imu_roll_, false);
        register_input("/chassis/imu/pitch_rate", chassis_imu_pitch_rate_, false);
        register_input("/chassis/imu/roll_rate", chassis_imu_roll_rate_, false);

        register_output("/gimbal/scope/control_torque", scope_motor_control_torque_, kNaN);
        register_output("/chassis/angle", chassis_angle_, kNaN);
        register_output("/chassis/control_angle", chassis_control_angle_, kNaN);
        register_output("/chassis/control_mode", mode_);
        register_output("/chassis/control_velocity", chassis_control_velocity_);

        auto reg_joint_output = [this](size_t i) {
            const auto& name = kJointNames[i];
            auto& j = joints_[i];
            register_output(joint_path_(name, "control_angle_error"), angle_errors_[i], kNaN);
            register_output(joint_path_(name, "target_angle"), j.target_angle, kNaN);
            register_output(
                joint_path_(name, "target_physical_angle"), j.target_physical_angle, kNaN);
            register_output(
                joint_path_(name, "target_physical_velocity"), j.target_physical_velocity, kNaN);
            register_output(
                joint_path_(name, "target_physical_acceleration"), j.target_physical_acceleration,
                kNaN);
            register_output(joint_path_(name, "suspension_torque"), j.suspension_torque, kNaN);
        };
        for (size_t i = 0; i < kJointCount; ++i) {
            reg_joint_output(i);
            register_output(
                joint_path_(kJointNames[i], "suspension_mode"), suspension_modes_[i], false);
        }
        register_output("/chassis/processed_encoder/angle", processed_encoder_angle_, kNaN);

        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        chassis_control_velocity_->vector << kNaN, kNaN, kNaN;

        int offset_count = 0;
        for (size_t i = 0; i < kJointCount; ++i)
            if (has_parameter(std::string(kJointNames[i]) + "_joint_offset"))
                ++offset_count;
        if (offset_count > 0 && offset_count != static_cast<int>(kJointCount))
            throw std::runtime_error(
                "joint offsets must be configured for all four joints or removed entirely");
        joint_feedback_source_ = (offset_count == static_cast<int>(kJointCount))
                                   ? JointFeedbackSource::kLegacyEncoderAngle
                                   : JointFeedbackSource::kMotorAngle;
    }

    void before_updating() override {
        auto ensure = [this](auto& field, double value, const char* name) {
            if (!field.ready()) {
                field.make_and_bind_directly(value);
                RCLCPP_WARN(get_logger(), "Failed to fetch \"%s\". Set to %.1f.", name, value);
            }
        };
        ensure(gimbal_yaw_angle_, 0.0, "/gimbal/yaw/angle");
        ensure(gimbal_yaw_angle_error_, 0.0, "/gimbal/yaw/control_angle_error");
        for (auto& j : joints_)
            ensure(j.torque, 0.0, "joint torque");
        ensure(chassis_imu_pitch_, 0.0, "chassis imu pitch");
        ensure(chassis_imu_roll_, 0.0, "chassis imu roll");
        ensure(chassis_imu_pitch_rate_, 0.0, "chassis imu pitch_rate");
        ensure(chassis_imu_roll_rate_, 0.0, "chassis imu roll_rate");
        validate_joint_feedback_inputs_();
    }

    void update() override {
        using rmcs_msgs::Switch;
        const auto switch_right = *switch_right_;
        const auto switch_left = *switch_left_;
        const auto keyboard = *keyboard_;
        do {
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                reset_all_controls_();
                break;
            }
            update_mode_from_inputs_(switch_left, switch_right, keyboard);
            update_velocity_control_(keyboard);
            update_lift_target_toggle_(keyboard);
            run_joint_intent_pipeline_();
        } while (false);
        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;
        last_keyboard_ = keyboard;
    }

private:
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kRadToDeg = 180.0 / std::numbers::pi;

    static std::string joint_path_(const char* name, const char* suffix) {
        char b[128];
        std::snprintf(b, sizeof(b), "/chassis/%s_joint/%s", name, suffix);
        return {b};
    }

    void validate_joint_feedback_inputs_() const {
        bool ok = true;
        for (const auto& j : joints_) {
            if (joint_feedback_source_ == JointFeedbackSource::kMotorAngle) {
                if (!j.angle.ready())
                    ok = false;
            } else {
                if (!j.encoder_angle.ready())
                    ok = false;
            }
        }
        if (ok)
            return;
        throw std::runtime_error(
            joint_feedback_source_ == JointFeedbackSource::kMotorAngle
                ? "missing V2 joint feedback inputs: /chassis/*_joint/angle"
                : "missing legacy joint feedback inputs: /chassis/*_joint/encoder_angle");
    }

    // --- helpers ---
    bool suspension_requested_() const {
        return suspension_.enabled()
            && (keyboard_->ctrl
                || (switch_left_.ready() && switch_right_.ready()
                    && *switch_left_ == rmcs_msgs::Switch::DOWN
                    && *switch_right_ == rmcs_msgs::Switch::UP));
    }

    // --- mode ---
    void update_mode_from_inputs_(
        rmcs_msgs::Switch sl, rmcs_msgs::Switch sr, const rmcs_msgs::Keyboard& kb) {
        auto m = *mode_;
        if (sl == rmcs_msgs::Switch::DOWN)
            return;
        if (last_switch_right_ == rmcs_msgs::Switch::MIDDLE && sr == rmcs_msgs::Switch::DOWN) {
            m = (m == rmcs_msgs::ChassisMode::SPIN) ? rmcs_msgs::ChassisMode::STEP_DOWN
                                                    : rmcs_msgs::ChassisMode::SPIN;
        } else if (!last_keyboard_.c && kb.c) {
            m = (m == rmcs_msgs::ChassisMode::SPIN) ? rmcs_msgs::ChassisMode::AUTO
                                                    : rmcs_msgs::ChassisMode::SPIN;
        } else if (!last_keyboard_.x && kb.x) {
            m = (m == rmcs_msgs::ChassisMode::LAUNCH_RAMP) ? rmcs_msgs::ChassisMode::AUTO
                                                           : rmcs_msgs::ChassisMode::LAUNCH_RAMP;
        } else if (!last_keyboard_.z && kb.z) {
            m = (m == rmcs_msgs::ChassisMode::STEP_DOWN) ? rmcs_msgs::ChassisMode::AUTO
                                                         : rmcs_msgs::ChassisMode::STEP_DOWN;
        }
        *mode_ = m;
    }

    // --- velocity ---
    void update_velocity_control_(const rmcs_msgs::Keyboard& kb) {
        auto tv = velocity_control_.compute_translational(*joystick_right_, kb, *gimbal_yaw_angle_);
        bool toggle = (last_keyboard_.c != kb.c && *mode_ != rmcs_msgs::ChassisMode::SPIN);
        auto ar = velocity_control_.compute_angular(
            *mode_, *gimbal_yaw_angle_, *gimbal_yaw_angle_error_, toggle);
        double dt = update_dt_();
        velocity_control_.update_acceleration_estimate(tv, dt, suspension_.control_accel_limit());
        *chassis_angle_ = ar.chassis_angle;
        *chassis_control_angle_ = ar.chassis_control_angle;
        chassis_control_velocity_->vector << tv, ar.angular_velocity;
    }

    double update_dt_() const {
        if (update_rate_.ready() && std::isfinite(*update_rate_) && *update_rate_ > 1e-6)
            return 1.0 / *update_rate_;
        return 1e-3;
    }

    // --- lift toggle ---
    void update_lift_target_toggle_(rmcs_msgs::Keyboard keyboard) {
        constexpr double kRotaryKnobEdgeThreshold = 0.7;

        const bool keyboard_toggle = !last_keyboard_.q && keyboard.q;
        const bool rotary_knob_toggle =
            last_rotary_knob_ < kRotaryKnobEdgeThreshold
            && *rotary_knob_ >= kRotaryKnobEdgeThreshold;

        if (apply_symmetric_target_)
            trajectory_.fill_symmetric_targets();

        if (rotary_knob_toggle || keyboard_toggle) {
            trajectory_.set_target_angle(
                std::abs(trajectory_.target_angle() - trajectory_.max_angle()) < 1e-6
                    ? trajectory_.min_angle()
                    : trajectory_.max_angle());
            apply_symmetric_target_ = true;
        }

        last_rotary_knob_ = *rotary_knob_;
    }

    // --- reset ---
    void reset_all_controls_() {
        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        velocity_control_.reset_acceleration_estimate();
        suspension_.reset_all();
        chassis_control_velocity_->vector << kNaN, kNaN, kNaN;
        *chassis_angle_ = kNaN;
        *chassis_control_angle_ = kNaN;
        trajectory_.reset(trajectory_.max_angle());
        *scope_motor_control_torque_ = kNaN;
        for (auto& e : angle_errors_)
            *e = kNaN;
        for (auto& j : joints_) {
            *j.target_angle = kNaN;
            *j.target_physical_angle = kNaN;
            *j.target_physical_velocity = kNaN;
            *j.target_physical_acceleration = kNaN;
            *j.suspension_torque = kNaN;
        }
        for (auto& m : suspension_modes_)
            *m = false;
        *processed_encoder_angle_ = kNaN;
    }

    // --- feedback ---
    JointFeedbackFrame read_joint_feedback_() const {
        JointFeedbackFrame f;
        f.motor_angles.fill(kNaN);
        f.physical_angles.fill(kNaN);
        f.physical_velocities.fill(kNaN);
        f.joint_torques.fill(kNaN);
        f.eso_z2.fill(kNaN);
        f.eso_z3.fill(kNaN);
        for (size_t i = 0; i < kJointCount; ++i) {
            const auto& j = joints_[i];
            if (j.angle.ready() && std::isfinite(*j.angle)) {
                f.motor_angles[i] = *j.angle;
                f.physical_angles[i] = 1.090830782496456 - f.motor_angles[i];
            }
            if (j.physical_angle.ready() && std::isfinite(*j.physical_angle))
                f.physical_angles[i] = *j.physical_angle;
            if (j.physical_velocity.ready() && std::isfinite(*j.physical_velocity))
                f.physical_velocities[i] = *j.physical_velocity;
            if (j.torque.ready() && std::isfinite(*j.torque))
                f.joint_torques[i] = *j.torque;
            if (j.eso_z2.ready() && std::isfinite(*j.eso_z2))
                f.eso_z2[i] = *j.eso_z2;
            if (j.eso_z3.ready() && std::isfinite(*j.eso_z3))
                f.eso_z3[i] = *j.eso_z3;
        }
        return f;
    }

    // --- main pipeline ---
    void run_joint_intent_pipeline_() {
        auto feedback = read_joint_feedback_();

        suspension_.update_imu_calibration(
            trajectory_.symmetric_requested(), *chassis_imu_pitch_, *chassis_imu_roll_,
            update_dt_());

        if (!trajectory_.active()
            && !trajectory_.initialize_from_feedback(
                feedback.motor_angles, feedback.physical_angles)) {
            reset_all_controls_();
            return;
        }

        if (apply_symmetric_target_)
            trajectory_.fill_symmetric_targets();
        trajectory_.refresh_deploy_targets(
            suspension_requested_(), keyboard_->ctrl, trajectory_.min_angle());

        scope_motor_control_(keyboard_->ctrl);

        auto target_physical = trajectory_.current_physical();
        std::array<double, kJointCount> sus_modes, sus_torques;
        sus_modes.fill(false);
        sus_torques.fill(kNaN);

        suspension_.update(
            feedback, *chassis_imu_pitch_, *chassis_imu_roll_, *chassis_imu_pitch_rate_,
            *chassis_imu_roll_rate_, update_dt_(), suspension_requested_(), trajectory_.min_angle(),
            trajectory_.max_angle(), velocity_control_.control_acceleration_estimate(),
            target_physical, sus_modes, sus_torques);

        trajectory_.update_trajectory(
            update_dt_(), suspension_requested_(), suspension_.target_vel_limit(),
            suspension_.target_accel_limit());

        for (size_t i = 0; i < kJointCount; ++i) {
            *joints_[i].target_angle = trajectory_.target_angles()[i];
            *joints_[i].target_physical_angle = trajectory_.target_physical_angles()[i];
            *joints_[i].target_physical_velocity = trajectory_.target_velocities()[i];
            *joints_[i].target_physical_acceleration = trajectory_.target_accelerations()[i];
            *joints_[i].suspension_torque = sus_torques[i];
            *suspension_modes_[i] = static_cast<bool>(sus_modes[i]);
        }

        if (apply_symmetric_target_ && trajectory_.symmetric_requested())
            trajectory_.fill_symmetric_targets();

        double sum = 0.0;
        int cnt = 0;
        for (const auto& j : joints_) {
            if (j.physical_angle.ready() && std::isfinite(*j.physical_angle)) {
                sum += *j.physical_angle;
                ++cnt;
            }
        }
        *processed_encoder_angle_ = (cnt > 0) ? kRadToDeg * sum / cnt : kNaN;
    }

    void scope_motor_control_(bool prone_override) {
        if (prone_override && *mode_ != rmcs_msgs::ChassisMode::SPIN)
            *scope_motor_control_torque_ = -0.3;
        else
            *scope_motor_control_torque_ = 0.3;
    }

    // --- member variables ---
    InputInterface<Eigen::Vector2d> joystick_right_, joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_, switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<double> rotary_knob_, update_rate_;
    double last_rotary_knob_ = 0.0;
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    InputInterface<double> gimbal_yaw_angle_, gimbal_yaw_angle_error_;
    OutputInterface<double> chassis_angle_, chassis_control_angle_;
    OutputInterface<rmcs_msgs::ChassisMode> mode_;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;

    ChassisVelocityControl velocity_control_;
    ActiveSuspension suspension_;
    JointTrajectoryPlanner trajectory_;
    bool apply_symmetric_target_ = true;

    std::array<JointIO, kJointCount> joints_{};
    std::array<JointIO::Out, kJointCount> angle_errors_;
    std::array<OutputInterface<bool>, kJointCount> suspension_modes_;
    InputInterface<double> chassis_imu_pitch_, chassis_imu_roll_, chassis_imu_pitch_rate_,
        chassis_imu_roll_rate_;
    OutputInterface<double> scope_motor_control_torque_, processed_encoder_angle_;

    std::array<double, kJointCount> joint_offsets_{};
    JointFeedbackSource joint_feedback_source_ = JointFeedbackSource::kLegacyEncoderAngle;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::DeformableChassis, rmcs_executor::Component)
