#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <numbers>

#include <rclcpp/node.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::chassis {

class DeformableChassisActiveSuspension {
public:
    static constexpr size_t kJointCount    = 4;
    static constexpr double offset_limit_rad_ = 1.0 * std::numbers::pi / 180.0;

    struct Corrections {
        std::array<double, kJointCount> angle        = {0.0, 0.0, 0.0, 0.0};
        std::array<double, kJointCount> velocity     = {0.0, 0.0, 0.0, 0.0};
        std::array<double, kJointCount> acceleration = {0.0, 0.0, 0.0, 0.0};
        std::array<bool, kJointCount> active         = {false, false, false, false};
    };

    void configure(rclcpp::Node& node) {
        load_pid_(
            node, "active_suspension_pitch_outer_", pitch_outer_pid_, //
            8.0, 0.35, 0.28, -2.0, 2.0, -3.0, 3.0);
        load_pid_(
            node, "active_suspension_pitch_inner_", pitch_inner_pid_, //
            2.0, 0.0, 0.0, -1.0, 1.0, -0.785, 0.785);
        load_pid_(
            node, "active_suspension_roll_outer_", roll_outer_pid_, //
            8.0, 0.35, 0.28, -2.0, 2.0, -3.0, 3.0);
        load_pid_(
            node, "active_suspension_roll_inner_", roll_inner_pid_, //
            2.0, 0.0, 0.0, -1.0, 1.0, -0.785, 0.785);

        correction_vel_limit_ = std::max(
            deg_to_rad_(std::abs(node.get_parameter_or(
                "active_suspension_correction_velocity_limit_deg", 720.0))),
            1e-6);
        correction_acc_limit_ = std::max(
            deg_to_rad_(std::abs(node.get_parameter_or(
                "active_suspension_correction_acceleration_limit_deg", 3600.0))),
            1e-6);

        calibration_wait_time_ = std::max(
            node.get_parameter_or("chassis_imu_calibration_wait_s", 2.0), 0.0);
        calibration_sample_time_ = std::max(
            node.get_parameter_or("chassis_imu_calibration_sample_s", 3.0), 1e-6);
    }

    void calibrate(
        double pitch, double roll,
        bool symmetric_target, double dt) {

        if (calibrated_once_)
            return;

        if (!symmetric_target) {
            reset_calibration_window_();
            return;
        }

        if (!std::isfinite(pitch) || !std::isfinite(roll))
            return;

        calibration_hold_elapsed_ += dt;
        if (calibration_hold_elapsed_ < calibration_wait_time_)
            return;

        double calibration_end =
            calibration_wait_time_ + calibration_sample_time_;
        if (calibration_hold_elapsed_ < calibration_end) {
            pitch_sum_ += pitch;
            roll_sum_ += roll;
            ++sample_count_;
            return;
        }

        if (calibration_completed_for_window_)
            return;

        calibration_completed_for_window_ = true;
        if (sample_count_ == 0)
            return;

        pitch_offset_ = std::clamp(
            pitch_sum_ / static_cast<double>(sample_count_),
            -offset_limit_rad_, offset_limit_rad_);
        roll_offset_ = std::clamp(
            roll_sum_ / static_cast<double>(sample_count_),
            -offset_limit_rad_, offset_limit_rad_);
        calibrated_once_ = true;
    }

    bool calibrated() const { return calibrated_once_; }
    double pitch_offset() const { return pitch_offset_; }
    double roll_offset() const { return roll_offset_; }

    double scope_torque(bool suspension_active, bool is_spin) const {
        if (suspension_active && !is_spin)
            return -0.3;
        return 0.3;
    }

    Corrections update(
        double pitch, double roll,
        double pitch_rate, double roll_rate,
        bool suspension_active, bool low_prone,
        double min_angle_deg, double max_angle_deg,
        double base_angle_deg, bool correction_inverted,
        const std::array<double, kJointCount>& base_joint_angles,
        double dt) {

        Corrections corr;

        if (!suspension_active) {
            reset_attitude_();
            run_correction_trajectory_(
                low_prone, min_angle_deg, max_angle_deg,
                base_angle_deg, base_joint_angles, dt, corr);
            return corr;
        }

        constexpr double max_attitude = 30.0 * std::numbers::pi / 180.0;
        double cp = std::clamp(pitch, -max_attitude, max_attitude);
        double cr = std::clamp(roll, -max_attitude, max_attitude);

        double pitch_outer = pitch_outer_pid_.update(-cp);
        double roll_outer  = roll_outer_pid_.update(cr);
        double pitch_diff  = pitch_inner_pid_.update(
            pitch_outer - pitch_rate);
        double roll_diff = roll_inner_pid_.update(
            roll_outer + roll_rate);

        if (!std::isfinite(pitch_diff) || !std::isfinite(roll_diff)) {
            reset_attitude_();
            return corr;
        }

        compute_correction_targets_(pitch_diff, roll_diff, correction_inverted);
        run_correction_trajectory_(
            low_prone, min_angle_deg, max_angle_deg, base_angle_deg,
            base_joint_angles, dt, corr);
        return corr;
    }

    void reset() {
        pitch_outer_pid_.reset();
        pitch_inner_pid_.reset();
        roll_outer_pid_.reset();
        roll_inner_pid_.reset();
        correction_target_rad_.fill(0.0);
        correction_state_rad_.fill(0.0);
        correction_velocity_state_rad_.fill(0.0);
        correction_acceleration_state_rad_.fill(0.0);
        reset_calibration_window_();
    }

private:
    static double deg_to_rad_(double deg) {
        return deg * std::numbers::pi / 180.0;
    }

    void load_pid_(
        rclcpp::Node& node, const std::string& prefix,
        pid::PidCalculator& pid,
        double kp_default, double ki_default, double kd_default,
        double integral_min_default, double integral_max_default,
        double output_min_default, double output_max_default) {

        pid.kp = node.get_parameter_or(prefix + "kp", kp_default);
        pid.ki = node.get_parameter_or(prefix + "ki", ki_default);
        pid.kd = node.get_parameter_or(prefix + "kd", kd_default);
        pid.integral_min =
            node.get_parameter_or(prefix + "integral_min", integral_min_default);
        pid.integral_max =
            node.get_parameter_or(prefix + "integral_max", integral_max_default);
        pid.output_min =
            node.get_parameter_or(prefix + "output_min", output_min_default);
        pid.output_max =
            node.get_parameter_or(prefix + "output_max", output_max_default);
    }

    void reset_attitude_() {
        pitch_outer_pid_.reset();
        pitch_inner_pid_.reset();
        roll_outer_pid_.reset();
        roll_inner_pid_.reset();
        correction_target_rad_.fill(0.0);
    }

    void reset_calibration_window_() {
        calibration_hold_elapsed_           = 0.0;
        sample_count_                       = 0;
        pitch_sum_                          = 0.0;
        roll_sum_                           = 0.0;
        calibration_completed_for_window_   = false;
    }

    void compute_correction_targets_(
        double pitch_diff, double roll_diff, bool inverted) {

        if (inverted) {
            double fp = std::max(pitch_diff, 0.0);
            double bp = std::max(-pitch_diff, 0.0);
            double lr = std::max(-roll_diff, 0.0);
            double rr = std::max(roll_diff, 0.0);
            correction_target_rad_[0] = -(fp + lr);
            correction_target_rad_[1] = -(bp + lr);
            correction_target_rad_[2] = -(bp + rr);
            correction_target_rad_[3] = -(fp + rr);
        } else {
            double fp = std::max(-pitch_diff, 0.0);
            double bp = std::max(pitch_diff, 0.0);
            double lr = std::max(roll_diff, 0.0);
            double rr = std::max(-roll_diff, 0.0);
            correction_target_rad_[0] = fp + lr;
            correction_target_rad_[1] = bp + lr;
            correction_target_rad_[2] = bp + rr;
            correction_target_rad_[3] = fp + rr;
        }
    }

    void run_correction_trajectory_(
        bool low_prone,
        double min_angle_deg, double max_angle_deg,
        double base_angle_deg,
        const std::array<double, kJointCount>& base_joint_angles,
        double dt,
        Corrections& corr) {

        double max_target_rad = deg_to_rad_(max_angle_deg);
        double min_susp_rad   = deg_to_rad_(min_angle_deg - 5.0);

        for (size_t i = 0; i < kJointCount; ++i) {
            double base_angle = std::isfinite(base_joint_angles[i])
                                    ? base_joint_angles[i]
                                    : (low_prone
                                           ? min_susp_rad
                                           : deg_to_rad_(base_angle_deg));

            double correction_min = min_susp_rad - base_angle;
            double correction_max = max_target_rad - base_angle;
            double target = std::clamp(
                correction_target_rad_[i], correction_min, correction_max);

            double& angle_st = correction_state_rad_[i];
            double& vel_st   = correction_velocity_state_rad_[i];
            double& acc_st   = correction_acceleration_state_rad_[i];

            double position_error    = target - angle_st;
            double stopping_distance =
                vel_st * vel_st / (2.0 * correction_acc_limit_);

            double desired_velocity = 0.0;
            if (std::abs(position_error) > 1e-6
                && std::abs(position_error) > stopping_distance) {
                desired_velocity =
                    std::copysign(correction_vel_limit_, position_error);
            }

            double velocity_error = desired_velocity - vel_st;
            acc_st = std::clamp(
                velocity_error / dt, -correction_acc_limit_,
                correction_acc_limit_);

            vel_st += acc_st * dt;
            vel_st = std::clamp(
                vel_st, -correction_vel_limit_, correction_vel_limit_);
            angle_st += vel_st * dt;

            double next_error = target - angle_st;
            if ((position_error > 0.0 && next_error < 0.0)
                || (position_error < 0.0 && next_error > 0.0)
                || (std::abs(next_error) < 1e-5 && std::abs(vel_st) < 1e-3)) {
                angle_st = target;
                vel_st   = 0.0;
                acc_st   = 0.0;
            }

            corr.active[i] = true;
        }

        corr.angle        = correction_state_rad_;
        corr.velocity     = correction_velocity_state_rad_;
        corr.acceleration = correction_acceleration_state_rad_;
    }

    pid::PidCalculator pitch_outer_pid_{};
    pid::PidCalculator pitch_inner_pid_{};
    pid::PidCalculator roll_outer_pid_{};
    pid::PidCalculator roll_inner_pid_{};

    double correction_vel_limit_   = 40.0;
    double correction_acc_limit_   = 200.0;

    double calibration_wait_time_   = 2.0;
    double calibration_sample_time_ = 3.0;
    double calibration_hold_elapsed_           = 0.0;
    size_t sample_count_                       = 0;
    double pitch_sum_                          = 0.0;
    double roll_sum_                           = 0.0;
    bool calibration_completed_for_window_     = false;
    bool calibrated_once_                      = false;
    double pitch_offset_ = 0.0;
    double roll_offset_  = 0.0;

    std::array<double, kJointCount> correction_target_rad_ = {
        0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> correction_state_rad_ = {
        0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> correction_velocity_state_rad_ = {
        0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> correction_acceleration_state_rad_ = {
        0.0, 0.0, 0.0, 0.0};
};

} // namespace rmcs_core::controller::chassis
