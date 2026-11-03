#pragma once

#include <algorithm>
#include <cmath>

namespace rmcs_core::hardware {

struct Joint2LiftPidParameters {
    double kp_nm_per_rad{20.0};
    double ki_nm_per_rad_s{0.5};
    double kd_nm_s_per_rad{3.0};
    double torque_limit_nm{5.0};
    double torque_slew_nm_s{8.0};
    // Zero preserves the original symmetric slew. A positive value lets the
    // controller release breakaway torque faster without increasing drive slew.
    double torque_release_nm_s{0.0};
    double reference_speed_rad_s{0.12};
    double integral_limit_rad_s{3.0};
    double lift_angle_rad{0.5235987755982988};
    double lower_limit_rad{-1.084};
    double upper_limit_rad{1.528};
    double lift_direction{1.0};
};

// A bounded joint-space position PID with a slow position reference ramp.
// Safety permission is deliberately supplied by the caller; removing it
// immediately clears the integral and torque output.
class Joint2LiftPid {
public:
    explicit Joint2LiftPid(Joint2LiftPidParameters parameters = {})
        : parameters_(parameters) {}

    bool valid_parameters() const noexcept {
        return std::isfinite(parameters_.kp_nm_per_rad) && parameters_.kp_nm_per_rad >= 0.0
            && std::isfinite(parameters_.ki_nm_per_rad_s) && parameters_.ki_nm_per_rad_s >= 0.0
            && std::isfinite(parameters_.kd_nm_s_per_rad) && parameters_.kd_nm_s_per_rad >= 0.0
            && std::isfinite(parameters_.torque_limit_nm) && parameters_.torque_limit_nm > 0.0
            && std::isfinite(parameters_.torque_slew_nm_s) && parameters_.torque_slew_nm_s > 0.0
            && std::isfinite(parameters_.torque_release_nm_s)
            && (parameters_.torque_release_nm_s == 0.0
                || parameters_.torque_release_nm_s >= parameters_.torque_slew_nm_s)
            && std::isfinite(parameters_.reference_speed_rad_s)
            && parameters_.reference_speed_rad_s > 0.0
            && std::isfinite(parameters_.integral_limit_rad_s)
            && parameters_.integral_limit_rad_s >= 0.0
            && std::isfinite(parameters_.lift_angle_rad) && parameters_.lift_angle_rad > 0.0
            && std::isfinite(parameters_.lower_limit_rad)
            && std::isfinite(parameters_.upper_limit_rad)
            && parameters_.lower_limit_rad < parameters_.upper_limit_rad
            && (parameters_.lift_direction == 1.0 || parameters_.lift_direction == -1.0);
    }

    bool capture_target(double current_angle) noexcept {
        if (!std::isfinite(current_angle) || !valid_parameters())
            return false;
        target_angle_ = std::clamp(
            current_angle + parameters_.lift_direction * parameters_.lift_angle_rad,
            parameters_.lower_limit_rad, parameters_.upper_limit_rad);
        target_valid_ = true;
        resume(current_angle);
        return true;
    }

    void resume(double current_angle) noexcept {
        if (!std::isfinite(current_angle)) {
            running_ = false;
            return;
        }
        reference_angle_ = current_angle;
        integral_ = 0.0;
        output_torque_ = 0.0;
        running_ = target_valid_;
    }

    void stop() noexcept {
        integral_ = 0.0;
        output_torque_ = 0.0;
        running_ = false;
    }

    void invalidate_target() noexcept {
        stop();
        target_valid_ = false;
        target_angle_ = 0.0;
        reference_angle_ = 0.0;
    }

    double update(double angle, double velocity, double dt, bool permitted) noexcept {
        if (!permitted || !target_valid_ || !std::isfinite(angle) || !std::isfinite(velocity)
            || !std::isfinite(dt) || dt <= 0.0 || dt > 0.02 || !valid_parameters()) {
            stop();
            return 0.0;
        }
        if (!running_)
            resume(angle);

        const double remaining = target_angle_ - reference_angle_;
        const double max_step = parameters_.reference_speed_rad_s * dt;
        reference_angle_ += std::clamp(remaining, -max_step, max_step);
        const double error = reference_angle_ - angle;
        const double candidate_integral = std::clamp(
            integral_ + error * dt, -parameters_.integral_limit_rad_s,
            parameters_.integral_limit_rad_s);
        const double candidate_output = parameters_.kp_nm_per_rad * error
                                      + parameters_.ki_nm_per_rad_s * candidate_integral
                                      - parameters_.kd_nm_s_per_rad * velocity;
        const double saturated_output = std::clamp(
            candidate_output, -parameters_.torque_limit_nm, parameters_.torque_limit_nm);
        // Do not wind up farther into saturation; allow integration when it
        // helps unwind an already saturated output.
        if (candidate_output == saturated_output || candidate_output * error <= 0.0)
            integral_ = candidate_integral;

        const double desired_output = std::clamp(
            parameters_.kp_nm_per_rad * error + parameters_.ki_nm_per_rad_s * integral_
                - parameters_.kd_nm_s_per_rad * velocity,
            -parameters_.torque_limit_nm, parameters_.torque_limit_nm);
        if (parameters_.torque_release_nm_s > 0.0
            && output_torque_ * desired_output < 0.0) {
            // A faster release may only bring the old direction to zero. The
            // opposite direction starts on a later update at the drive slew.
            const double release_step = parameters_.torque_release_nm_s * dt;
            output_torque_ += std::clamp(-output_torque_, -release_step, release_step);
        } else {
            const bool reducing = std::abs(desired_output) < std::abs(output_torque_);
            const double rate = reducing && parameters_.torque_release_nm_s > 0.0
                ? parameters_.torque_release_nm_s : parameters_.torque_slew_nm_s;
            const double max_torque_step = rate * dt;
            output_torque_ += std::clamp(
                desired_output - output_torque_, -max_torque_step, max_torque_step);
        }
        return output_torque_;
    }

    double target_angle() const noexcept { return target_angle_; }
    double reference_angle() const noexcept { return reference_angle_; }
    double position_error(double angle) const noexcept { return target_angle_ - angle; }
    bool target_valid() const noexcept { return target_valid_; }

private:
    Joint2LiftPidParameters parameters_;
    double target_angle_{0.0};
    double reference_angle_{0.0};
    double integral_{0.0};
    double output_torque_{0.0};
    bool target_valid_{false};
    bool running_{false};
};

// Double-down is the explicit re-arm action. Once armed, centering the right
// stick after releasing double-down is required before torque is permitted.
class Joint2LiftSafetyGate {
public:
    bool update(
        bool configured_enabled, bool remote_valid, bool switches_valid, bool both_down,
        bool feedback_valid, bool update_valid, bool stick_centered) noexcept {
        if (!remote_valid || !switches_valid || !feedback_valid || !update_valid) {
            double_down_confirmed_ = false;
            stick_centered_after_safety_ = false;
        } else if (both_down) {
            double_down_confirmed_ = true;
            stick_centered_after_safety_ = false;
        } else if (double_down_confirmed_ && stick_centered) {
            stick_centered_after_safety_ = true;
        }
        permitted_ = configured_enabled && remote_valid && switches_valid && feedback_valid
                   && update_valid && !both_down && double_down_confirmed_
                   && stick_centered_after_safety_;
        return permitted_;
    }

    bool permitted() const noexcept { return permitted_; }
    bool double_down_confirmed() const noexcept { return double_down_confirmed_; }

private:
    bool double_down_confirmed_{false};
    bool stick_centered_after_safety_{false};
    bool permitted_{false};
};

} // namespace rmcs_core::hardware
