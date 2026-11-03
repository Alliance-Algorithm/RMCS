#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace rmcs_core::controller::gimbal {

struct CoordinatedYawParameters {
    double small_yaw_direction = 1.0;
    double heading_gain = 2.0;
    double follow_gain = 1.6;
    double follow_deadband_rad = 0.01;
    // Zero disables delayed takeover or the separate centered-stick gain.
    double follow_start_offset_rad = 0.0;
    double centered_follow_gain = 0.0;
    // Zero keeps legacy allocation. Otherwise reserve compensation authority
    // by limiting base motion to a fraction of the small joint's reverse cap.
    double counterrotation_speed_fraction = 0.0;
    double heading_error_deadband_rad = 0.003;
    double big_velocity_noise_threshold_rad_s = 0.02;
    double max_target_error_rad = 0.35;
    double max_small_offset_rad = 0.8;
    // Optional reference braking boundary relative to the captured center.
    // Zero preserves the original allocation without an angle-dependent cap.
    double small_speed_zero_offset_rad = 0.0;
    double small_braking_response_time_s = 0.0;
    double max_small_velocity_rad_s = 1.18125;
    double max_small_acceleration_rad_s2 = 2.0;
    double max_big_velocity_rad_s = 1.0;
    double max_big_acceleration_rad_s2 = 1.0;
};

struct CoordinatedYawOutput {
    bool active = false;
    bool reference_limited = false;
    double target_angle = 0.0;
    double heading_angle = 0.0;
    double heading_error = 0.0;
    double small_center_angle = 0.0;
    double small_offset = 0.0;
    double small_velocity = 0.0;
    double small_acceleration = 0.0;
    double big_velocity = 0.0;
    double big_acceleration = 0.0;
};

// A fixed-base, encoder-composed heading reference. The caller owns arming,
// feedback freshness, actuator torque control and the actual mechanical limits.
// The small-yaw sign relates its joint-positive direction to big-yaw positive.
class CoordinatedYaw {
public:
    explicit CoordinatedYaw(CoordinatedYawParameters parameters = {}) : parameters_(parameters) {
        const auto positive = [](double value) { return std::isfinite(value) && value > 0.0; };
        if ((parameters_.small_yaw_direction != 1.0 && parameters_.small_yaw_direction != -1.0)
            || !positive(parameters_.heading_gain) || !positive(parameters_.follow_gain)
            || !std::isfinite(parameters_.follow_deadband_rad)
            || parameters_.follow_deadband_rad < 0.0
            || !std::isfinite(parameters_.follow_start_offset_rad)
            || parameters_.follow_start_offset_rad < 0.0
            || !std::isfinite(parameters_.centered_follow_gain)
            || parameters_.centered_follow_gain < 0.0
            || !std::isfinite(parameters_.counterrotation_speed_fraction)
            || parameters_.counterrotation_speed_fraction < 0.0
            || parameters_.counterrotation_speed_fraction > 1.0
            || !std::isfinite(parameters_.heading_error_deadband_rad)
            || parameters_.heading_error_deadband_rad < 0.0
            || !std::isfinite(parameters_.big_velocity_noise_threshold_rad_s)
            || parameters_.big_velocity_noise_threshold_rad_s < 0.0
            || !positive(parameters_.max_target_error_rad)
            || parameters_.follow_deadband_rad >= parameters_.max_target_error_rad
            || parameters_.heading_error_deadband_rad >= parameters_.max_target_error_rad
            || parameters_.max_target_error_rad >= std::numbers::pi
            || !positive(parameters_.max_small_offset_rad)
            || !std::isfinite(parameters_.small_speed_zero_offset_rad)
            || parameters_.small_speed_zero_offset_rad < 0.0
            || !std::isfinite(parameters_.small_braking_response_time_s)
            || parameters_.small_braking_response_time_s < 0.0
            || !positive(parameters_.max_small_velocity_rad_s)
            || !positive(parameters_.max_small_acceleration_rad_s2)
            || !positive(parameters_.max_big_velocity_rad_s)
            || !positive(parameters_.max_big_acceleration_rad_s2)
            || parameters_.max_big_velocity_rad_s > parameters_.max_small_velocity_rad_s
            || parameters_.max_big_acceleration_rad_s2 > parameters_.max_small_acceleration_rad_s2
            || parameters_.max_small_offset_rad <= parameters_.follow_deadband_rad
                + parameters_.max_big_velocity_rad_s / parameters_.follow_gain
            || (parameters_.follow_start_offset_rad > 0.0
                && (parameters_.follow_start_offset_rad <= parameters_.follow_deadband_rad
                    || parameters_.follow_start_offset_rad >= parameters_.max_small_offset_rad
                    || (parameters_.small_speed_zero_offset_rad > 0.0
                        && parameters_.follow_start_offset_rad >= parameters_.small_speed_zero_offset_rad)))
            || (parameters_.small_speed_zero_offset_rad > 0.0
                && (parameters_.small_speed_zero_offset_rad >= parameters_.max_small_offset_rad
                    || parameters_.small_speed_zero_offset_rad <= parameters_.follow_deadband_rad
                        + parameters_.max_big_velocity_rad_s / parameters_.follow_gain)))
            throw std::invalid_argument("invalid coordinated yaw parameters");
    }

    CoordinatedYawOutput update(
        double requested_heading_rate, double big_unwrapped_angle, double small_wrapped_angle,
        double measured_big_velocity, double dt, bool permitted) {
        if (!permitted || !std::isfinite(requested_heading_rate)
            || !std::isfinite(big_unwrapped_angle) || !std::isfinite(small_wrapped_angle)
            || !std::isfinite(measured_big_velocity) || !std::isfinite(dt)
            || dt <= 0.0 || dt > 0.02) {
            reset();
            return {};
        }

        CoordinatedYawOutput output;
        output.active = true;
        if (!initialized_) {
            initialized_ = true;
            big_origin_ = previous_big_ = big_unwrapped_angle;
            small_center_ = previous_small_wrapped_ = small_wrapped_angle;
            output.small_center_angle = small_center_;
            return output;
        }

        const double small_step = std::remainder(
            small_wrapped_angle - previous_small_wrapped_, 2.0 * std::numbers::pi);
        const double big_step = big_unwrapped_angle - previous_big_;
        // CAN angle samples can remain unchanged across multiple control ticks.
        // Check the step against elapsed observation time, capped by the same
        // 50 ms continuity window used by the hardware feedback path.
        big_angle_elapsed_ = std::min(0.05, big_angle_elapsed_ + dt);
        small_angle_elapsed_ = std::min(0.05, small_angle_elapsed_ + dt);
        // Permit encoder quantization and asynchronous CAN refreshes, but do
        // not chase a discontinuity whose accumulated angle is now ambiguous.
        if (!std::isfinite(small_step) || !std::isfinite(big_step)
            || std::abs(small_step) > parameters_.max_small_velocity_rad_s * small_angle_elapsed_ + 0.03
            || std::abs(big_step) > std::max(2.0, parameters_.max_big_velocity_rad_s)
                * big_angle_elapsed_ + 0.03) {
            reset();
            return {};
        }
        small_position_ += small_step;
        if (!std::isfinite(small_position_)
            || std::abs(small_position_) > parameters_.max_small_offset_rad) {
            reset();
            return {};
        }
        previous_small_wrapped_ = small_wrapped_angle;
        previous_big_ = big_unwrapped_angle;
        if (small_step != 0.0)
            small_angle_elapsed_ = 0.0;
        if (big_step != 0.0)
            big_angle_elapsed_ = 0.0;
        const double heading = big_unwrapped_angle - big_origin_
            + parameters_.small_yaw_direction * small_position_;
        if (!std::isfinite(heading)) {
            reset();
            return {};
        }

        // A sustained input must be slow enough for big yaw eventually to
        // take over; otherwise the small axis would run out of joint travel.
        const double accepted_rate = std::clamp(
            requested_heading_rate, -parameters_.max_big_velocity_rad_s,
            parameters_.max_big_velocity_rad_s);
        // Anti-accumulation limits only NEW input increments that would add
        // outward lead. An external disturbance or a slow handoff must not
        // drag an existing, centered-stick target along with actual heading.
        const double requested_increment = accepted_rate * dt;
        const double increment = requested_increment > 0.0
            ? std::min(requested_increment,
                       std::max(0.0, heading + parameters_.max_target_error_rad - target_))
            : std::max(requested_increment,
                       std::min(0.0, heading - parameters_.max_target_error_rad - target_));
        target_ += increment;
        output.reference_limited = accepted_rate != requested_heading_rate
            || increment != requested_increment;
        const double error = target_ - heading;
        // Only soften settled angle correction after the stick is centered.
        // This continuous deadband does not gate the independent compensation
        // for actual big-yaw motion, including its handoff through zero error.
        const double feedback_error = accepted_rate == 0.0
            ? std::copysign(std::max(0.0, std::abs(error) - parameters_.heading_error_deadband_rad), error)
            : error;

        // Subtract the deadband continuously so crossing its boundary does not
        // create a follow-speed step. Following starts from actual deflection,
        // without waiting for the small axis to reach the heading target.
        const double restoring_offset = std::copysign(
            std::max(0.0, std::abs(small_position_) - parameters_.follow_deadband_rad),
            small_position_);
        if (parameters_.follow_start_offset_rad == 0.0) {
            following_ = true;
        } else if (!following_) {
            following_ = std::abs(small_position_) >= parameters_.follow_start_offset_rad;
        } else {
            // Starting later must not leave the small joint permanently at the
            // start angle. Finish the handoff through the original center band.
            // Require a quiet interval so coasting through center does not
            // disable following while either actuator is still settling.
            const double quiet_speed = parameters_.big_velocity_noise_threshold_rad_s;
            const bool quiet = std::abs(small_position_) <= parameters_.follow_deadband_rad
                && std::abs(measured_big_velocity) <= quiet_speed
                && std::abs(big_velocity_) <= quiet_speed
                && std::abs(small_velocity_) <= quiet_speed;
            follow_quiet_time_ = quiet ? follow_quiet_time_ + dt : 0.0;
            if (follow_quiet_time_ >= std::max(0.05, parameters_.small_braking_response_time_s)) {
                following_ = false;
                follow_quiet_time_ = 0.0;
            }
        }
        const double follow_gain = accepted_rate == 0.0 && parameters_.centered_follow_gain > 0.0
            ? parameters_.centered_follow_gain : parameters_.follow_gain;
        const double requested_big = following_
            ? parameters_.small_yaw_direction * follow_gain * restoring_offset : 0.0;
        // DM speed feedback dithers by up to about 0.018 rad/s at rest.
        // Suppress only that feedforward noise; raw encoder-composed heading
        // still corrects genuine slow motion, with a small transient angle
        // error rather than promising exact instantaneous cancellation.
        const double compensation_velocity =
            std::abs(measured_big_velocity) <= parameters_.big_velocity_noise_threshold_rad_s
                ? 0.0 : measured_big_velocity;
        const double requested_small = parameters_.small_yaw_direction
            * (accepted_rate + parameters_.heading_gain * feedback_error - compensation_velocity);
        if (!std::isfinite(requested_big) || !std::isfinite(requested_small)
            || !std::isfinite(target_) || !std::isfinite(error)) {
            reset();
            return {};
        }
        const double previous_small_velocity = small_velocity_;
        const double previous_big_velocity = big_velocity_;
        double small_lower = -parameters_.max_small_velocity_rad_s;
        double small_upper = parameters_.max_small_velocity_rad_s;
        if (parameters_.small_speed_zero_offset_rad > 0.0) {
            // Reserve sample/actuator response travel plus stopping distance:
            // v*(dt+response_time) + v^2/(2*a) <= remaining angle. Brake before
            // the boundary, including when crossing center from the other side.
            // Bounds are directional: returning from +boundary keeps negative
            // authority, and returning from -boundary keeps positive authority.
            const double deceleration = parameters_.max_small_acceleration_rad_s2;
            const auto braking_speed = [&](double remaining) {
                if (remaining <= 0.0)
                    return 0.0;
                const double step = deceleration * (dt + parameters_.small_braking_response_time_s);
                return std::min(parameters_.max_small_velocity_rad_s,
                    std::sqrt(step * step + 2.0 * deceleration * remaining) - step);
            };
            small_lower = -braking_speed(parameters_.small_speed_zero_offset_rad + small_position_);
            small_upper = braking_speed(parameters_.small_speed_zero_offset_rad - small_position_);
        }
        const double bounded_small = std::clamp(requested_small, small_lower, small_upper);
        output.reference_limited = output.reference_limited || bounded_small != requested_small;
        small_velocity_ = ramp(
            bounded_small, small_velocity_, parameters_.max_small_velocity_rad_s,
            parameters_.max_small_acceleration_rad_s2 * dt, output.reference_limited);
        // A delayed actuator or an external push may leave no acceleration-
        // limited solution inside the updated envelope. In that case stopping
        // outward demand takes precedence; this is a reference limit, not a
        // guarantee that the physical joint cannot coast past the boundary.
        const double safe_small = std::clamp(small_velocity_, small_lower, small_upper);
        output.reference_limited = output.reference_limited || safe_small != small_velocity_;
        small_velocity_ = safe_small;
        double achievable_big = requested_big;
        if (parameters_.counterrotation_speed_fraction > 0.0) {
            const double fraction = parameters_.counterrotation_speed_fraction;
            const double positive_capacity = parameters_.small_yaw_direction > 0.0
                ? -small_lower : small_upper;
            const double negative_capacity = parameters_.small_yaw_direction > 0.0
                ? small_upper : -small_lower;
            achievable_big = std::clamp(
                requested_big, -fraction * negative_capacity, fraction * positive_capacity);
            output.reference_limited = output.reference_limited || achievable_big != requested_big;
        }
        big_velocity_ = ramp(
            achievable_big, big_velocity_, parameters_.max_big_velocity_rad_s,
            parameters_.max_big_acceleration_rad_s2 * dt, output.reference_limited);

        output.target_angle = target_;
        output.heading_angle = heading;
        output.heading_error = error;
        output.small_center_angle = small_center_;
        output.small_offset = small_position_;
        output.small_velocity = small_velocity_;
        // Keep inertial feedforward inside the actuator reference contract,
        // even if a boundary projection had to remove outward velocity faster.
        output.small_acceleration = std::clamp(
            (small_velocity_ - previous_small_velocity) / dt,
            -parameters_.max_small_acceleration_rad_s2,
            parameters_.max_small_acceleration_rad_s2);
        output.big_velocity = big_velocity_;
        output.big_acceleration = (big_velocity_ - previous_big_velocity) / dt;
        return output;
    }

    void reset() {
        initialized_ = false;
        following_ = false;
        follow_quiet_time_ = 0.0;
        big_origin_ = previous_big_ = previous_small_wrapped_ = small_center_ = 0.0;
        big_angle_elapsed_ = 0.0;
        small_angle_elapsed_ = 0.0;
        small_position_ = target_ = small_velocity_ = big_velocity_ = 0.0;
    }

private:
    static double ramp(double requested, double previous, double velocity_limit,
                       double maximum_step, bool& limited) {
        const double bounded = std::clamp(requested, -velocity_limit, velocity_limit);
        const double next = previous + std::clamp(bounded - previous, -maximum_step, maximum_step);
        limited = limited || bounded != requested || next != bounded;
        return next;
    }

    CoordinatedYawParameters parameters_;
    bool initialized_ = false;
    bool following_ = false;
    double follow_quiet_time_ = 0.0;
    double big_origin_ = 0.0;
    double previous_big_ = 0.0;
    double big_angle_elapsed_ = 0.0;
    double small_angle_elapsed_ = 0.0;
    double previous_small_wrapped_ = 0.0;
    double small_center_ = 0.0;
    double small_position_ = 0.0;
    double target_ = 0.0;
    double small_velocity_ = 0.0;
    double big_velocity_ = 0.0;
};

} // namespace rmcs_core::controller::gimbal
