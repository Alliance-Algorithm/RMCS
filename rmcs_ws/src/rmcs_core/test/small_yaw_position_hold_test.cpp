#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "hardware/small_yaw_damping.hpp"
#include "hardware/small_yaw_position_hold.hpp"

namespace {

using rmcs_core::hardware::SmallYawMotionEstimator;
using rmcs_core::hardware::SmallYawPositionHold;
using rmcs_core::hardware::SmallYawPositionHoldParameters;

constexpr double kEncoderStep = 2.0 * std::numbers::pi / 65536.0;
constexpr double kCurrentStepNm = 3.6 * 66.0 / 4096.0;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

void near(double actual, double expected, const char* message, double tolerance = 1e-10) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
        throw std::runtime_error(std::string{message} + ": actual=" + std::to_string(actual)
                                 + ", expected=" + std::to_string(expected));
}

SmallYawPositionHoldParameters enabled_parameters() {
    SmallYawPositionHoldParameters parameters;
    parameters.enabled = true;
    return parameters;
}

SmallYawPositionHoldParameters sliding_parameters() {
    auto parameters = enabled_parameters();
    parameters.sliding_enabled = true;
    parameters.torque_limit_nm = 10.0;
    parameters.torque_slew_nm_s = 18.0;
    return parameters;
}

void capture_and_rearm_hold_the_current_angle() {
    SmallYawPositionHold hold{enabled_parameters()};
    near(hold.update(0.7, 0.0, true, 0.001), 0.0, "capture must not kick the motor");
    require(hold.reference_valid(), "capture must publish a valid reference");
    near(hold.target_angle(), 0.7, "hold must capture current angle rather than fixed zero");
    require(hold.update(0.75, 0.0, true, 0.001) < 0.0,
            "positive displacement must produce restoring torque");
    near(hold.position_error(), -0.05, "telemetry must expose signed angular error");
    near(hold.update(0.75, 0.0, false, 0.001), 0.0, "protection must immediately remove torque");
    require(!hold.reference_valid(), "protection must clear the old target");
    near(hold.update(-0.6, 0.3, true, 0.001), 0.0, "unlock must capture without a kick");
    near(hold.target_angle(), -0.6, "unlock must adopt the new physical angle");
    near(hold.position_error(), 0.0, "unlock must clear previous error");
    hold.reset();
    require(!hold.reference_valid(), "explicit reset must invalidate target");
    SmallYawPositionHold disabled{SmallYawPositionHoldParameters{}};
    near(disabled.update(0.0, 0.0, true, 0.001), 0.0, "hold must default to disabled");
    require(!disabled.reference_valid(), "disabled hold must not capture a reference");
}

void angle_wrap_and_one_count_jitter_do_not_kick() {
    for (const double direction : {-1.0, 1.0}) {
        SmallYawPositionHold hold{enabled_parameters()};
        const double target = direction * (std::numbers::pi - 0.001);
        hold.update(target, 0.0, true, 0.001);
        const double wrapped = -direction * (std::numbers::pi - 0.001);
        const double torque = hold.update(wrapped, 0.0, true, 0.001);
        near(hold.position_error(), -direction * 0.002, "wrap must preserve the short angular error");
        near(torque, -direction * enabled_parameters().position_gain_nm_rad * 0.002,
             "crossing +/- pi must not saturate or reverse restoring torque");
    }
    SmallYawPositionHold hold{enabled_parameters()};
    hold.update(1.0, 0.0, true, 0.001);
    for (int cycle = 0; cycle < 100; ++cycle) {
        const double torque = hold.update(1.0 + (cycle % 2) * kEncoderStep, 0.0, true, 0.001);
        near(std::round(torque / kCurrentStepNm), 0.0,
             "one-count stationary jitter must not create an encoded motor current");
    }
}

void invalid_inputs_and_scheduling_gaps_clear_the_reference() {
    for (const double invalid : {kNaN, std::numeric_limits<double>::infinity(),
                                 -std::numeric_limits<double>::infinity()}) {
        for (int field = 0; field < 3; ++field) {
            SmallYawPositionHold hold{enabled_parameters()};
            hold.update(0.0, 0.0, true, 0.001);
            hold.update(0.1, 0.0, true, 0.001);
            near(hold.update(field == 0 ? invalid : 0.1, field == 1 ? invalid : 0.0,
                             true, field == 2 ? invalid : 0.001),
                 0.0, "invalid inputs must immediately remove torque");
            require(!hold.reference_valid(), "invalid inputs must discard the target");
            near(hold.update(0.1, 0.0, true, 0.001), 0.0,
                 "recovered input must capture a new target without a kick");
        }
    }
    for (const double dt : {-0.001, 0.0, 0.020001, 1.0}) {
        SmallYawPositionHold hold{enabled_parameters()};
        hold.update(0.0, 0.0, true, 0.001);
        hold.update(0.1, 0.0, true, 0.001);
        near(hold.update(0.1, 0.0, true, dt), 0.0, "invalid dt must remove torque");
        require(!hold.reference_valid(), "invalid dt must require a fresh capture");
    }
}

void parameter_validation_and_command_reversal() {
    using Parameters = SmallYawPositionHoldParameters;
    for (const auto field : {&Parameters::position_gain_nm_rad, &Parameters::velocity_gain_nm_s_rad,
                             &Parameters::torque_limit_nm, &Parameters::torque_slew_nm_s,
                             &Parameters::position_deadband_rad}) {
        auto parameters = enabled_parameters();
        parameters.*field = kNaN;
        bool rejected = false;
        try {
            const SmallYawPositionHold hold{parameters};
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        require(rejected, "nonfinite parameters must be rejected");
    }
    for (const double limit : {-1.0, 0.0, 2.50001}) {
        auto parameters = enabled_parameters();
        parameters.torque_limit_nm = limit;
        bool rejected = false;
        try {
            const SmallYawPositionHold hold{parameters};
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        require(rejected, "torque limits outside (0,2.5] must be rejected");
    }
    SmallYawPositionHold hold{enabled_parameters()};
    hold.update(0.0, 0.0, true, 0.001);
    for (int cycle = 0; cycle < 100; ++cycle)
        hold.update(0.2, 0.0, true, 0.001);
    near(hold.update(0.2, 0.0, true, 0.001), -2.5, "torque must saturate at 2.5 Nm");
    require(hold.update(-0.2, 0.0, true, 0.001) > 0.0,
            "opposite restoring demand must immediately discard stale torque sign");
}

void sliding_capture_restores_signed_relative_angle_across_wrap() {
    for (const double direction : {-1.0, 1.0}) {
        SmallYawPositionHold hold{sliding_parameters()};
        near(hold.update(0.7, 0.0, true, 0.001), 0.0,
             "sliding capture must not kick the motor");
        near(hold.target_angle(), 0.7, "sliding must capture the relative motor angle");
        near(hold.target_velocity(), 0.0, "capture must start with a zero velocity reference");
        const double torque = hold.update(0.7 + direction * 0.01, 0.0, true, 0.001);
        require(direction * torque < 0.0, "sliding torque must restore the captured angle");
        near(hold.target_velocity(), -direction * 0.04,
             "signed position error must generate the restoring velocity reference");

        hold.reset();
        hold.update(direction * (std::numbers::pi - 0.001), 0.0, true, 0.001);
        const double wrap_torque = hold.update(
            -direction * (std::numbers::pi - 0.001), 0.0, true, 0.001);
        near(hold.position_error(), -direction * 0.002,
             "sliding position error must use the shortest path across the wrap");
        near(hold.target_velocity(), -direction * 0.008,
             "wrapped error must produce a small velocity reference");
        require(direction * wrap_torque < 0.0,
                "wrapped sliding torque must have the restoring sign");
    }
}

void sliding_torque_and_reference_limits_preserve_slew_and_reversal() {
    auto parameters = sliding_parameters();
    parameters.sliding_model.inertia = 1.0;
    parameters.sliding_model.tracking_gain = 100.0;
    parameters.torque_slew_nm_s = 100.0;
    SmallYawPositionHold hold{parameters};
    hold.update(0.0, 0.0, true, 0.001);
    near(hold.update(0.5, 0.0, true, 0.001), -0.1,
         "sliding torque buildup must obey the existing slew limit");
    near(hold.target_velocity(), -0.35, "large position errors must bound target speed");
    double torque = 0.0;
    for (int cycle = 0; cycle < 300; ++cycle) {
        torque = hold.update(0.5, 0.0, true, 0.001);
        require(std::abs(torque) <= 10.0, "sliding output must never exceed 10 Nm");
    }
    near(torque, -10.0, "sliding hold must allow output above the old 2.5 Nm cap");
    near(hold.update(-0.5, 0.0, true, 0.001), 0.1,
         "restoring reversal must discard stale torque before rebuilding");
    near(hold.target_velocity(), 0.35, "positive restoring reference must also be bounded");
}

void sliding_deadband_brakes_motion_without_position_feedforward() {
    SmallYawPositionHold hold{sliding_parameters()};
    hold.update(1.0, 0.0, true, 0.001);
    near(hold.update(1.0 + kEncoderStep, 0.0, true, 0.001), 0.0,
         "a stationary one-count offset must not request a sliding restoring torque");
    near(hold.target_velocity(), 0.0, "the existing position deadband must suppress the reference");
    const double braking = hold.update(1.0 + kEncoderStep, 0.2, true, 0.001);
    require(braking < 0.0, "motion inside the position deadband must still be braked");
    for (int cycle = 0; cycle < 30; ++cycle)
        hold.update(1.0 + kEncoderStep, 0.0, true, 0.001);
    near(hold.update(1.0 + kEncoderStep, 0.0, true, 0.001), 0.0,
         "a stationary offset inside the deadband must settle to zero torque");

    auto parameters = sliding_parameters();
    parameters.sliding_model.inertia = 0.311472;
    parameters.sliding_model.tracking_gain = 8.0;
    parameters.sliding_model.robust_gain = 2.0;
    SmallYawPositionHold noisy_hold{parameters};
    noisy_hold.update(1.0, 0.0, true, 0.001);
    double peak_torque = 0.0;
    for (int cycle = 0; cycle < 1000; ++cycle) {
        // A single count alternates on each 100 Hz feedback frame. The angle
        // deadband removes the reference, but encoder-derived braking remains.
        const double angle = 1.0 + ((cycle / 10) % 2) * kEncoderStep;
        const double torque = noisy_hold.update(angle, 0.0, true, 0.001);
        near(noisy_hold.target_velocity(), 0.0,
             "one-count jitter must remain inside the position-reference deadband");
        require(torque * noisy_hold.velocity() <= 0.0,
                "deadband jitter may only produce dissipative braking");
        peak_torque = std::max(peak_torque, std::abs(torque));
    }
    std::cout << "  sliding one-count jitter peak torque=" << peak_torque << '\n';
    require(peak_torque < 0.25,
            "one-count feedback jitter must remain bounded by the buildup slew limit");
}

void sliding_uses_sign_law_and_a_local_model_without_integral() {
    auto parameters = sliding_parameters();
    parameters.torque_slew_nm_s = 10000.0;
    parameters.sliding_model.inertia = 1.0;
    parameters.sliding_model.tracking_gain = 1.0;
    parameters.sliding_model.robust_gain = 2.0;
    parameters.sliding_model.boundary_width = 100.0;
    parameters.sliding_model.auxiliary_law = rmcs_core::controller::robust::AuxiliaryLaw::Smooth;
    parameters.sliding_model.integral_gain = 100.0;
    parameters.sliding_model.integral_torque_limit = 5.0;
    parameters.sliding_model.output_min = -0.01;
    parameters.sliding_model.output_max = 0.01;
    SmallYawPositionHold hold{parameters};
    hold.update(0.0, 0.0, true, 0.001);
    // 0.025 rad * 4/s = 0.1 rad/s; J * (k * e + rho * sign(e)) = 2.1 Nm.
    near(hold.update(-0.025, 0.0, true, 0.001), 2.1,
         "hold must override the incoming smooth law and output limits locally");
    for (int cycle = 0; cycle < 200; ++cycle)
        hold.update(-0.025, 0.0, true, 0.001);
    near(hold.update(-0.025, 0.0, true, 0.001), 2.1,
         "position hold must not accumulate the velocity model's load integral");
}

void sliding_protection_resets_reference_velocity_and_rearms_without_kick() {
    for (int fault = 0; fault < 4; ++fault) {
        SmallYawPositionHold hold{sliding_parameters()};
        hold.update(0.7, 0.0, true, 0.001);
        hold.update(0.8, 0.0, true, 0.001);
        near(hold.update(fault == 1 ? kNaN : 0.8, fault == 2 ? kNaN : 0.0,
                         fault != 0, fault == 3 ? 0.020001 : 0.001),
             0.0, "sliding protection must immediately remove torque");
        require(!hold.reference_valid(), "sliding protection must discard the captured reference");
        near(hold.target_velocity(), 0.0, "sliding protection must clear the velocity reference");
        near(hold.update(-0.6, 0.0, true, 0.001), 0.0,
             "sliding rearm must capture a fresh angle without retaining torque");
        near(hold.target_angle(), -0.6, "sliding rearm must adopt the new physical angle");
    }
    using Parameters = SmallYawPositionHoldParameters;
    for (const auto field : {&Parameters::sliding_position_gain,
                             &Parameters::sliding_velocity_limit_rad_s}) {
        for (const double invalid : {kNaN, 0.0, -0.1}) {
            auto parameters = sliding_parameters();
            parameters.*field = invalid;
            bool rejected = false;
            try {
                const SmallYawPositionHold hold{parameters};
            } catch (const std::invalid_argument&) {
                rejected = true;
            }
            require(rejected, "invalid sliding reference parameters must be rejected");
        }
    }
    for (const double limit : {-1.0, 0.0, 10.00001}) {
        auto parameters = sliding_parameters();
        parameters.torque_limit_nm = limit;
        bool rejected = false;
        try {
            const SmallYawPositionHold hold{parameters};
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        require(rejected, "sliding torque limits outside (0,10] must be rejected");
    }
}

struct MotionResult {
    double maximum_displacement = 0.0;
    double final_error = 0.0;
    double final_speed = 0.0;
    double tail_range = 0.0;
    double tail_peak_speed = 0.0;
    double tail_average_speed = 0.0;
};

// Exercise the whole sampled loop: 100 Hz, 16-bit encoder feedback; the existing
// 40 ms external velocity estimator; 500 Hz quantized LK current delivery; and a
// frictionless inertial plant. This catches delay-induced instability that a
// test feeding exact velocity directly into the PD law would miss.
MotionResult simulate_disturbance(
    double inertia, double direction, int phase, bool jitter = false, bool sliding = false) {
    auto parameters = sliding ? sliding_parameters() : enabled_parameters();
    if (sliding) {
        parameters.sliding_model.inertia = inertia;
        parameters.sliding_model.tracking_gain = 8.0;
        parameters.sliding_model.robust_gain = 2.0;
    }
    SmallYawPositionHold hold{parameters};
    SmallYawMotionEstimator external_velocity;
    constexpr double dt = 0.001;
    constexpr double target = 0.35;
    double angle = target, velocity = 0.0;
    double feedback = std::round(angle / kEncoderStep) * kEncoderStep;
    double command = 0.0;
    double tail_min = 1e9, tail_max = -1e9;
    double tail_start = 0.0;
    int next_feedback_cycle = phase, next_control_cycle = 0;
    int previous_control_cycle = -1, previous_send_cycle = -1000;
    std::size_t feedback_index = 0, control_index = 0;
    constexpr std::array feedback_periods{8, 12, 9, 11, 10};
    constexpr std::array control_periods{1, 1, 2, 1, 1, 1, 2};
    MotionResult result;
    for (int cycle = 0; cycle < 8000; ++cycle) {
        if (cycle >= next_feedback_cycle) {
            feedback = std::remainder(std::round(angle / kEncoderStep) * kEncoderStep,
                                      2.0 * std::numbers::pi);
            next_feedback_cycle = cycle + (jitter ? feedback_periods[feedback_index++ % 5] : 10);
            if (jitter && cycle >= 1000 && cycle < 1012)
                next_feedback_cycle += 10;
        }
        if (cycle >= next_control_cycle) {
            const double control_dt = (cycle - previous_control_cycle) * dt;
            const double estimated_velocity = external_velocity.update(feedback, control_dt);
            const double requested = hold.update(feedback, estimated_velocity, true, control_dt);
            // Match the hardware's elapsed-time scheduler. Every immediate
            // withdrawal or reversal also restarts its normal 2 ms timer.
            if (cycle - previous_send_cycle >= 2
                || (command != 0.0 && (requested == 0.0 || command * requested < 0.0))) {
                command = std::round(requested / kCurrentStepNm) * kCurrentStepNm;
                previous_send_cycle = cycle;
            }
            previous_control_cycle = cycle;
            next_control_cycle = cycle + (jitter ? control_periods[control_index++ % 7] : 1);
        }
        require(std::isfinite(command) && std::abs(command) <= parameters.torque_limit_nm,
                "sampled motor command must remain bounded");
        const double external_torque = cycle >= 200 && cycle < 350 ? direction * 2.0 : 0.0;
        velocity += (command + external_torque) / inertia * dt;
        angle += velocity * dt;
        result.maximum_displacement =
            std::max(result.maximum_displacement, std::abs(angle - hold.target_angle()));
        if (cycle == 6000)
            tail_start = angle;
        if (cycle >= 6000) {
            tail_min = std::min(tail_min, angle);
            tail_max = std::max(tail_max, angle);
            result.tail_peak_speed = std::max(result.tail_peak_speed, std::abs(velocity));
        }
    }
    result.final_error = std::abs(angle - hold.target_angle());
    result.final_speed = std::abs(velocity);
    result.tail_range = tail_max - tail_min;
    result.tail_average_speed = std::abs(angle - tail_start) / 2.0;
    return result;
}

void sampled_sliding_disturbance_remains_bounded_and_returns() {
    for (const double direction : {-1.0, 1.0}) {
        for (const int phase : {0, 9}) {
            const auto motion = simulate_disturbance(0.311472, direction, phase, true, true);
            std::cout << "  sliding direction=" << direction << " phase=" << phase
                      << " displacement=" << motion.maximum_displacement
                      << " final_error=" << motion.final_error
                      << " tail_range=" << motion.tail_range
                      << " tail_peak_speed=" << motion.tail_peak_speed << '\n';
            require(motion.maximum_displacement > 0.002,
                    "disturbance must actually move the sampled sliding plant");
            require(motion.final_error < 0.001,
                    "sampled sliding loop must return near the captured angle");
            require(motion.tail_range < 0.001,
                    "sampled sliding loop must not develop a growing position oscillation");
            require(motion.tail_peak_speed < 0.02,
                    "sampled sliding loop must settle to a bounded speed ripple");
        }
    }
}

void sampled_disturbance_release_returns_with_bounded_quantization_motion() {
    for (const double inertia : {0.02, 0.08, 0.2}) {
        for (const double direction : {-1.0, 1.0}) {
            for (const int phase : {0, 3, 9}) for (const bool jitter : {false, true}) {
                const auto motion = simulate_disturbance(inertia, direction, phase, jitter);
                std::cout << "  J=" << inertia << " direction=" << direction << " phase=" << phase
                          << " jitter=" << jitter << " displacement=" << motion.maximum_displacement
                          << " final_error=" << motion.final_error
                          << " tail_range=" << motion.tail_range
                          << " tail_peak_speed=" << motion.tail_peak_speed
                          << " tail_average_speed=" << motion.tail_average_speed << '\n';
                require(motion.maximum_displacement > 0.002,
                        "disturbance must actually move the simulated plant");
                require(motion.final_error < 0.002,
                        "released disturbance must return within 0.002 rad of captured angle");
                require(motion.tail_range < 0.001,
                        "settled position micro-motion must remain below 0.001 rad");
                // A single raw-current increment retained until fresh feedback
                // changes speed by quantum * period / J. Requiring a lower
                // peak than this would reject unavoidable current quantization
                // even when angle is confined to its encoder/deadband region.
                const double quantized_velocity_step = kCurrentStepNm * 0.012 / inertia;
                require(motion.tail_peak_speed < quantized_velocity_step + 0.003,
                        "settled speed must remain bounded by one current quantum");
                require(motion.tail_average_speed < 0.0003,
                        "settled loop must have negligible long-window angular drift");
            }
        }
    }
}

} // namespace

int main() {
    using Test = void (*)();
    const std::array<std::pair<const char*, Test>, 11> tests{{
        {"capture_and_rearm_hold_the_current_angle", capture_and_rearm_hold_the_current_angle},
        {"angle_wrap_and_one_count_jitter_do_not_kick", angle_wrap_and_one_count_jitter_do_not_kick},
        {"invalid_inputs_and_scheduling_gaps_clear_the_reference",
         invalid_inputs_and_scheduling_gaps_clear_the_reference},
        {"parameter_validation_and_command_reversal", parameter_validation_and_command_reversal},
        {"sliding_capture_restores_signed_relative_angle_across_wrap",
         sliding_capture_restores_signed_relative_angle_across_wrap},
        {"sliding_torque_and_reference_limits_preserve_slew_and_reversal",
         sliding_torque_and_reference_limits_preserve_slew_and_reversal},
        {"sliding_deadband_brakes_motion_without_position_feedforward",
         sliding_deadband_brakes_motion_without_position_feedforward},
        {"sliding_uses_sign_law_and_a_local_model_without_integral",
         sliding_uses_sign_law_and_a_local_model_without_integral},
        {"sliding_protection_resets_reference_velocity_and_rearms_without_kick",
         sliding_protection_resets_reference_velocity_and_rearms_without_kick},
        {"sampled_sliding_disturbance_remains_bounded_and_returns",
         sampled_sliding_disturbance_remains_bounded_and_returns},
        {"sampled_disturbance_release_returns_with_bounded_quantization_motion",
         sampled_disturbance_release_returns_with_bounded_quantization_motion},
    }};
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try {
            test();
            std::cout << "PASS " << name << '\n';
        } catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL " << name << ": " << error.what() << '\n';
        }
    }
    std::cout << tests.size() - failures << '/' << tests.size() << " tests passed\n";
    return failures == 0 ? 0 : 1;
}
