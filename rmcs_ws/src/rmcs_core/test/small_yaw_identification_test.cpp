#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>

#include "hardware/small_yaw_identification.hpp"

namespace {
using rmcs_core::hardware::SmallYawIdentification;
using rmcs_core::hardware::SmallYawIdentificationInput;
using rmcs_core::hardware::SmallYawIdentificationParameters;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}
void near(double actual, double expected, const char* message, double tolerance = 1e-9) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
        throw std::runtime_error(std::string{message} + ": actual=" + std::to_string(actual)
                                 + ", expected=" + std::to_string(expected));
}
SmallYawIdentificationParameters enabled_parameters() {
    SmallYawIdentificationParameters p;
    p.enabled = true;
    p.command_sign = 1.0;
    p.torque_limit_nm = 6.0;
    p.sine_torque_amplitude_nm = 6.0;
    p.sine_frequency_hz = 0.15;
    p.sine_frequency_end_hz = 0.60;
    p.sine_duration_s = 16.0;
    p.constant_torque_nm = 4.5;
    p.constant_duration_s = 2.0;
    p.constant_rest_duration_s = 0.5;
    p.linear_ramp_peak_nm = 5.0;
    p.linear_ramp_duration_s = 4.0;
    p.linear_ramp_group_count = 2;
    p.max_velocity_rad_s = 1.0;
    return p;
}
SmallYawIdentificationInput valid_input() {
    SmallYawIdentificationInput input;
    input.remote_valid = input.feedback_valid = true;
    input.joint2_permitted = true;
    input.both_down = false;
    return input;
}
void reach_auto_start(SmallYawIdentification& sequence, SmallYawIdentificationInput& input) {
    input.both_down = true;
    near(sequence.update(input), 0.0, "double-down cannot produce torque");
    input.both_down = false;
    bool started = false;
    for (int i = 0; i < 2000 && !started; ++i) {
        near(sequence.update(input), 0.0, "joint2 stabilization cannot produce torque");
        started = sequence.active();
    }
    require(started, "stable joint2 must automatically start the baseline");
    require(std::string_view{sequence.state()} == "settling",
            "stable joint2 must start automatically");
    require(sequence.permitted() && sequence.active() && sequence.origin_valid(),
            "automatic start captures the small-yaw origin");
    near(sequence.elapsed_s(), 0.0, "automatic start begins the zero-torque baseline");
}
void start(SmallYawIdentification& sequence, SmallYawIdentificationInput& input) {
    reach_auto_start(sequence, input);
}
void reach_first_pulse(SmallYawIdentification& sequence, SmallYawIdentificationInput& input) {
    start(sequence, input);
    double torque = 0.0;
    for (int i = 0; i < 1000; ++i)
        torque = sequence.update(input);
    near(torque, 2.0, "first pulse must follow one second baseline");
}

void waveform_is_balanced_and_runs_once() {
    SmallYawIdentification sequence{enabled_parameters()};
    auto input = valid_input();
    start(sequence, input);
    std::array<int, 16> counts{};
    double pulse_impulse = 0.0;
    constexpr std::array<double, 4> amplitudes{2.0, 3.5, 5.0, 6.0};
    constexpr std::array<int, 4> signs{1, -1, -1, 1};
    for (int tick = 1; tick <= 67900; ++tick) {
        const double torque = sequence.update(input);
        double expected = 0.0;
        if (tick >= 1000 && tick < 15400) {
            const int index = (tick - 1000) / 900;
            const int phase = (tick - 1000) % 900;
            if (phase < 200) {
                expected = amplitudes[index / 4] * signs[index % 4];
                ++counts[index];
            }
        } else if (tick >= 15400 && tick < 31400) {
            const double sine_elapsed = (tick - 15400) * input.dt;
            expected = 6.0 * std::sin(2.0 * std::numbers::pi * (
                0.15 * sine_elapsed + 0.5 * (0.60 - 0.15)
                * sine_elapsed * sine_elapsed / 16.0));
        } else if (tick >= 31400 && tick < 33400) {
            expected = 4.5;
        } else if (tick >= 33900 && tick < 35900) {
            expected = -4.5;
        } else if (tick >= 35900 && tick < 67900) {
            const int leg = (tick - 35900) / 4000;
            const double leg_elapsed = (tick - 35900 - leg * 4000) * input.dt;
            switch (leg % 4) {
            case 0: expected = 5.0 * leg_elapsed / 4.0; break;
            case 1: expected = 5.0 * (1.0 - leg_elapsed / 4.0); break;
            case 2: expected = -5.0 * leg_elapsed / 4.0; break;
            case 3: expected = -5.0 * (1.0 - leg_elapsed / 4.0); break;
            }
        }
        near(torque, expected, "identification waveform boundaries must match", 1e-8);
        if (tick >= 1000 && tick < 15400)
            pulse_impulse += torque * input.dt;
    }
    for (const auto count : counts)
        require(count == 200, "each of sixteen pulses must contain exactly 200 ms");
    near(pulse_impulse, 0.0, "positive and negative pulse torque impulse must balance");
    require(sequence.complete() && !sequence.active() && !sequence.permitted(),
            "completion must remove permission and latch");
    near(sequence.total_duration_s(), 67.9, "total duration includes all excitation segments");
    for (int i = 0; i < 2000; ++i) {
        near(sequence.update(input), 0.0, "completed run cannot retrigger without double-down");
    }
    reach_first_pulse(sequence, input);
    require(!sequence.complete(), "fresh double-down must reset completion");
}

void breakaway_staircase_has_five_groups_with_randomized_tail() {
    auto parameters = enabled_parameters();
    parameters.torque_limit_nm = 7.0;
    parameters.breakaway_staircase_enabled = true;
    parameters.breakaway_pulse_duration_s = 0.5;
    parameters.breakaway_rest_duration_s = 2.0;
    parameters.breakaway_group_count = 5;
    SmallYawIdentification sequence{parameters};
    auto input = valid_input();
    start(sequence, input);
    constexpr std::array<double, 8> levels{3.0, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0};
    constexpr int settle_ticks = 1000;
    constexpr int pulse_ticks = 500;
    constexpr int period_ticks = 2500;
    constexpr int pulses_per_group = 16;
    constexpr int group_ticks = pulses_per_group * period_ticks;
    constexpr int total_ticks = settle_ticks + 5 * group_ticks;
    require(sequence.total_duration_s() == 201.0,
            "breakaway staircase duration must be 201 seconds");
    constexpr std::array<std::array<double, 16>, 2> randomized{{
        {{5.5, -3.0, 6.5, -4.0, 7.0, -4.5, 6.0, -5.0,
          3.0, -6.5, 4.5, -7.0, 5.0, -5.5, 4.0, -6.0}},
        {{-6.0, 4.5, -3.0, 7.0, -5.0, 6.5, -4.0, 5.5,
          -7.0, 3.0, -5.5, 4.0, -6.5, 5.0, -4.5, 6.0}},
    }};
    for (int tick = 1; tick <= total_ticks; ++tick) {
        const double torque = sequence.update(input);
        double expected = 0.0;
        if (tick >= settle_ticks && tick < total_ticks) {
            const int pulse = (tick - settle_ticks) / period_ticks;
            const int phase = (tick - settle_ticks) % period_ticks;
            if (phase < pulse_ticks) {
                const int group = pulse / pulses_per_group;
                const int within_group = pulse % pulses_per_group;
                if (group < 3) {
                    const bool positive = within_group < 8;
                    const int level_index = positive ? within_group : within_group - 8;
                    expected = (positive ? 1.0 : -1.0) * levels[level_index];
                } else {
                    expected = randomized[group - 3][within_group];
                }
            }
        }
        near(torque, expected,
             "breakaway staircase pulse and rest timing must match", 1e-8);
    }
    require(sequence.complete() && !sequence.active() && !sequence.permitted(),
            "breakaway staircase must complete and latch safely");
}

void arming_requires_fresh_double_down_and_joint2_stability() {
    SmallYawIdentification sequence{enabled_parameters()};
    auto input = valid_input();
    for (int i = 0; i < 1200; ++i) {
        near(sequence.update(input), 0.0, "startup release must not bypass double-down");
    }
    require(!sequence.active() && !sequence.permitted(), "unarmed startup must fail closed");
    input.both_down = true;
    input.feedback_valid = false;
    sequence.update(input);
    input.feedback_valid = true;
    input.both_down = false;
    sequence.update(input);
    require(sequence.faulted(), "double-down with stale feedback must not arm");
    input.both_down = true;
    sequence.update(input);
    input.both_down = false;
    input.joint2_position_error = 0.2;
    for (int i = 0; i < 2000; ++i)
        sequence.update(input);
    require(!sequence.active(), "moving posture cannot start automatically");
    input.joint2_position_error = 8.0 * std::numbers::pi / 180.0;
    bool started = false;
    for (int i = 0; i < 1100 && !started; ++i) {
        near(sequence.update(input), 0.0,
             "joint2 stability starts only the zero-torque baseline");
        started = sequence.active();
    }
    require(sequence.active() && std::string_view{sequence.state()} == "settling",
            "joint2 stability alone must automatically start");
}

void every_running_interlock_withdraws_torque_and_latches() {
    constexpr std::array<const char*, 10> expected_reasons{
        "remote_invalid", "feedback_invalid", "feedback_invalid",
        "update_gap", "update_gap", "update_gap", "joint2_not_permitted",
        "joint2_unstable", "joint2_unstable", "velocity_limit"};
    for (std::size_t fault = 0; fault < expected_reasons.size(); ++fault) {
        SmallYawIdentification sequence{enabled_parameters()};
        auto input = valid_input();
        reach_first_pulse(sequence, input);
        switch (fault) {
        case 0: input.remote_valid = false; break;
        case 1: input.feedback_valid = false; break;
        case 2: input.angle = kNaN; break;
        case 3: input.dt = kNaN; break;
        case 4: input.dt = 0.0; break;
        case 5: input.dt = 0.020001; break;
        case 6: input.joint2_permitted = false; break;
        case 7: input.joint2_position_error = kNaN; break;
        case 8: input.joint2_angle = kNaN; break;
        case 9: input.dt = 0.01; input.angle += 0.02; break;
        }
        near(sequence.update(input), 0.0, "every interlock must remove an active pulse immediately");
        require(sequence.faulted() && !sequence.active() && !sequence.permitted(),
                "fault must revoke permission and latch");
        require(std::string_view{sequence.fault_reason()} == expected_reasons[fault],
                "fault telemetry must name first interlock failure");
        input = valid_input();
        for (int i = 0; i < 1500; ++i)
            near(sequence.update(input), 0.0, "restored inputs cannot restart faulted run");
        require(sequence.faulted(), "recovery requires actual double-down");
        input.both_down = true;
        near(sequence.update(input), 0.0, "double-down reset must still output zero");
        require(!sequence.faulted(), "fresh double-down must clear the latched fault");
    }
}

void output_velocity_below_one_rad_s_passes_and_above_one_stops() {
    SmallYawIdentification sequence{enabled_parameters()};
    auto input = valid_input();
    reach_first_pulse(sequence, input);
    for (int i = 0; i < 30; ++i) {
        input.angle += 0.0008;
        near(sequence.update(input), 2.0,
             "output-axis motion below one rad/s must not stop the pulse");
    }
    require(!sequence.faulted() && sequence.active(),
            "sub-limit output-axis velocity must remain permitted");

    input.angle += 0.004;
    near(sequence.update(input), 0.0,
         "output-axis velocity above one rad/s must remove pulse torque");
    require(sequence.faulted()
            && std::string_view{sequence.fault_reason()} == "velocity_limit",
            "angle-derived speed protection must remain active");
}

void estimator_handles_wrap_and_catches_motion_without_native_speed() {
    SmallYawIdentification sequence{enabled_parameters()};
    auto input = valid_input();
    input.angle = std::numbers::pi - 0.002;
    start(sequence, input);
    const double origin = input.angle;
    for (int i = 1; i <= 30; ++i) {
        input.angle = std::remainder(origin + i * 0.0003, 2.0 * std::numbers::pi);
        sequence.update(input);
        require(!sequence.faulted(), "small motion across encoder wrap must not cause false protection");
        near(sequence.relative_angle(), i * 0.0003, "wrapped origin displacement must stay continuous");
    }
    near(sequence.estimated_velocity(), 0.3, "ten millisecond angle estimate must preserve real speed");
    for (int i = 1; i <= 20; ++i) {
        input.angle = std::remainder(origin + 0.009 - i * 0.0003, 2.0 * std::numbers::pi);
        sequence.update(input);
        require(!sequence.faulted(), "ordinary reverse motion must stay continuous");
    }
    near(sequence.estimated_velocity(), -0.3, "estimator must reverse without resetting history");
    input.angle += 0.04;
    near(sequence.update(input), 0.0, "encoder speed protection must work when native speed reports zero");
    require(std::string_view{sequence.fault_reason()} == "velocity_limit",
            "short-window estimator must detect hidden fast motion");
}

void origin_is_captured_on_trigger_and_double_down_stops() {
    SmallYawIdentification sequence{enabled_parameters()};
    auto input = valid_input();
    reach_auto_start(sequence, input);
    for (int i = 0; i < 20; ++i) {
        input.angle += 0.0002;
        sequence.update(input);
    }
    near(sequence.relative_angle(), 0.004,
         "automatic start captures origin at joint2 readiness, after double-down");
    for (int i = 0; i < 1000; ++i)
        sequence.update(input);
    input.both_down = true;
    near(sequence.update(input), 0.0, "double-down must stop an active pulse immediately");
    require(!sequence.active() && !sequence.origin_valid() && !sequence.complete(),
            "double-down must clear active sequence and origin");
    input.both_down = false;
    for (int i = 0; i < 900; ++i) {
        near(sequence.update(input), 0.0,
             "double-down reset must wait for the fresh joint2 stability dwell");
        require(!sequence.active(), "automatic restart must wait for joint2 stability");
    }
    for (int i = 0; i < 200; ++i)
        sequence.update(input);
    require(sequence.active() && std::string_view{sequence.state()} == "settling",
            "double-down reset must rearm automatic identification after stable joint2 dwell");
}

void joint2_instability_is_the_only_posture_motion_interlock() {
    SmallYawIdentification sequence{enabled_parameters()};
    auto input = valid_input();
    input.both_down = true;
    sequence.update(input);
    input.both_down = false;
    for (int i = 0; i < 1260; ++i)
        sequence.update(input);
    require(sequence.active(), "joint2 stability must auto-start the sequence");
    for (int i = 0; i < 1800; ++i) {
        input.angle += 0.0005;
        sequence.update(input);
        require(!sequence.faulted(),
                "small yaw may travel beyond the removed relative-angle window below 1 rad/s");
    }
    require(sequence.relative_angle() > std::numbers::pi / 4.0,
            "relative angle remains telemetry and does not impose another motion limit");
}

void parameter_limits_and_disabled_mode() {
    using P = SmallYawIdentificationParameters;
    for (const double invalid_sign : {0.0, 2.0, kNaN}) {
        auto p = enabled_parameters();
        p.command_sign = invalid_sign;
        bool rejected = false;
        try { const SmallYawIdentification sequence{p}; }
        catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "command polarity must be exactly +1 or -1");
    }
    for (const auto field : {&P::torque_limit_nm, &P::pulse_duration_s, &P::rest_duration_s,
                             &P::settle_duration_s,
                             &P::constant_torque_nm, &P::constant_duration_s,
                             &P::constant_rest_duration_s, &P::linear_ramp_peak_nm,
                             &P::linear_ramp_duration_s,
                             &P::max_velocity_rad_s, &P::joint2_position_tolerance_rad,
                             &P::joint2_stability_window_s,
                             &P::joint2_stable_duration_s}) {
        for (const double invalid : {kNaN, -1.0, 0.0, 100.0}) {
            auto p = enabled_parameters();
            p.*field = invalid;
            bool rejected = false;
            try { const SmallYawIdentification sequence{p}; }
            catch (const std::invalid_argument&) { rejected = true; }
            require(rejected, "nonfinite and out-of-range identification parameters must be rejected");
        }
    }
    auto seven_nm_identification = enabled_parameters();
    seven_nm_identification.torque_limit_nm = 7.0;
    const SmallYawIdentification accepts_seven_nm{seven_nm_identification};
    for (const double robust_only_limit : {7.01, 10.0}) {
        auto p = enabled_parameters();
        p.torque_limit_nm = robust_only_limit;
        bool rejected = false;
        try { const SmallYawIdentification sequence{p}; }
        catch (const std::invalid_argument&) { rejected = true; }
        require(rejected,
                "the 10 Nm robust command capability must not expand the identification limit");
    }
    auto widened_velocity_limit = enabled_parameters();
    widened_velocity_limit.max_velocity_rad_s = 5.0;
    const SmallYawIdentification accepts_widened_velocity_limit{widened_velocity_limit};
    require(!accepts_widened_velocity_limit.faulted(),
            "the widened 5 rad/s velocity limit must be accepted");
    SmallYawIdentification disabled{SmallYawIdentificationParameters{}};
    auto input = valid_input();
    for (int i = 0; i < 2000; ++i)
        near(disabled.update(input), 0.0, "default configuration must stay disabled");
    require(!disabled.permitted() && !disabled.active(), "disabled sequence cannot permit commands");
    auto p = enabled_parameters();
    p.torque_limit_nm = 1.0;
    p.linear_ramp_peak_nm = 1.0;
    SmallYawIdentification bounded{p};
    start(bounded, input);
    for (int i = 0; i < 14000; ++i)
        require(std::abs(bounded.update(input)) <= 1.0, "configured torque cap must bound every fixed amplitude");
}

void joint2_angle_diagnostic_window_does_not_gate_stability() {
    SmallYawIdentification sequence{enabled_parameters()};
    auto input = valid_input();
    input.both_down = true;
    sequence.update(input);
    input.both_down = false;
    for (int i = 0; i < 1500; ++i) {
        input.joint2_angle = 0.9 + 0.0022 * std::sin(i * input.dt * std::numbers::pi);
        near(sequence.update(input), 0.0, "small posture jitter must never excite automatically");
        if (i < 250)
            require(!sequence.active(), "the one-second joint2 stability dwell is still running");
    }
    require(std::string_view{sequence.state()} == "settling" && sequence.active(),
            "stable posture with small angle variation must auto-start");
    require(sequence.joint2_window_ready(), "ready must have a full posture window");
    for (int i = 0; i < 68000; ++i) {
        input.joint2_angle = 0.9 + 0.0022 * std::sin((i + 1500) * input.dt * std::numbers::pi);
        sequence.update(input);
        require(!sequence.faulted(), "small joint2 angle variation must not abort valid pulses");
    }
    require(sequence.complete(), "stable posture must allow the complete excitation sequence");

}

void joint2_angle_span_is_diagnostic_only() {
    for (int mode = 0; mode < 3; ++mode) {
        SmallYawIdentification sequence{enabled_parameters()};
        auto input = valid_input();
        reach_first_pulse(sequence, input);
        for (int i = 0; i < 250; ++i) {
            if (mode == 0)
                input.joint2_angle = 0.025; // A real angle step during an active pulse.
            else if (mode == 1)
                input.joint2_angle = i % 2 ? -0.015 : 0.015; // Zero net displacement.
            else
                input.joint2_angle += 0.1 * input.dt; // Sustained drift.
            input.joint2_position_error = -input.joint2_angle;
            sequence.update(input);
            require(!sequence.faulted(),
                    "joint2 angle-span diagnostic must not trip identification");
        }
        require(sequence.joint2_window_ready(),
                "angle-span diagnostics remain available after a full 250 ms window");
        require(sequence.joint2_angle_span() > std::numbers::pi / 180.0,
                "test motion must exceed the former one-degree threshold");
        input.joint2_position_error = 0.2;
        near(sequence.update(input), 0.0,
             "joint2 target-error instability must still withdraw small-yaw torque");
        require(sequence.faulted()
                && std::string_view{sequence.fault_reason()} == "joint2_unstable",
                "joint2 target error remains part of its stability condition");
    }

    SmallYawIdentification sequence{enabled_parameters()};
    auto input = valid_input();
    input.both_down = true;
    sequence.update(input);
    input.both_down = false;
    for (int i = 0; i < 1300; ++i) {
        const double phase = (i - 100) * input.dt * std::numbers::pi;
        input.joint2_angle = 0.025 * std::sin(phase);
        input.joint2_position_error = -input.joint2_angle;
        sequence.update(input);
    }
    require(sequence.permitted() && sequence.active()
            && std::string_view{sequence.state()} == "settling",
            "a large diagnostic angle span must not block stable joint2 auto-start");
    require(sequence.joint2_window_ready()
            && sequence.joint2_angle_span() > std::numbers::pi / 180.0,
            "startup must proceed while the diagnostic span exceeds one degree");
    input.both_down = true;
    sequence.update(input);
    require(!sequence.joint2_window_ready(), "double-down must clear diagnostic posture history");
    near(sequence.joint2_stable_elapsed_s(), 0.0, "double-down must clear posture dwell");
}
} // namespace

int main() {
    try {
        waveform_is_balanced_and_runs_once();
        breakaway_staircase_has_five_groups_with_randomized_tail();
        arming_requires_fresh_double_down_and_joint2_stability();
        every_running_interlock_withdraws_torque_and_latches();
        output_velocity_below_one_rad_s_passes_and_above_one_stops();
        estimator_handles_wrap_and_catches_motion_without_native_speed();
        origin_is_captured_on_trigger_and_double_down_stops();
        joint2_instability_is_the_only_posture_motion_interlock();
        parameter_limits_and_disabled_mode();
        joint2_angle_diagnostic_window_does_not_gate_stability();
        joint2_angle_span_is_diagnostic_only();
        std::cout << "small_yaw_identification_test: 11 checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "small_yaw_identification_test: " << error.what() << '\n';
        return 1;
    }
}
