#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <stdexcept>

namespace rmcs_core::hardware {

struct SmallYawIdentificationParameters {
    bool enabled = false;
    double command_sign = 1.0;
    double torque_limit_nm = 6.0;
    double pulse_duration_s = 0.20;
    double rest_duration_s = 0.7;
    double settle_duration_s = 1.0;
    // A continuous sinusoidal segment follows the balanced step sequence.
    // It excites the low-speed friction/drive response without changing the
    // safety interlocks or the torque ceiling.
    double sine_torque_amplitude_nm = 6.0;
    // The sine segment is a linear chirp.  Starting below the old 0.25 Hz
    // tone exposes friction, while the upper band improves inertia
    // observability while using the 6 Nm identification ceiling.
    double sine_frequency_hz = 0.15;
    double sine_frequency_end_hz = 0.60;
    double sine_duration_s = 16.0;
    // After the sine, use a symmetric constant-torque pair to expose static
    // friction and bias, then positive and negative linear cycles for dynamics.
    double constant_torque_nm = 1.0;
    double constant_duration_s = 2.0;
    double constant_rest_duration_s = 0.5;
    double linear_ramp_peak_nm = 4.5;
    // Each linear leg lasts this long. A group contains the two directional
    // cycles 0 -> +peak -> 0 and 0 -> -peak -> 0.
    double linear_ramp_duration_s = 6.0;
    // Keep one group as the backwards-compatible default; the identification
    // profile requests two groups for repeated linear excitation.
    int linear_ramp_group_count = 1;
    // Optional breakaway staircase: the first three positive/negative groups
    // use ascending levels; the following groups use fixed, different
    // permutations of the same levels to reduce sequence-order bias.
    bool breakaway_staircase_enabled = false;
    double breakaway_pulse_duration_s = 0.5;
    double breakaway_rest_duration_s = 2.0;
    int breakaway_group_count = 5;
    double max_velocity_rad_s = 1.0;
    double joint2_position_tolerance_rad = 10.0 * std::numbers::pi / 180.0;
    // The measured angle window is published for diagnosis; stability uses
    // only target position error.
    double joint2_stability_window_s = 0.25;
    double joint2_stable_duration_s = 1.0;
};

struct SmallYawIdentificationInput {
    double angle = 0.0;
    double joint2_position_error = 0.0;
    double joint2_angle = 0.0;
    bool remote_valid = false;
    bool both_down = true;
    bool feedback_valid = false;
    bool joint2_permitted = false;
    double dt = 0.001;
};

// This class owns the identification request, not the CAN sender. The caller
// must make identification, position holding and legacy braking mutually
// exclusive, and repeat the hardware freshness/zero-output interlocks.
//
// A confirmed DOWN/DOWN is followed by joint2 stability and automatic start.
// Both completion and faults latch; a fresh valid
// DOWN/DOWN is the only way to rearm. No excitation torque is produced during
// the baseline settling interval.
class SmallYawIdentification {
public:
    explicit SmallYawIdentification(SmallYawIdentificationParameters parameters)
        : parameters_(parameters) {
        if (!valid_parameters(parameters_))
            throw std::invalid_argument(
                "invalid small yaw identification parameters");
    }

    double update(const SmallYawIdentificationInput& input) noexcept {
        permitted_ = active_ = false;
        if (!parameters_.enabled) {
            reset();
            state_ = "disabled";
            return 0.0;
        }
        if (!input.remote_valid)
            return trip("remote_invalid");
        if (!std::isfinite(input.dt) || input.dt <= 0.0 || input.dt > 0.02)
            return trip("update_gap");
        if (!input.feedback_valid || !std::isfinite(input.angle))
            return trip("feedback_invalid");
        if (input.both_down) {
            reset();
            double_down_confirmed_ = true;
            seed_motion(input.angle);
            state_ = "double_down_protection";
            return 0.0;
        }
        if (faulted_) {
            state_ = "fault";
            return 0.0;
        }
        if (complete_) {
            state_ = "identification_complete";
            return 0.0;
        }
        if (!double_down_confirmed_) {
            state_ = "waiting_for_double_down";
            return 0.0;
        }

        update_motion(input.angle, input.dt);
        if (origin_valid_)
            relative_angle_ = position_ - origin_;
        const bool joint2_inputs_valid = input.joint2_permitted
            && std::isfinite(input.joint2_position_error)
            && std::isfinite(input.joint2_angle);
        if (joint2_inputs_valid)
            update_joint2_posture(input.joint2_angle, input.dt);
        else
            reset_joint2_posture();
        const bool joint2_stable = joint2_inputs_valid
            && std::abs(input.joint2_position_error)
                <= parameters_.joint2_position_tolerance_rad;

        if (running_) {
            if (!input.joint2_permitted)
                return trip("joint2_not_permitted");
            if (!joint2_stable)
                return trip("joint2_unstable");
            if (std::abs(estimated_velocity_)
                > parameters_.max_velocity_rad_s + kTimeEpsilon)
                return trip("velocity_limit");

            elapsed_s_ += input.dt;
            if (elapsed_s_ + kTimeEpsilon >= total_duration_s()) {
                complete_ = true;
                running_ = false;
                state_ = "identification_complete";
                return 0.0;
            }
            permitted_ = active_ = true;
            return waveform();
        }

        if (!joint2_stable) {
            stable_time_s_ = 0.0;
            state_ = "waiting_for_joint2_stability";
            return 0.0;
        }
        stable_time_s_ = std::min(
            parameters_.joint2_stable_duration_s, stable_time_s_ + input.dt);
        if (stable_time_s_ + kTimeEpsilon < parameters_.joint2_stable_duration_s) {
            state_ = "waiting_for_joint2_stability";
            return 0.0;
        }
        if (std::abs(estimated_velocity_) > parameters_.max_velocity_rad_s + kTimeEpsilon) {
            state_ = "waiting_for_small_yaw_stability";
            return 0.0;
        }

        permitted_ = running_ = origin_valid_ = active_ = true;
        origin_ = position_;
        relative_angle_ = elapsed_s_ = 0.0;
        state_ = "settling";
        return 0.0;
    }

    const char* state() const noexcept { return state_; }
    const char* fault_reason() const noexcept { return fault_reason_; }
    double elapsed_s() const noexcept { return elapsed_s_; }
    double relative_angle() const noexcept { return relative_angle_; }
    double estimated_velocity() const noexcept { return estimated_velocity_; }
    double joint2_angle_span() const noexcept { return joint2_angle_span_; }
    double joint2_stable_elapsed_s() const noexcept { return stable_time_s_; }
    bool joint2_window_ready() const noexcept { return joint2_window_ready_; }
    bool active() const noexcept { return active_; }
    bool permitted() const noexcept { return permitted_; }
    bool complete() const noexcept { return complete_; }
    bool faulted() const noexcept { return faulted_; }
    bool origin_valid() const noexcept { return origin_valid_; }
    double total_duration_s() const noexcept {
        if (parameters_.breakaway_staircase_enabled)
            return parameters_.settle_duration_s
                 + static_cast<double>(parameters_.breakaway_group_count)
                     * 2.0 * static_cast<double>(kBreakawayLevelCount)
                     * (parameters_.breakaway_pulse_duration_s
                        + parameters_.breakaway_rest_duration_s);
        return parameters_.settle_duration_s
             + kPulseCount * (parameters_.pulse_duration_s + parameters_.rest_duration_s)
             + parameters_.sine_duration_s
             + 2.0 * parameters_.constant_duration_s
             + parameters_.constant_rest_duration_s
             + 4.0 * static_cast<double>(parameters_.linear_ramp_group_count)
                 * parameters_.linear_ramp_duration_s;
    }

private:
    struct Sample {
        double time = 0.0;
        double position = 0.0;
    };
    static constexpr std::size_t kPulseCount = 16;
    static constexpr std::size_t kBreakawayLevelCount = 8;
    struct BreakawayPulse {
        int sign;
        std::size_t level_index;
    };
    // The final two groups are deterministic permutations, so the run is
    // repeatable while avoiding a fixed ascending-level order. Each contains
    // every level once in each sign, with positive/negative pulses interleaved.
    static constexpr std::array<BreakawayPulse, 2 * kBreakawayLevelCount>
        kBreakawayRandomOrder0{
            BreakawayPulse{1, 4}, BreakawayPulse{-1, 0}, BreakawayPulse{1, 6}, BreakawayPulse{-1, 1},
            BreakawayPulse{1, 7}, BreakawayPulse{-1, 2}, BreakawayPulse{1, 5}, BreakawayPulse{-1, 3},
            BreakawayPulse{1, 0}, BreakawayPulse{-1, 6}, BreakawayPulse{1, 2}, BreakawayPulse{-1, 7},
            BreakawayPulse{1, 3}, BreakawayPulse{-1, 4}, BreakawayPulse{1, 1}, BreakawayPulse{-1, 5}};
    static constexpr std::array<BreakawayPulse, 2 * kBreakawayLevelCount>
        kBreakawayRandomOrder1{
            BreakawayPulse{-1, 5}, BreakawayPulse{1, 2}, BreakawayPulse{-1, 0}, BreakawayPulse{1, 7},
            BreakawayPulse{-1, 3}, BreakawayPulse{1, 6}, BreakawayPulse{-1, 1}, BreakawayPulse{1, 4},
            BreakawayPulse{-1, 7}, BreakawayPulse{1, 0}, BreakawayPulse{-1, 4}, BreakawayPulse{1, 1},
            BreakawayPulse{-1, 6}, BreakawayPulse{1, 3}, BreakawayPulse{-1, 2}, BreakawayPulse{1, 5}};
    static constexpr std::size_t kSampleCapacity = 64;
    static constexpr std::size_t kPostureSampleCapacity = 1024;
    static constexpr double kFullTurn = 2.0 * std::numbers::pi;
    static constexpr double kVelocityWindowS = 0.01;
    static constexpr double kSamplePeriodS = 0.001;
    static constexpr double kTimeEpsilon = 1e-10;

    static bool valid_parameters(const SmallYawIdentificationParameters& p) noexcept {
        const auto in_range = [](double value, double low, double high) {
            return std::isfinite(value) && value >= low && value <= high;
        };
        return (p.command_sign == 1.0 || p.command_sign == -1.0)
            && in_range(p.torque_limit_nm, 0.001, 7.0)
            && in_range(p.pulse_duration_s, 0.01, 0.3)
            && in_range(p.rest_duration_s, 0.1, 3.0)
            && in_range(p.settle_duration_s, 0.1, 5.0)
            && in_range(p.sine_torque_amplitude_nm, 0.001, 6.0)
            && in_range(p.sine_frequency_hz, 0.01, 2.0)
            && in_range(p.sine_frequency_end_hz, 0.01, 2.0)
            && in_range(p.sine_duration_s, 0.5, 60.0)
            && in_range(p.constant_torque_nm, 0.001, 6.0)
            && in_range(p.constant_duration_s, 0.1, 20.0)
            && in_range(p.constant_rest_duration_s, 0.1, 5.0)
            && in_range(p.linear_ramp_peak_nm, 0.001, p.torque_limit_nm)
            && in_range(p.linear_ramp_duration_s, 0.5, 60.0)
            && p.linear_ramp_group_count >= 1 && p.linear_ramp_group_count <= 4
            && in_range(p.breakaway_pulse_duration_s, 0.01, 5.0)
            && in_range(p.breakaway_rest_duration_s, 0.1, 10.0)
            && p.breakaway_group_count >= 1 && p.breakaway_group_count <= 10
            && in_range(p.max_velocity_rad_s, 0.001, 5.0)
            && in_range(p.joint2_position_tolerance_rad, 0.0001, std::numbers::pi / 12.0)
            && in_range(p.joint2_stability_window_s, 0.05, 1.0)
            && in_range(p.joint2_stable_duration_s, 0.1, 10.0);
    }

    void reset() noexcept {
        double_down_confirmed_ = running_ = complete_ = faulted_ = false;
        permitted_ = active_ = origin_valid_ = have_angle_ = false;
        stable_time_s_ = elapsed_s_ = relative_angle_ = estimated_velocity_ = 0.0;
        position_ = origin_ = previous_angle_ = time_s_ = 0.0;
        sample_count_ = next_sample_ = 0;
        reset_joint2_posture();
        fault_reason_ = "none";
    }

    double trip(const char* reason) noexcept {
        if (!faulted_)
            fault_reason_ = reason;
        faulted_ = true;
        running_ = permitted_ = active_ = double_down_confirmed_ = false;
        stable_time_s_ = 0.0;
        state_ = "fault";
        return 0.0;
    }

    double waveform() noexcept {
        const double after_settle = elapsed_s_ + kTimeEpsilon - parameters_.settle_duration_s;
        if (after_settle < 0.0) {
            state_ = "settling";
            return 0.0;
        }
        if (parameters_.breakaway_staircase_enabled)
            return breakaway_waveform(after_settle);
        const double period = parameters_.pulse_duration_s + parameters_.rest_duration_s;
        const auto pulse_index = static_cast<std::size_t>(after_settle / period);
        if (pulse_index < kPulseCount) {
            const double phase_time = after_settle - pulse_index * period;
            if (phase_time >= parameters_.pulse_duration_s) {
                state_ = "rest";
                return 0.0;
            }
            constexpr std::array<double, 4> amplitudes{2.0, 3.5, 5.0, 6.0};
            constexpr std::array<int, 4> signs{1, -1, -1, 1};
            state_ = "pulse";
            return signs[pulse_index % signs.size()]
                 * std::min(amplitudes[pulse_index / signs.size()], parameters_.torque_limit_nm);
        }
        const double sine_elapsed = after_settle - kPulseCount * period;
        if (sine_elapsed < parameters_.sine_duration_s) {
            state_ = "sine";
            const double amplitude = std::min(
                parameters_.sine_torque_amplitude_nm, parameters_.torque_limit_nm);
            const double duration = parameters_.sine_duration_s;
            const double frequency_delta = parameters_.sine_frequency_end_hz
                - parameters_.sine_frequency_hz;
            const double phase = 2.0 * std::numbers::pi * (
                parameters_.sine_frequency_hz * sine_elapsed
                + 0.5 * frequency_delta * sine_elapsed * sine_elapsed / duration);
            return amplitude * std::sin(phase);
        }
        const double after_sine = sine_elapsed - parameters_.sine_duration_s;
        const double constant_pair_duration =
            2.0 * parameters_.constant_duration_s + parameters_.constant_rest_duration_s;
        if (after_sine < constant_pair_duration) {
            if (after_sine < parameters_.constant_duration_s) {
                state_ = "constant_positive";
                return std::min(parameters_.constant_torque_nm, parameters_.torque_limit_nm);
            }
            if (after_sine < parameters_.constant_duration_s
                + parameters_.constant_rest_duration_s) {
                state_ = "rest";
                return 0.0;
            }
            state_ = "constant_negative";
            return -std::min(parameters_.constant_torque_nm, parameters_.torque_limit_nm);
        }
        const double ramp_elapsed = after_sine - constant_pair_duration;
        const double ramp_duration = parameters_.linear_ramp_duration_s;
        const double limit = std::min(
            parameters_.linear_ramp_peak_nm, parameters_.torque_limit_nm);
        const double group_duration = 4.0 * ramp_duration;
        const auto group = static_cast<int>(ramp_elapsed / group_duration);
        if (group >= parameters_.linear_ramp_group_count)
            return 0.0;
        const double group_elapsed = ramp_elapsed - group * group_duration;
        if (group_elapsed < ramp_duration) {
            state_ = "ramp_up";
            return limit * group_elapsed / ramp_duration;
        }
        if (group_elapsed < 2.0 * ramp_duration) {
            state_ = "ramp_down";
            return limit * (1.0 - (group_elapsed - ramp_duration) / ramp_duration);
        }
        if (group_elapsed < 3.0 * ramp_duration) {
            state_ = "ramp_reverse_up";
            return -limit * (group_elapsed - 2.0 * ramp_duration) / ramp_duration;
        }
        if (group_elapsed < 4.0 * ramp_duration) {
            state_ = "ramp_reverse_down";
            return -limit * (1.0 - (group_elapsed - 3.0 * ramp_duration) / ramp_duration);
        }
        return 0.0;
    }

    double breakaway_waveform(double after_settle) noexcept {
        constexpr std::array<double, kBreakawayLevelCount> levels{
            3.0, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0};
        const double period = parameters_.breakaway_pulse_duration_s
                            + parameters_.breakaway_rest_duration_s;
        const auto pulse_index = static_cast<std::size_t>(after_settle / period);
        const auto pulses_per_group = 2 * kBreakawayLevelCount;
        const auto total_pulses = static_cast<std::size_t>(parameters_.breakaway_group_count)
                                * pulses_per_group;
        if (pulse_index >= total_pulses) {
            state_ = "breakaway_rest";
            return 0.0;
        }
        const double phase_time = after_settle - pulse_index * period;
        const auto group = pulse_index / pulses_per_group;
        const auto within_group = pulse_index % pulses_per_group;
        int sign = 1;
        std::size_t level_index = within_group;
        if (group < 3) {
            if (within_group >= kBreakawayLevelCount) {
                sign = -1;
                level_index = within_group - kBreakawayLevelCount;
            }
        } else {
            const auto& order = (group - 3) % 2 == 0
                ? kBreakawayRandomOrder0 : kBreakawayRandomOrder1;
            const auto pulse = order[within_group];
            sign = pulse.sign;
            level_index = pulse.level_index;
        }
        if (phase_time >= parameters_.breakaway_pulse_duration_s) {
            state_ = "breakaway_rest";
            return 0.0;
        }
        state_ = sign > 0 ? "breakaway_positive" : "breakaway_negative";
        return static_cast<double>(sign) * levels[level_index];
    }

    void reset_joint2_posture() noexcept {
        posture_begin_ = posture_count_ = 0;
        posture_time_s_ = joint2_angle_span_ = 0.0;
        joint2_window_ready_ = false;
    }

    // Measure actual posture excursion, rather than requiring every noisy
    // 20 ms derivative sample to be below a standstill threshold. Preserve a
    // full window across waiting, auto-start and running states.
    // Retaining the point bracketing the window is deliberately conservative.
    void update_joint2_posture(double angle, double dt) noexcept {
        posture_time_s_ += dt;
        const auto point = [this](std::size_t offset) -> const Sample& {
            return posture_samples_[(posture_begin_ + offset) % kPostureSampleCapacity];
        };
        if (posture_count_ == 0
            || posture_time_s_ - point(posture_count_ - 1).time + kTimeEpsilon >= kSamplePeriodS) {
            if (posture_count_ == kPostureSampleCapacity) {
                posture_begin_ = (posture_begin_ + 1) % kPostureSampleCapacity;
                --posture_count_;
            }
            posture_samples_[(posture_begin_ + posture_count_) % kPostureSampleCapacity]
                = {posture_time_s_, angle};
            ++posture_count_;
        }
        while (posture_count_ > 2
            && posture_time_s_ - point(1).time + kTimeEpsilon
                >= parameters_.joint2_stability_window_s) {
            posture_begin_ = (posture_begin_ + 1) % kPostureSampleCapacity;
            --posture_count_;
        }
        double minimum = angle, maximum = angle;
        for (std::size_t i = 0; i < posture_count_; ++i) {
            minimum = std::min(minimum, point(i).position);
            maximum = std::max(maximum, point(i).position);
        }
        joint2_angle_span_ = maximum - minimum;
        joint2_window_ready_ = posture_time_s_ - point(0).time + kTimeEpsilon
            >= parameters_.joint2_stability_window_s;
    }

    void append_sample(Sample sample) noexcept {
        samples_[next_sample_] = sample;
        next_sample_ = (next_sample_ + 1) % kSampleCapacity;
        sample_count_ = std::min(sample_count_ + 1, kSampleCapacity);
    }

    void seed_motion(double angle) noexcept {
        have_angle_ = true;
        previous_angle_ = std::remainder(angle, kFullTurn);
        time_s_ = position_ = estimated_velocity_ = 0.0;
        next_sample_ = sample_count_ = 0;
        append_sample({0.0, 0.0});
    }

    // An unbroken 10 ms angle window, with wrap-aware incremental position.
    // Unlike the drift/brake estimator, this never resets on a direction change.
    // Before a full window exists, the available interval is used rather than
    // leaving an unprotected 10 ms startup period.
    void update_motion(double angle, double dt) noexcept {
        if (!have_angle_) {
            seed_motion(angle);
            return;
        }
        const double wrapped = std::remainder(angle, kFullTurn);
        position_ += std::remainder(wrapped - previous_angle_, kFullTurn);
        previous_angle_ = wrapped;
        time_s_ += dt;
        const auto newest_index = (next_sample_ + kSampleCapacity - 1) % kSampleCapacity;
        if (time_s_ - samples_[newest_index].time + kTimeEpsilon >= kSamplePeriodS)
            append_sample({time_s_, position_});

        const auto oldest_index = (next_sample_ + kSampleCapacity - sample_count_) % kSampleCapacity;
        const double target_time = std::max(samples_[oldest_index].time, time_s_ - kVelocityWindowS);
        Sample before = samples_[oldest_index];
        double old_position = before.position;
        for (std::size_t offset = 1; offset < sample_count_; ++offset) {
            const auto& after = samples_[(oldest_index + offset) % kSampleCapacity];
            if (after.time >= target_time) {
                const double fraction = std::clamp(
                    (target_time - before.time) / (after.time - before.time), 0.0, 1.0);
                old_position = before.position + fraction * (after.position - before.position);
                break;
            }
            before = after;
            old_position = before.position;
        }
        const double span = time_s_ - target_time;
        estimated_velocity_ = span > 0.0 ? (position_ - old_position) / span : 0.0;
    }

    SmallYawIdentificationParameters parameters_;
    std::array<Sample, kSampleCapacity> samples_{};
    std::array<Sample, kPostureSampleCapacity> posture_samples_{};
    std::size_t posture_begin_ = 0, posture_count_ = 0;
    double posture_time_s_ = 0.0, joint2_angle_span_ = 0.0;
    bool joint2_window_ready_ = false;
    std::size_t next_sample_ = 0, sample_count_ = 0;
    bool double_down_confirmed_ = false;
    bool running_ = false, complete_ = false, faulted_ = false;
    bool permitted_ = false, active_ = false, origin_valid_ = false, have_angle_ = false;
    double stable_time_s_ = 0.0, elapsed_s_ = 0.0, relative_angle_ = 0.0;
    double estimated_velocity_ = 0.0, previous_angle_ = 0.0, position_ = 0.0;
    double origin_ = 0.0, time_s_ = 0.0;
    const char* state_ = "waiting_for_double_down";
    const char* fault_reason_ = "none";
};

} // namespace rmcs_core::hardware
