#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <stdexcept>

namespace rmcs_core::controller::gimbal {

struct BigYawIdentificationParameters {
    double command_sign = 1.0;
    double torque_limit_nm = 6.0;
    std::array<double, 4> pulse_torque_amplitudes_nm{2.0, 3.5, 5.0, 6.0};
    double pulse_duration_s = 0.20;
    double rest_duration_s = 0.70;
    double settle_duration_s = 1.0;
    double sine_torque_amplitude_nm = 6.0;
    double sine_frequency_hz = 0.15;
    double sine_frequency_end_hz = 0.60;
    double sine_duration_s = 16.0;
    double constant_torque_nm = 4.5;
    double constant_duration_s = 2.0;
    double constant_rest_duration_s = 0.5;
    double linear_ramp_peak_nm = 5.0;
    double linear_ramp_duration_s = 4.0;
    int linear_ramp_group_count = 2;
};

struct BigYawIdentificationSample {
    double torque_nm = 0.0;
    const char* stage = "settling";
    int pass = 1;
    bool complete = false;
};

// Pure time-to-torque mapping. Arming, feedback freshness and CAN output remain
// the responsibility of the existing controller and hardware command path.
// One pass matches the small-yaw pulse/chirp/constant/linear sequence. The
// second pass repeats every sample with the opposite sign, including settling.
class BigYawIdentificationWaveform {
public:
    explicit BigYawIdentificationWaveform(BigYawIdentificationParameters parameters)
        : parameters_(parameters) {
        validate();
        pass_duration_s_ = parameters_.settle_duration_s
            + kPulseCount * (parameters_.pulse_duration_s + parameters_.rest_duration_s)
            + parameters_.sine_duration_s
            + 2.0 * parameters_.constant_duration_s + parameters_.constant_rest_duration_s
            + 4.0 * parameters_.linear_ramp_group_count * parameters_.linear_ramp_duration_s;
        if (!std::isfinite(duration_s()))
            throw std::invalid_argument("big yaw identification duration must be finite");
    }

    double pass_duration_s() const noexcept { return pass_duration_s_; }
    double duration_s() const noexcept { return 2.0 * pass_duration_s_; }

    BigYawIdentificationSample sample(double elapsed_s) const noexcept {
        if (!std::isfinite(elapsed_s) || elapsed_s < 0.0)
            return {0.0, "invalid_time", 0, false};
        if (elapsed_s + kTimeEpsilon >= duration_s())
            return {0.0, "identification_complete", 2, true};

        const int pass = elapsed_s + kTimeEpsilon >= pass_duration_s_ ? 2 : 1;
        const double local_time = std::max(
            0.0, elapsed_s - (pass - 1) * pass_duration_s_);
        auto result = sample_pass(local_time + kTimeEpsilon);
        result.torque_nm *= parameters_.command_sign * (pass == 1 ? 1.0 : -1.0);
        result.pass = pass;
        return result;
    }

private:
    static constexpr std::array<int, 4> kPulseSigns{1, -1, -1, 1};
    static constexpr std::size_t kPulseCount = 4 * kPulseSigns.size();
    static constexpr double kTimeEpsilon = 1e-10;

    double bounded_amplitude(double amplitude) const noexcept {
        return std::min(amplitude, parameters_.torque_limit_nm);
    }

    BigYawIdentificationSample sample_pass(double local_time) const noexcept {
        double time = local_time - parameters_.settle_duration_s;
        if (time < 0.0)
            return {0.0, "settling"};

        const double pulse_period = parameters_.pulse_duration_s + parameters_.rest_duration_s;
        const double pulses_duration = kPulseCount * pulse_period;
        if (time < pulses_duration) {
            const auto index = static_cast<std::size_t>(time / pulse_period);
            const double phase = time - index * pulse_period;
            if (phase >= parameters_.pulse_duration_s)
                return {0.0, "rest"};
            return {kPulseSigns[index % kPulseSigns.size()]
                        * bounded_amplitude(parameters_.pulse_torque_amplitudes_nm[
                            index / kPulseSigns.size()]),
                    "pulse"};
        }
        time -= pulses_duration;
        if (time < parameters_.sine_duration_s) {
            const double phase = 2.0 * std::numbers::pi
                * (parameters_.sine_frequency_hz * time
                   + 0.5 * (parameters_.sine_frequency_end_hz - parameters_.sine_frequency_hz)
                       * time * time / parameters_.sine_duration_s);
            return {bounded_amplitude(parameters_.sine_torque_amplitude_nm) * std::sin(phase),
                    "sine"};
        }
        time -= parameters_.sine_duration_s;
        if (time < parameters_.constant_duration_s)
            return {bounded_amplitude(parameters_.constant_torque_nm), "constant_positive"};
        time -= parameters_.constant_duration_s;
        if (time < parameters_.constant_rest_duration_s)
            return {0.0, "rest"};
        time -= parameters_.constant_rest_duration_s;
        if (time < parameters_.constant_duration_s)
            return {-bounded_amplitude(parameters_.constant_torque_nm), "constant_negative"};
        time -= parameters_.constant_duration_s;

        const double leg = parameters_.linear_ramp_duration_s;
        const double group_duration = 4.0 * leg;
        if (time >= parameters_.linear_ramp_group_count * group_duration)
            return {0.0, "settling"};
        const auto group = static_cast<int>(time / group_duration);
        time -= group * group_duration;
        const double peak = bounded_amplitude(parameters_.linear_ramp_peak_nm);
        if (time < leg)
            return {peak * time / leg, "ramp_up"};
        if (time < 2.0 * leg)
            return {peak * (2.0 - time / leg), "ramp_down"};
        if (time < 3.0 * leg)
            return {-peak * (time / leg - 2.0), "ramp_reverse_up"};
        return {-peak * (4.0 - time / leg), "ramp_reverse_down"};
    }

    void validate() const {
        const auto positive = [](double value) {
            return std::isfinite(value) && value > 0.0;
        };
        if ((parameters_.command_sign != 1.0 && parameters_.command_sign != -1.0)
            || !positive(parameters_.torque_limit_nm) || parameters_.torque_limit_nm > 10.0
            || !positive(parameters_.pulse_duration_s)
            || !positive(parameters_.rest_duration_s)
            || !positive(parameters_.settle_duration_s)
            || !positive(parameters_.sine_torque_amplitude_nm)
            || !positive(parameters_.sine_frequency_hz)
            || !positive(parameters_.sine_frequency_end_hz)
            || !positive(parameters_.sine_duration_s)
            || !positive(parameters_.constant_torque_nm)
            || !positive(parameters_.constant_duration_s)
            || !positive(parameters_.constant_rest_duration_s)
            || !positive(parameters_.linear_ramp_peak_nm)
            || !positive(parameters_.linear_ramp_duration_s)
            || parameters_.linear_ramp_group_count <= 0
            || !std::all_of(parameters_.pulse_torque_amplitudes_nm.begin(),
                            parameters_.pulse_torque_amplitudes_nm.end(), positive))
            throw std::invalid_argument("invalid big yaw identification waveform parameters");
    }

    BigYawIdentificationParameters parameters_;
    double pass_duration_s_ = 0.0;
};

} // namespace rmcs_core::controller::gimbal
