#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>

#include "controller/gimbal/big_yaw_identification_waveform.hpp"

namespace {

using rmcs_core::controller::gimbal::BigYawIdentificationParameters;
using rmcs_core::controller::gimbal::BigYawIdentificationWaveform;

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

void near(double actual, double expected, const char* message, double tolerance = 1e-8) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
        throw std::runtime_error(std::string{message} + ": actual=" + std::to_string(actual)
                                 + ", expected=" + std::to_string(expected));
}

void duration_and_completion() {
    const BigYawIdentificationWaveform waveform{BigYawIdentificationParameters{}};
    near(waveform.pass_duration_s(), 67.9, "one pass must retain the small-yaw duration");
    near(waveform.duration_s(), 135.8, "both mirrored passes must fit in the advertised duration");
    require(waveform.sample(0.0).stage == std::string_view{"settling"},
            "each recording must begin with zero-torque settling");
    const auto second = waveform.sample(67.9);
    require(second.pass == 2 && !second.complete && second.torque_nm == 0.0,
            "the mirrored pass must begin with a zero interval");
    for (const double time : {135.8, 136.0, 1000.0}) {
        const auto sample = waveform.sample(time);
        require(sample.complete && sample.torque_nm == 0.0 && sample.pass == 2,
                "completion must remove excitation and never restart the waveform");
    }
}

void pulse_levels_and_order() {
    const BigYawIdentificationWaveform waveform{BigYawIdentificationParameters{}};
    constexpr std::array levels{2.0, 3.5, 5.0, 6.0};
    constexpr std::array signs{1, -1, -1, 1};
    for (std::size_t index = 0; index < 16; ++index) {
        const double start = 1.0 + index * 0.9;
        const auto pulse = waveform.sample(start + 0.1);
        near(pulse.torque_nm, levels[index / 4] * signs[index % 4],
             "the balanced pulse order must preserve all four small-yaw torque levels");
        require(pulse.stage == std::string_view{"pulse"}, "active pulse must be labelled");
        near(waveform.sample(start + 0.2).torque_nm, 0.0,
             "a pulse must end after 0.2 seconds");
        near(waveform.sample(start + 0.85).torque_nm, 0.0,
             "the remaining pulse period must have zero torque");
    }
}

void chirp_constant_and_linear_shapes() {
    const BigYawIdentificationWaveform waveform{BigYawIdentificationParameters{}};
    constexpr double sine_start = 15.4;
    for (const double time : {0.0, 0.25, 1.0, 5.25, 15.9}) {
        const auto sample = waveform.sample(sine_start + time);
        const double phase = 2.0 * std::numbers::pi * (0.15 * time + 0.5 * 0.45 * time * time / 16.0);
        near(sample.torque_nm, 6.0 * std::sin(phase),
             "chirp must retain the small-yaw frequency sweep", 1e-7);
        require(sample.stage == std::string_view{"sine"}, "chirp must follow the final pulse rest");
    }
    near(waveform.sample(31.4).torque_nm, 4.5, "positive constant hold must follow the chirp");
    near(waveform.sample(33.399).torque_nm, 4.5, "constant torque must last the full two seconds");
    near(waveform.sample(33.4).torque_nm, 0.0, "constant pair must have a zero-torque gap");
    near(waveform.sample(33.9).torque_nm, -4.5, "negative constant hold must follow the gap");
    near(waveform.sample(35.899).torque_nm, -4.5, "negative hold must last the full two seconds");

    constexpr double ramp_start = 35.9;
    for (const int group : {0, 1}) {
        const double start = ramp_start + group * 16.0;
        constexpr std::array expected{0.0, 2.5, 5.0, 2.5, 0.0, -2.5, -5.0, -2.5};
        for (std::size_t point = 0; point < expected.size(); ++point)
            near(waveform.sample(start + 2.0 * point).torque_nm, expected[point],
                 "each linear group must make a complete positive and negative triangle");
        for (const double knot : {4.0, 8.0, 12.0})
            near(waveform.sample(start + knot - 1e-6).torque_nm,
                 waveform.sample(start + knot + 1e-6).torque_nm,
                 "linear torque must be continuous at each peak and zero crossing", 3e-6);
    }
}

void entire_second_pass_is_mirrored_and_bounded() {
    const BigYawIdentificationWaveform positive{BigYawIdentificationParameters{}};
    auto negative_parameters = BigYawIdentificationParameters{};
    negative_parameters.command_sign = -1.0;
    const BigYawIdentificationWaveform negative{negative_parameters};
    double signed_integral = 0.0;
    constexpr double step = 0.001;
    for (int tick = 0; tick < 67'900; ++tick) {
        const double time = (tick + 0.25) * step;
        const auto first = positive.sample(time);
        const auto second = positive.sample(time + positive.pass_duration_s());
        require(first.pass == 1 && second.pass == 2 && !first.complete && !second.complete,
                "both passes must remain active until their final sample");
        require(std::string_view{first.stage} == second.stage,
                "mirroring must preserve every waveform stage and duration");
        near(second.torque_nm, -first.torque_nm,
             "the complete second pass must reverse every torque sample", 1e-7);
        near(negative.sample(time).torque_nm, -first.torque_nm,
             "command sign must reverse the entire sequence");
        require(std::abs(first.torque_nm) <= 6.0 && std::abs(second.torque_nm) <= 6.0,
                "all waveform families must respect the configured torque cap");
        signed_integral += (first.torque_nm + second.torque_nm) * step;
    }
    near(signed_integral, 0.0, "both passes must have zero combined signed torque area", 1e-7);

    auto clipped_parameters = BigYawIdentificationParameters{};
    clipped_parameters.torque_limit_nm = 3.0;
    const BigYawIdentificationWaveform clipped{clipped_parameters};
    for (double time = 0.0; time < clipped.duration_s(); time += 0.013)
        require(std::abs(clipped.sample(time).torque_nm) <= 3.0,
                "lower torque caps must also constrain pulses, sine, holds and ramps");
}

void invalid_configuration_and_times() {
    using Parameters = BigYawIdentificationParameters;
    constexpr std::array<double Parameters::*, 14> fields{
        &Parameters::torque_limit_nm, &Parameters::pulse_duration_s,
        &Parameters::rest_duration_s, &Parameters::settle_duration_s,
        &Parameters::sine_torque_amplitude_nm, &Parameters::sine_frequency_hz,
        &Parameters::sine_frequency_end_hz, &Parameters::sine_duration_s,
        &Parameters::constant_torque_nm, &Parameters::constant_duration_s,
        &Parameters::constant_rest_duration_s, &Parameters::linear_ramp_peak_nm,
        &Parameters::linear_ramp_duration_s, &Parameters::command_sign};
    constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    constexpr double infinity = std::numeric_limits<double>::infinity();
    for (const auto field : fields) {
        for (const double invalid : {0.0, nan, infinity}) {
            auto parameters = Parameters{};
            parameters.*field = invalid;
            bool rejected = false;
            try {
                const BigYawIdentificationWaveform waveform{parameters};
                (void)waveform;
            } catch (const std::invalid_argument&) {
                rejected = true;
            }
            require(rejected, "invalid waveform parameters must be rejected before operation");
        }
    }
    for (const int count : {0, -1}) {
        auto parameters = Parameters{};
        parameters.linear_ramp_group_count = count;
        bool rejected = false;
        try {
            const BigYawIdentificationWaveform waveform{parameters};
            (void)waveform;
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        require(rejected, "linear excitation must contain a positive number of complete groups");
    }
    const BigYawIdentificationWaveform waveform{Parameters{}};
    for (const double time : {-1.0, nan, infinity}) {
        const auto sample = waveform.sample(time);
        require(sample.torque_nm == 0.0 && !sample.complete && sample.pass == 0,
                "invalid elapsed time must never request motor torque");
    }
}

} // namespace

int main() {
    duration_and_completion();
    pulse_levels_and_order();
    chirp_constant_and_linear_shapes();
    entire_second_pass_is_mirrored_and_bounded();
    invalid_configuration_and_times();
}
