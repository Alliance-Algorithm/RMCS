#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace rmcs_core::controller::identification {

struct WheelProbeConfig {
    std::array<double, 3> low_speeds{};
    std::array<double, 5> speeds{}; // output-shaft rad/s, ascending
    double ramp_s = 0, hold_s = 0, coast_s = 0, step_hold_s = 0;
    double chirp_s = 0, validation_s = 0;
    int step_repetitions = 0;
};

struct WheelProbeSample {
    std::array<double, 2> velocity_target{};
    std::array<bool, 2> servo{};
    // 1=selected wheel(s) velocity servo, 2=both enabled zero-current coast.
    // A non-selected wheel always receives zero current, never a zero-speed servo.
    std::uint8_t mode = 0, waveform = 0;
    int segment_id = -1;
    bool validation = false;
};

class WheelProbePlan {
public:
    static constexpr const char* kRevision = "wheel_multiband_v2";
    static constexpr double kReferenceHz = 50.0;

    struct Segment {
        double start_s, duration_s, from, to;
        std::array<double, 2> axis_scale;
        double amplitude = 0, f0 = 0, f1 = 0;
        std::uint8_t waveform = 0;
        bool coast = false, validation = false;
        std::string label;
    };

    explicit WheelProbePlan(const WheelProbeConfig& c) {
        validate_speeds(c.low_speeds, 1.0);
        validate_speeds(c.speeds, 30.0);
        if (c.low_speeds.back() >= c.speeds.front())
            throw std::invalid_argument("Low-speed band must precede platform speeds");
        for (double seconds :
             {c.ramp_s, c.hold_s, c.coast_s, c.step_hold_s, c.chirp_s, c.validation_s})
            if (!std::isfinite(seconds) || seconds <= 0
                || std::abs(seconds * kReferenceHz - std::round(seconds * kReferenceHz)) > 1e-8)
                throw std::invalid_argument("Wheel durations must align to 50 Hz targets");
        if (c.ramp_s < .2 || c.hold_s < 1 || c.coast_s < 2 || c.step_hold_s < .5 || c.chirp_s < 8
            || c.validation_s < 8 || c.step_repetitions < 2 || c.step_repetitions > 4)
            throw std::invalid_argument("Wheel sequence lacks settling or repeated excitation");
        max_target_speed_ = c.speeds.back();
        add("initial_zero_current", {0, 0}, 0, 0, c.coast_s, 0, true);
        for (const auto axes : {std::array{1.0, 0.0}, std::array{0.0, 1.0}}) {
            const std::string side = axes[0] ? "left_" : "right_";
            for (double speed : c.low_speeds)
                for (double sign : {1., -1.})
                    platform(side + "low", axes, sign * speed, c);
            for (double speed : c.speeds)
                for (double sign : {1., -1.})
                    platform(side + "platform", axes, sign * speed, c);

            // The reference is intentionally discontinuous here. Current is
            // bounded by the independently checked C620 / driver current contract.
            for (int repeat = 0; repeat < c.step_repetitions; ++repeat) {
                for (double target : {0., c.speeds[0], 0., -c.speeds[0], 0.})
                    step(side + "step_" + std::to_string(repeat), axes, target, c.step_hold_s);
                for (double sign : {1., -1.}) {
                    for (std::size_t i = 1; i < c.speeds.size(); ++i)
                        step(
                            side + "step_" + std::to_string(repeat), axes, sign * c.speeds[i],
                            c.step_hold_s);
                    for (int i = 3; i >= 1; --i)
                        step(
                            side + "step_" + std::to_string(repeat), axes, sign * c.speeds[i],
                            c.step_hold_s);
                    step(side + "step_" + std::to_string(repeat), axes, 0, c.step_hold_s);
                }
            }

            for (double speed : {c.speeds[1], c.speeds[2], c.speeds[3]})
                for (double sign : {1., -1.}) {
                    add(side + "coast_runup", axes, 0, sign * speed, c.ramp_s, 1);
                    add(side + "coast_plateau", axes, sign * speed, sign * speed, c.hold_s);
                    // No braking segment may be inserted before this release.
                    add(side + "zero_current_release", axes, 0, 0, c.coast_s, 0, true);
                    step(side + "recover_zero", axes, 0, c.ramp_s);
                }

            add(side + "triangle_entry", axes, 0, -c.speeds[0], c.ramp_s, 1);
            add(side + "triangle", axes, -c.speeds[0], c.speeds[0], 8., 4);
            add(side + "triangle_exit", axes, -c.speeds[0], 0, c.ramp_s, 1);
            for (double bias : {0., c.speeds[2], -c.speeds[2]}) {
                add(side + "chirp_entry", axes, 0, bias, c.ramp_s, 1);
                add(side + "low_chirp", axes, bias, bias, c.chirp_s, 6, false, false,
                    bias == 0 ? 3. : 5., .2, 2.);
                add(side + "high_chirp", axes, bias, bias, c.chirp_s, 6, false, false,
                    bias == 0 ? .4 : .8, 2., 8.);
                add(side + "chirp_exit", axes, bias, 0, c.ramp_s, 1);
            }
            for (double bias : {7., -7.}) {
                add(side + "validation_entry", axes, 0, bias, c.ramp_s, 1, false, true);
                add(side + "heldout_multisine", axes, bias, bias, c.validation_s, 8, false, true,
                    5.);
                add(side + "validation_exit", axes, bias, 0, c.ramp_s, 1, false, true);
            }
            add(side + "end_zero_current", axes, 0, 0, c.coast_s, 0, true);
        }
        // API signs only: physical common/differential directions must be
        // established from the actual mounting, not inferred from these names.
        for (const auto axes : {std::array{1., 1.}, std::array{1., -1.}}) {
            const std::string name = axes[1] > 0 ? "both_api_same_" : "both_api_opposite_";
            add(name + "entry", axes, 0, 5., c.ramp_s, 1, false, true);
            for (double target : {5., 10., 0., -10., 0.})
                add(name + "heldout_step", axes, target, target, c.hold_s, 3, false, true);
            add(name + "heldout_chirp", axes, 0, 0, c.chirp_s, 6, false, true, 4., .3, 3.);
            add(name + "heldout_multisine", axes, 0, 0, c.validation_s, 8, false, true, 4.);
            add(name + "end_zero_current", axes, 0, 0, c.coast_s, 0, true, true);
        }
        for (const auto& s : segments_)
            max_target_speed_ = std::max(
                max_target_speed_, std::max(std::abs(s.from), std::abs(s.to)) + s.amplitude);
        if (max_target_speed_ > 30.)
            throw std::invalid_argument("Excitation exceeds this 30 rad/s wheel experiment range");
        if (duration_s_ > 1200)
            throw std::invalid_argument("Wheel sequence exceeds twenty minutes");
    }

    [[nodiscard]] double duration() const { return duration_s_; }
    [[nodiscard]] double max_target_speed() const { return max_target_speed_; }
    [[nodiscard]] const std::vector<Segment>& segments() const { return segments_; }

    [[nodiscard]] WheelProbeSample at(double elapsed) const {
        if (!std::isfinite(elapsed) || elapsed < 0 || elapsed > duration_s_)
            throw std::out_of_range("Wheel probe time outside trajectory");
        std::size_t index = segments_.size() - 1;
        for (std::size_t i = 0; i < segments_.size(); ++i)
            if (elapsed < segments_[i].start_s + segments_[i].duration_s - 1e-9) {
                index = i;
                break;
            }
        const auto& segment = segments_[index];
        const double t = std::clamp(elapsed - segment.start_s, 0., segment.duration_s);
        const double u = t / segment.duration_s;
        double value = segment.to;
        if (segment.waveform == 1) {
            const double blend = u * u * u * (10 + u * (-15 + 6 * u));
            value = segment.from + (segment.to - segment.from) * blend;
        } else if (segment.waveform == 4) {
            // Four two-second triangles, continuous in velocity.
            const double cycle = std::fmod(t, 2.) / 2.;
            const double triangle = 1. - std::abs(2. * cycle - 1.);
            value = segment.from + (segment.to - segment.from) * triangle;
        } else if (segment.waveform == 6 || segment.waveform == 8) {
            // Smooth the first/last 0.5 s to meet the surrounding bias exactly.
            const double envelope = std::pow(
                std::sin(
                    std::numbers::pi / 2
                    * std::clamp(std::min(t, segment.duration_s - t) / .5, 0., 1.)),
                2);
            double oscillation = 0;
            if (segment.waveform == 6) {
                const double phase =
                    2 * std::numbers::pi
                    * (segment.f0 * t
                       + .5 * (segment.f1 - segment.f0) / segment.duration_s * t * t);
                oscillation = std::sin(phase);
            } else {
                constexpr std::array hz{.37, 1.13, 2.71, 5.17};
                constexpr std::array phase{0., .7, 1.9, 2.8};
                for (std::size_t i = 0; i < hz.size(); ++i)
                    oscillation += .25 * std::sin(2 * std::numbers::pi * hz[i] * t + phase[i]);
            }
            value = segment.to + segment.amplitude * envelope * oscillation;
        }
        WheelProbeSample sample;
        sample.segment_id = static_cast<int>(index);
        sample.mode = segment.coast ? 2 : 1;
        sample.waveform = segment.waveform;
        sample.validation = segment.validation;
        for (std::size_t i = 0; i < 2; ++i) {
            sample.servo[i] = !segment.coast && segment.axis_scale[i] != 0;
            sample.velocity_target[i] = segment.coast ? 0 : segment.axis_scale[i] * value;
        }
        return sample;
    }

    // A reference update belongs to its wall-time 20 ms slot even if the
    // executor missed a tick. Such gaps remain visible in the recorded clock.
    [[nodiscard]] WheelProbeSample held_at(double elapsed) const {
        if (!std::isfinite(elapsed) || elapsed < 0 || elapsed > duration_s_)
            throw std::out_of_range("Wheel probe time outside trajectory");
        return at(std::min(duration_s_, std::floor(elapsed * kReferenceHz + 1e-9) / kReferenceHz));
    }

    [[nodiscard]] std::string manifest_json() const {
        std::ostringstream out;
        out << std::setprecision(17) << "{\"revision\":\"" << kRevision
            << "\",\"reference_frequency_hz\":50,\"duration_s\":" << duration_s_
            << ",\"segments\":[";
        for (std::size_t i = 0; i < segments_.size(); ++i) {
            const auto& s = segments_[i];
            if (i)
                out << ',';
            out << "{\"id\":" << i << ",\"label\":\"" << s.label << "\",\"start_s\":" << s.start_s
                << ",\"duration_s\":" << s.duration_s << ",\"from\":" << s.from
                << ",\"to\":" << s.to << ",\"axis_scale\":[" << s.axis_scale[0] << ','
                << s.axis_scale[1] << "],\"amplitude\":" << s.amplitude << ",\"f0\":" << s.f0
                << ",\"f1\":" << s.f1 << ",\"waveform\":" << static_cast<int>(s.waveform)
                << ",\"coast\":" << (s.coast ? "true" : "false")
                << ",\"validation\":" << (s.validation ? "true" : "false") << '}';
        }
        out << "]}";
        return out.str();
    }

private:
    template <std::size_t N>
    static void validate_speeds(const std::array<double, N>& values, double maximum) {
        double previous = 0;
        for (double speed : values) {
            if (!std::isfinite(speed) || speed <= previous || speed > maximum)
                throw std::invalid_argument("Invalid output-shaft wheel speed band");
            previous = speed;
        }
    }

    void
        add(const std::string& label, std::array<double, 2> axes, double from, double to,
            double seconds, std::uint8_t waveform = 0, bool coast = false, bool validation = false,
            double amplitude = 0, double f0 = 0, double f1 = 0) {
        segments_.push_back(
            {duration_s_, seconds, from, to, axes, amplitude, f0, f1, waveform, coast, validation,
             label});
        duration_s_ += seconds;
    }
    void step(const std::string& label, std::array<double, 2> axes, double target, double seconds) {
        add(label, axes, target, target, seconds, 3);
    }
    void platform(
        const std::string& label, std::array<double, 2> axes, double target,
        const WheelProbeConfig& c) {
        add(label + "_entry", axes, 0, target, c.ramp_s, 1);
        add(label + "_steady", axes, target, target, c.hold_s);
        add(label + "_exit", axes, target, 0, c.ramp_s, 1);
    }
    std::vector<Segment> segments_;
    double duration_s_ = 0, max_target_speed_ = 0;
};

} // namespace rmcs_core::controller::identification
