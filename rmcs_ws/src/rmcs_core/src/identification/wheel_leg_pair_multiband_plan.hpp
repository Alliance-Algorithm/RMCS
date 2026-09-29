#pragma once

#include <string>
#include <tuple>
#include <vector>

#include "identification/wheel_leg_pair_identification_planner.hpp"

namespace rmcs_core::controller::identification {

struct PairMultibandConfig {
    std::array<double, 4> common_hz{}, relative_hz{};
    std::array<double, 3> common_amplitude{}, relative_amplitude{}, band_s{};
    double shape_offset = 0, eighth_turn_s = 0, dwell_s = 0, ramp_s = 0;
    double common_step = 0, relative_step = 0, step_rise_s = 0, validation_s = 0;
    double load_offset = 0, validation_relative_scale = 1;
};

// Deterministic references for a fixed chassis and the complete mounted pair.
// c=(q_hip+q_aux)/2 and d=q_aux-q_hip are motor coordinates, not knee beta.
// A loaded anchor is an explicit experiment request, not inferred from clearance.
class PairMultibandPlan {
public:
    enum class Kind { kHold, kMove, kStep, kGrid, kCommon, kRelative, kMixed, kMultisine };
    struct Segment {
        Kind kind;
        std::string name;
        std::array<double, 2> from, to; // [c, d]
        double start_s, duration_s, end_s;
        int band = 0;
        bool validation = false;
    };

    static void validate_config(const PairMultibandConfig& c) {
        for (const auto& edges : {c.common_hz, c.relative_hz})
            for (std::size_t i = 0; i < edges.size(); ++i)
                if (!std::isfinite(edges[i]) || edges[i] <= 0
                    || (i && edges[i] <= edges[i - 1]))
                    throw std::invalid_argument("Multiband frequency edges must be positive and ordered");
        for (const auto& values : {c.common_amplitude, c.relative_amplitude, c.band_s})
            for (double value : values)
                if (!std::isfinite(value) || value <= 0)
                    throw std::invalid_argument("Multiband amplitudes/durations must be positive");
        for (double value : {c.shape_offset, c.eighth_turn_s, c.dwell_s, c.ramp_s,
                             c.common_step, c.relative_step, c.step_rise_s, c.validation_s})
            if (!std::isfinite(value) || value <= 0)
                throw std::invalid_argument("Multiband scalar parameters must be positive");
        if (2 * c.ramp_s >= *std::min_element(c.band_s.begin(), c.band_s.end())
            || 2 * c.ramp_s >= c.validation_s)
            throw std::invalid_argument("Multiband taper must leave a full-amplitude interval");
        if (!std::isfinite(c.load_offset) || c.load_offset < 0 || c.load_offset > .2
            || !std::isfinite(c.validation_relative_scale) || c.validation_relative_scale <= 0)
            throw std::invalid_argument("Invalid explicit multiband loading/validation request");
    }

    PairMultibandPlan(const PairMultibandConfig& config, const std::array<double, 2>& initial,
                      double delta_low, double delta_high)
        : config_(config), low_(delta_low), high_(delta_high) {
        validate_config(config);
        const std::array start{(initial[0] + initial[1]) / 2, initial[1] - initial[0]};
        if (!std::isfinite(start[0]) || !std::isfinite(start[1])
            || !std::isfinite(low_) || !std::isfinite(high_)
            || !(low_ < start[1] && start[1] < high_))
            throw std::invalid_argument("Multiband requires a measured pose inside the pair branch");
        const double center = (low_ + high_) / 2;
        auto current = start;
        add(Kind::kHold, "initial/hold", current, current, config.dwell_s);
        // Ramp paired motor targets, not a torque bias or an inferred knee angle.
        // If motion stalls against the spring, PD error supplies restoring torque.
        // Every 0.05 rad stage is recorded, including its settling response.
        const double loading = std::clamp(center - start[1], -config.load_offset, config.load_offset);
        const auto load_steps = static_cast<int>(std::ceil(std::abs(loading) / .05));
        for (int step = 1; step <= load_steps; ++step) {
            const std::array target{start[0], start[1] + loading * step / load_steps};
            const auto name = "spring_load/stage" + std::to_string(step);
            add(Kind::kMove, name + "/move", current, target, config.eighth_turn_s);
            add(Kind::kHold, name + "/hold", target, target, config.dwell_s);
            current = target;
        }
        const auto loaded_start = current;
        const double shift = std::clamp(center - loaded_start[1], -config.shape_offset, config.shape_offset);
        for (int shape = 0; shape < 2; ++shape) {
            const std::array anchor{loaded_start[0], loaded_start[1] + shape * shift};
            const auto prefix = "shape" + std::to_string(shape);
            if (shape) {
                add(Kind::kMove, prefix + "/small_shape_change", current, anchor, config.eighth_turn_s);
                add(Kind::kHold, prefix + "/settle", anchor, anchor, config.dwell_s);
                current = anchor;
            } else {
                excitation(prefix + "/pose0", anchor);
            }
            for (int eighth = 1; eighth <= 8; ++eighth) {
                const std::array target{anchor[0] + eighth * std::numbers::pi / 4, anchor[1]};
                const auto pose = prefix + "/out/pose" + std::to_string(45 * eighth);
                add(Kind::kGrid, pose + "/move", current, target, config.eighth_turn_s);
                add(Kind::kHold, pose + "/hold", target, target, config.dwell_s);
                current = target;
                if ((shape == 0 && (eighth == 2 || eighth == 4 || eighth == 6))
                    || (shape == 1 && (eighth == 2 || eighth == 6))) {
                    if (eighth == 2)
                        steps(pose, current);
                    excitation(pose, current);
                }
            }
            for (int eighth = 7; eighth >= 0; --eighth) {
                const std::array target{anchor[0] + eighth * std::numbers::pi / 4, anchor[1]};
                const auto pose = prefix + "/return/pose" + std::to_string(45 * eighth);
                add(Kind::kGrid, pose + "/move", current, target, config.eighth_turn_s);
                add(Kind::kHold, pose + "/hold", target, target, config.dwell_s);
                current = target;
                if (shape == 0 && (eighth == 5 || eighth == 1)) {
                    add(Kind::kMultisine, pose + "/validation_multisine", current, current,
                        config.validation_s, 0, true);
                    add(Kind::kHold, pose + "/validation_tail", current, current,
                        config.dwell_s, 0, true);
                }
            }
        }
        add(Kind::kMove, "return_initial/move", current, start, config.eighth_turn_s);
        add(Kind::kHold, "return_initial/hold", start, start, config.dwell_s);
    }

    [[nodiscard]] double duration() const { return segments_.back().end_s; }
    [[nodiscard]] const std::vector<Segment>& segments() const { return segments_; }
    [[nodiscard]] double relative_amplitude(double requested, double delta) const {
        // Clip oscillation at the selected anchor. Any shift of the anchor is
        // a separate explicit loading/shape command, not inferred by this cap.
        return std::min(requested, .8 * std::min(delta - low_, high_ - delta));
    }

    [[nodiscard]] PairSample at(double elapsed) const {
        if (!std::isfinite(elapsed) || elapsed < 0 || elapsed > duration())
            throw std::out_of_range("Multiband time outside trajectory");
        auto it = std::upper_bound(segments_.begin(), segments_.end(), elapsed,
            [](double t, const Segment& s) { return t < s.end_s; });
        if (it == segments_.end())
            --it;
        const auto& s = *it;
        const double t = std::clamp(elapsed - s.start_s, 0.0, s.duration_s);
        std::array<double, 3> common{s.from[0], 0, 0}, relative{s.from[1], 0, 0};
        PairSample sample;
        sample.beta = std::numeric_limits<double>::quiet_NaN();
        sample.segment_id = static_cast<int>(it - segments_.begin());
        sample.validation = s.validation;
        if (s.kind == Kind::kMove || s.kind == Kind::kStep || s.kind == Kind::kGrid) {
            const auto b = quintic(t / s.duration_s, s.duration_s);
            for (std::size_t j = 0; j < 3; ++j) {
                common[j] += (s.to[0] - s.from[0]) * b[j];
                relative[j] += (s.to[1] - s.from[1]) * b[j];
            }
            sample.waveform = s.kind == Kind::kStep ? 3 : s.kind == Kind::kGrid ? 7 : 1;
        } else if (s.kind != Kind::kHold) {
            const auto w = window(t, s.duration_s);
            if (s.kind == Kind::kMultisine) {
                // Disjoint frequency lines and different phases for the two
                // inputs. Fixed seed-free validation, absent from training chirps.
                constexpr std::array cf{.17, .43, .79, 1.31, 2.17, 3.73};
                constexpr std::array rf{.23, .61, 1.07, 1.73, 2.41, 3.91};
                constexpr std::array ca{.08, .04, .025, .012, .005, .003};
                constexpr std::array ra{.015, .012, .008, .005, .003, .002};
                const double scale = relative_amplitude(.045 * config_.validation_relative_scale,
                                                        s.from[1]) / .045;
                for (std::size_t k = 0; k < cf.size(); ++k) {
                    const double phase = std::numbers::pi * k * (static_cast<double>(k) - 1) / cf.size();
                    oscillation(common, w, t, cf[k], cf[k], s.duration_s, ca[k], phase);
                    oscillation(relative, w, t, rf[k], rf[k], s.duration_s,
                                scale * ra[k], phase + std::numbers::pi / 3);
                }
                sample.waveform = 8;
            } else {
                if (s.kind == Kind::kCommon || s.kind == Kind::kMixed)
                    oscillation(common, w, t, config_.common_hz[s.band], config_.common_hz[s.band + 1],
                                s.duration_s, config_.common_amplitude[s.band], 0);
                if (s.kind == Kind::kRelative || s.kind == Kind::kMixed)
                    oscillation(relative, w, t, config_.relative_hz[s.band], config_.relative_hz[s.band + 1],
                                s.duration_s, relative_amplitude(config_.relative_amplitude[s.band], s.from[1]),
                                s.kind == Kind::kMixed ? std::numbers::pi / 2 : 0);
                sample.waveform = 6;
            }
        }
        sample.position = {common[0] - relative[0] / 2, common[0] + relative[0] / 2};
        sample.velocity = {common[1] - relative[1] / 2, common[1] + relative[1] / 2};
        sample.acceleration = {common[2] - relative[2] / 2, common[2] + relative[2] / 2};
        return sample;
    }

private:
    static std::array<double, 3> quintic(double u, double duration) {
        return {u * u * u * (10 + u * (-15 + 6 * u)),
                30 * u * u * (1 - u) * (1 - u) / duration,
                60 * u * (1 - 3 * u + 2 * u * u) / (duration * duration)};
    }
    [[nodiscard]] std::array<double, 3> window(double t, double duration) const {
        if (t < config_.ramp_s)
            return quintic(t / config_.ramp_s, config_.ramp_s);
        if (t > duration - config_.ramp_s) {
            auto w = quintic((duration - t) / config_.ramp_s, config_.ramp_s);
            w[1] = -w[1];
            return w;
        }
        return {1, 0, 0};
    }
    static void oscillation(std::array<double, 3>& out, const std::array<double, 3>& w,
                            double t, double f0, double f1, double duration, double amplitude,
                            double phase_offset) {
        const double rate = (f1 - f0) / duration;
        const double phase = 2 * std::numbers::pi * (f0 * t + .5 * rate * t * t) + phase_offset;
        const double omega = 2 * std::numbers::pi * (f0 + rate * t);
        const double alpha = 2 * std::numbers::pi * rate;
        const double sn = std::sin(phase), cs = std::cos(phase);
        out[0] += amplitude * w[0] * sn;
        out[1] += amplitude * (w[1] * sn + w[0] * omega * cs);
        out[2] += amplitude * (w[2] * sn + 2 * w[1] * omega * cs
                                + w[0] * (alpha * cs - omega * omega * sn));
    }
    void add(Kind kind, const std::string& name, const std::array<double, 2>& from,
             const std::array<double, 2>& to, double duration, int band = 0, bool validation = false) {
        const double start = segments_.empty() ? 0 : segments_.back().end_s;
        segments_.push_back({kind, name, from, to, start, duration, start + duration, band, validation});
    }
    void excitation(const std::string& name, const std::array<double, 2>& pose) {
        for (const auto& [kind, band, label] : std::array{
                 std::tuple{Kind::kCommon, 0, "/common_low"},
                 std::tuple{Kind::kRelative, 0, "/differential_low"},
                 std::tuple{Kind::kMixed, 1, "/mixed_mid"},
                 std::tuple{Kind::kMixed, 2, "/mixed_high"}}) {
            add(kind, name + label, pose, pose, config_.band_s[band], band);
            add(Kind::kHold, name + label + "/tail", pose, pose, config_.dwell_s);
        }
    }
    void steps(const std::string& name, const std::array<double, 2>& pose) {
        for (std::size_t axis : {0, 1})
            for (double sign : {1.0, -1.0}) {
                auto target = pose;
                target[axis] += sign * (axis == 0 ? config_.common_step
                                        : relative_amplitude(config_.relative_step, pose[1]));
                const auto label = name + (axis == 0 ? "/common_step" : "/differential_step")
                                     + (sign > 0 ? "/positive" : "/negative");
                add(Kind::kStep, label, pose, target, config_.step_rise_s);
                add(Kind::kHold, label + "/hold", target, target, config_.dwell_s);
                add(Kind::kStep, label + "/return", target, pose, config_.step_rise_s);
                add(Kind::kHold, label + "/tail", pose, pose, config_.dwell_s);
            }
    }

    PairMultibandConfig config_;
    double low_, high_;
    std::vector<Segment> segments_;
};

} // namespace rmcs_core::controller::identification
