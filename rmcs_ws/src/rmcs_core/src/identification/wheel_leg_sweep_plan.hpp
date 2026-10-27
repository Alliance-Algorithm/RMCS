#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <utility>
#include <vector>

#include "identification/wheel_leg_pair_recording_plan.hpp"

namespace rmcs_core::controller::identification {

struct SweepScalar {
    double q = 0, dq = 0, ddq = 0;
};

inline SweepScalar sweep_blend(double from, double to, double t, double duration) {
    const double x = std::clamp(t / duration, 0.0, 1.0), d = to - from;
    return {
        from + d * x * x * x * (10 + x * (-15 + 6 * x)),
        d * 30 * x * x * (1 - x) * (1 - x) / duration,
        d * 60 * x * (1 - x) * (1 - 2 * x) / (duration * duration)};
}

inline SweepScalar sweep_turn(double from, double to, double t, double duration) {
    constexpr double ramp = 1.0;
    const double sign = to >= from ? 1 : -1, v = std::abs(to - from) / (duration - ramp);
    const auto head = [v](double s) {
        const double phase = std::numbers::pi * s / ramp;
        return SweepScalar{
            v * (s / 2 - ramp / (2 * std::numbers::pi) * std::sin(phase)),
            v / 2 * (1 - std::cos(phase)), v * std::numbers::pi / (2 * ramp) * std::sin(phase)};
    };
    t = std::clamp(t, 0.0, duration);
    SweepScalar value;
    if (t < ramp)
        value = head(t);
    else if (t > duration - ramp) {
        value = head(duration - t);
        value.q = std::abs(to - from) - value.q;
        value.ddq = -value.ddq;
    } else
        value = {v * (t - ramp / 2), v, 0};
    return {from + sign * value.q, sign * value.dq, sign * value.ddq};
}

class WheelLegJointSweepPlan {
public:
    struct Config {
        std::array<double, 4> turn_speeds{.2, .6, 1.5, std::numbers::pi};
        double fast_turn_period_s = 2;
        std::array<double, 2> probe_torque_nm{1, 1};
    };
    struct Sample {
        std::array<double, 2> q{}, dq{}, ddq{}, probe_torque{};
        double theta = 0, beta = 0, dtheta = 0, dbeta = 0;
        int segment = 0, kind = 0;
        bool beta_valid = true, validation = false;
    };
    struct Pose {
        double theta, beta;
    };
    struct Segment {
        Pose from, to;
        double start, duration;
        int kind; // 0: entry, 1: move/hold, 2: turn, 3: chirp, 4: rotating multisine, 6: local
                  // probe
        double amplitude = 0, f0 = 0, f1 = 0;
        bool validation = false;
    };

    WheelLegJointSweepPlan(PairGeometry geometry, double hip_sign)
        : WheelLegJointSweepPlan(std::move(geometry), hip_sign, Config{}) {}

    WheelLegJointSweepPlan(PairGeometry geometry, double hip_sign, Config config)
        : geometry_(std::move(geometry))
        , hip_sign_(hip_sign)
        , config_(config) {
        for (double speed : config_.turn_speeds)
            if (!std::isfinite(speed) || speed < .05 || speed > 6)
                throw std::invalid_argument("Invalid common-turn speed");
        if (!std::isfinite(config_.fast_turn_period_s) || config_.fast_turn_period_s < 1
            || config_.fast_turn_period_s > 20)
            throw std::invalid_argument("Invalid fast-turn period");
        for (double torque : config_.probe_torque_nm)
            if (!std::isfinite(torque) || torque <= 0 || torque > 3)
                throw std::invalid_argument("Invalid independent torque-probe amplitude");
    }

    void start(
        int mode, std::array<double, 2> initial, double hip_zero, double beta_min,
        double beta_max) {
        segments_.clear();
        duration_ = 0;
        initial_ = initial;
        hip_zero_ = hip_zero;
        const double theta = (initial[0] - hip_zero) / hip_sign_;
        Pose pose{theta, mode == 1 ? 70 * rad : beta_min};
        add(pose, pose, 5, 0);
        if (mode == 1) {
            // Two continuous revolutions leave a full constant-speed revolution at pi rad/s.
            // Low-speed forward/reverse pairs distinguish viscous and Coulomb effects.
            for (double speed : config_.turn_speeds) {
                add(pose, {theta + 4 * pi, pose.beta}, 4 * pi / speed + 1, 2);
                pose = segments_.back().to;
                add(pose, {theta, pose.beta}, 4 * pi / speed + 1, 2);
                pose = segments_.back().to;
                add(pose, pose, 2, 1);
            }
        } else if (mode == 2) {
            for (double speed : {3.0, 12.0, 30.0}) {
                for (int direction : {1, -1}) {
                    const double end = direction > 0 ? beta_max : beta_min;
                    while (std::abs(pose.beta - end) > 1e-8) {
                        const double next = direction > 0 ? std::min(end, pose.beta + 5 * rad)
                                                          : std::max(end, pose.beta - 5 * rad);
                        add(pose, {theta, next}, 1.875 * std::abs(next - pose.beta) / (speed * rad),
                            1);
                        pose = segments_.back().to;
                        add(pose, pose, 1, 1);
                    }
                }
            }
            for (double center : {50.0, 75.0, 100.0}) {
                // Configuration must leave the full requested excitation in its range.
                if (center * rad - 4 * rad < beta_min - 1e-9
                    || center * rad + 4 * rad > beta_max + 1e-9)
                    throw std::invalid_argument(
                        "Beta range excludes a requested chirp; no auto scaling");
                add(pose, {theta, center * rad}, 5, 1);
                pose = segments_.back().to;
                add(pose, pose, 30, 3, 4 * rad, .1, .8);
                add(pose, pose, 30, 3, 2 * rad, .8, 3);
                add(pose, pose, 30, 3, .6 * rad, 3, 10);
            }
        } else if (mode == 3) {
            for (double center : {50.0, 75.0, 100.0, 60.0, 90.0}) {
                const bool validation = center == 60 || center == 90;
                if (center * rad - 2 * rad < beta_min || center * rad + 2 * rad > beta_max)
                    throw std::invalid_argument("Beta range excludes a requested multisine");
                add(pose, {theta, center * rad}, 5, 1);
                pose = segments_.back().to;
                if (!validation) {
                    // Separate reference bands and additive torque probes make the plant
                    // input observable. A second complete segment is reserved for validation.
                    add(pose, pose, 30, 6, 2 * rad);
                    add(pose, pose, 30, 6, 2 * rad, 0, 0, true);
                }
                const double turn_s = 2 * config_.fast_turn_period_s + 1;
                add(pose, {theta + 4 * pi, pose.beta}, turn_s, 4, 2 * rad, 0, 0, validation);
                pose = segments_.back().to;
                add(pose, {theta, pose.beta}, turn_s, 4, 2 * rad, 0, 0, validation);
                pose = segments_.back().to;
            }
        } else
            throw std::invalid_argument("Unknown joint sweep mode");
    }

    Sample at(double elapsed) const {
        if (segments_.empty())
            throw std::logic_error("Joint sweep not started");
        const auto it = std::upper_bound(
            segments_.begin(), segments_.end(), elapsed,
            [](double t, const Segment& s) { return t < s.start; });
        const auto index =
            it == segments_.begin() ? 0 : static_cast<std::size_t>(it - segments_.begin() - 1);
        const auto& s = segments_[index];
        const double t = std::clamp(elapsed - s.start, 0.0, s.duration);
        auto theta = s.kind == 2 || s.kind == 4
                       ? sweep_turn(s.from.theta, s.to.theta, t, s.duration)
                       : sweep_blend(s.from.theta, s.to.theta, t, s.duration);
        auto beta = sweep_blend(s.from.beta, s.to.beta, t, s.duration);
        Sample out;
        if (s.kind == 3 || s.kind == 4 || s.kind == 6) {
            // C2 envelope: value, velocity and acceleration vanish at both ends.
            const auto rise = sweep_blend(0, 1, t, 3);
            const auto fall = sweep_blend(0, 1, s.duration - t, 3);
            const SweepScalar envelope{
                rise.q * fall.q, rise.dq * fall.q - rise.q * fall.dq,
                rise.ddq * fall.q - 2 * rise.dq * fall.dq + rise.q * fall.ddq};
            auto inject = [&](SweepScalar& x, double amplitude, double phase, double w, double dw) {
                const double sn = std::sin(phase), cs = std::cos(phase);
                x.q += amplitude * envelope.q * sn;
                x.dq += amplitude * (envelope.dq * sn + envelope.q * w * cs);
                x.ddq += amplitude
                       * (envelope.ddq * sn + 2 * envelope.dq * w * cs
                          + envelope.q * (dw * cs - w * w * sn));
            };
            if (s.kind == 3) {
                const double k = (s.f1 - s.f0) / s.duration;
                inject(
                    beta, s.amplitude, 2 * pi * (s.f0 * t + k * t * t / 2), 2 * pi * (s.f0 + k * t),
                    2 * pi * k);
            } else {
                constexpr std::array tf{.17, .43, .79, 1.31, 2.17, 3.73};
                constexpr std::array bf{.23, .61, 1.07, 1.73, 2.41, 3.91};
                const double phase = s.validation ? 1.27 : .31;
                for (std::size_t i = 0; i < tf.size(); ++i) {
                    inject(
                        theta, 3 * rad / tf.size(), 2 * pi * tf[i] * t + i * .7 + phase,
                        2 * pi * tf[i], 0);
                    inject(
                        beta, s.amplitude / bf.size(), 2 * pi * bf[i] * t + i * 1.1 + phase + 1,
                        2 * pi * bf[i], 0);
                }
                if (s.kind == 6) {
                    constexpr std::array hf{.37, .83, 1.91, 3.17, 5.23, 8.71};
                    constexpr std::array kf{.29, .97, 1.57, 2.83, 4.61, 9.13};
                    for (std::size_t axis = 0; axis < 2; ++axis)
                        for (std::size_t i = 0; i < hf.size(); ++i) {
                            const double hz = axis == 0 ? hf[i] : kf[i];
                            out.probe_torque[axis] +=
                                envelope.q * config_.probe_torque_nm[axis] / hf.size()
                                * std::sin(2 * pi * hz * t + i * 1.3 + phase + axis);
                        }
                }
            }
        }
        const auto g = geometry_.at(beta.q);
        out.q = {hip_zero_ + hip_sign_ * theta.q, hip_zero_ + hip_sign_ * theta.q + g.position};
        out.dq = {hip_sign_ * theta.dq, hip_sign_ * theta.dq + g.first * beta.dq};
        out.ddq = {
            hip_sign_ * theta.ddq,
            hip_sign_ * theta.ddq + g.first * beta.ddq + g.second * beta.dq * beta.dq};
        out.theta = theta.q;
        out.beta = beta.q;
        out.dtheta = theta.dq;
        out.dbeta = beta.dq;
        out.segment = static_cast<int>(index);
        out.kind = s.kind;
        out.validation = s.validation;
        if (s.kind == 0) {
            for (std::size_t i = 0; i < 2; ++i) {
                const auto blend = sweep_blend(initial_[i], out.q[i], t, s.duration);
                out.q[i] = blend.q;
                out.dq[i] = blend.dq;
                out.ddq[i] = blend.ddq;
            }
            const auto estimate = geometry_.beta(out.q[1] - out.q[0]);
            out.beta_valid = estimate.has_value();
            out.beta = estimate.value_or(std::numeric_limits<double>::quiet_NaN());
            out.dbeta = estimate ? (out.dq[1] - out.dq[0]) / geometry_.at(*estimate).first
                                 : std::numeric_limits<double>::quiet_NaN();
        }
        return out;
    }

    double duration() const { return duration_; }
    const auto& segments() const { return segments_; }
    static constexpr double pi = std::numbers::pi, rad = pi / 180;

private:
    void
        add(Pose from, Pose to, double duration, int kind, double amplitude = 0, double f0 = 0,
            double f1 = 0, bool validation = false) {
        geometry_.at(from.beta);
        geometry_.at(to.beta);
        segments_.push_back({from, to, duration_, duration, kind, amplitude, f0, f1, validation});
        duration_ += duration;
    }
    PairGeometry geometry_;
    double hip_sign_, hip_zero_ = 0, duration_ = 0;
    Config config_;
    std::array<double, 2> initial_{};
    std::vector<Segment> segments_;
};

} // namespace rmcs_core::controller::identification
