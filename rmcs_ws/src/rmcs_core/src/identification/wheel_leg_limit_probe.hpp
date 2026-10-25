#pragma once

#include <array>
#include <cmath>
#include <limits>

#include "identification/wheel_leg_sweep_plan.hpp"

namespace rmcs_core::controller::identification {

// A stall is only a candidate boundary (friction/current saturation can also
// cause it). Keep raw positions and an explicit result code for both ends.
class WheelLegLimitProbe {
public:
    struct Config {
        double low = 30 * PairGeometry::rad, high = 110 * PairGeometry::rad;
        double speed = .5 * PairGeometry::rad, retract = 3 * PairGeometry::rad;
        double window = 2, motion = .15 * PairGeometry::rad, effort = 4;
    };
    WheelLegLimitProbe(PairGeometry geometry, Config config)
        : geometry_(std::move(geometry))
        , config_(config) {
        geometry_.at(config.low);
        geometry_.at(config.high);
        if (!(config.low < config.high) || !std::isfinite(config.speed) || config.speed <= 0
            || !std::isfinite(config.retract) || config.retract <= 0
            || config.retract >= config.high - config.low || !std::isfinite(config.window)
            || config.window < .5 || !std::isfinite(config.motion) || config.motion <= 0
            || !std::isfinite(config.effort) || config.effort <= 0)
            throw std::invalid_argument("Invalid limit probe parameters");
    }

    void start(double time, std::array<double, 2> initial) {
        stage = 1;
        result = {0, 0};
        boundary_q = {nan, nan};
        boundary_beta = {nan, nan};
        initial_ = initial;
        reference_ = initial;
        window_q_ = initial[1];
        last_time_ = window_time_ = stage_start_ = time;
        effort_qualified_ = true;
        target_time_ = -1;
    }

    WheelLegJointSweepPlan::Sample
        update(double time, std::array<double, 2> measured, double last_knee_effort) {
        const double dt = std::max(0.0, time - last_time_);
        last_time_ = time;
        WheelLegJointSweepPlan::Sample sample;
        sample.segment = stage;
        sample.kind = 5;
        if (stage == 5) {
            sample.q = reference_;
            return sample;
        }
        const auto beta = geometry_.beta(reference_[1] - reference_[0]);
        const double derivative = geometry_.at(beta.value_or(70 * PairGeometry::rad)).first;
        const bool seeking = stage == 1 || stage == 3;
        double target_delta;
        if (seeking)
            target_delta = geometry_.at(stage == 1 ? config_.low : config_.high).position;
        else
            target_delta = retract_delta_;
        const double target = initial_[0] + target_delta;
        // Ramp the requested speed at every stage, including after a stall.
        const double ramp = sweep_blend(0, 1, time - stage_start_, 1).q;
        const double remaining = target - reference_[1];
        const double step = std::copysign(
            std::min(std::abs(remaining), std::abs(derivative) * config_.speed * ramp * dt),
            remaining);
        reference_[1] += step;
        const double velocity = dt > 0 ? step / dt : 0;
        const bool at_target = std::abs(target - reference_[1]) < 1e-9;
        if (at_target && target_time_ < 0)
            target_time_ = time;
        effort_qualified_ = effort_qualified_ && std::abs(last_knee_effort) >= config_.effort;
        const bool window_done = time - window_time_ >= config_.window;
        const bool stalled = seeking && window_done && effort_qualified_
                          && std::abs(measured[1] - window_q_) < config_.motion;
        if (seeking && (stalled || (at_target && time - target_time_ >= config_.window))) {
            const auto end = stage == 1 ? 0 : 1;
            boundary_q[end] = measured[1];
            const auto measured_beta = geometry_.beta(measured[1] - measured[0]);
            boundary_beta[end] = measured_beta.value_or(nan);
            result[end] = stalled ? 1 : 2; // 1: suspected boundary; 2: commanded range exhausted
            const double retreat_sign = stage == 1 ? 1 : -1;
            // Retreat from the measured position, never the stalled reference.
            // No clipping/extrapolation of an invalid measured beta is needed.
            retract_delta_ = measured[1] - initial_[0]
                           + retreat_sign * std::copysign(1.0, derivative) * std::abs(derivative)
                                 * config_.retract;
            if (measured_beta) {
                const double target_beta = *measured_beta + retreat_sign * config_.retract;
                if (target_beta >= config_.low && target_beta <= config_.high)
                    retract_delta_ = geometry_.at(target_beta).position;
            }
            reference_[1] = measured[1];
            ++stage;
            next_stage(time, measured[1]);
        } else if (!seeking && at_target && time - target_time_ >= 1) {
            ++stage;
            next_stage(time, measured[1]);
        } else if (window_done) {
            window_time_ = time;
            window_q_ = measured[1];
            effort_qualified_ = true;
        }
        sample.q = reference_;
        sample.dq = {0, sample.segment == stage ? velocity : 0};
        sample.segment = stage;
        const auto ref_beta = geometry_.beta(reference_[1] - reference_[0]);
        sample.beta = ref_beta.value_or(nan);
        sample.beta_valid = ref_beta.has_value();
        sample.dbeta = ref_beta ? velocity / geometry_.at(*ref_beta).first : nan;
        return sample;
    }

    int stage = 0;
    std::array<int, 2> result{};
    std::array<double, 2> boundary_q{nan, nan}, boundary_beta{nan, nan};
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

private:
    void next_stage(double time, double measured) {
        last_time_ = window_time_ = stage_start_ = time;
        window_q_ = measured;
        target_time_ = -1;
        effort_qualified_ = true;
    }
    PairGeometry geometry_;
    Config config_;
    std::array<double, 2> initial_{}, reference_{};
    double last_time_ = 0, window_time_ = 0, stage_start_ = 0, window_q_ = 0;
    double target_time_ = -1, retract_delta_ = 0;
    bool effort_qualified_ = true;
};

} // namespace rmcs_core::controller::identification
