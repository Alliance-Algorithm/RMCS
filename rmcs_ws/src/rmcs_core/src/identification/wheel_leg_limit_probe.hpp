#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "identification/wheel_leg_sweep_plan.hpp"

namespace rmcs_core::controller::identification {

// Angle-domain limit probe: the hip holds its entry angle and the knee target ramps
// through the configured model-angle range in one direction. There is deliberately no
// boundary detection and no retreat: the target is driven all the way to the end, and
// only the configured torque clamp limits the effort if the linkage bottoms out. The
// measured angle at each end is recorded, so the mechanical limit is read from the
// feedback rather than inferred from a stall test.
class WheelLegLimitProbe {
public:
    struct Config {
        double start = 0, end = 360 * PairGeometry::rad;
        double speed = .5 * PairGeometry::rad;
    };
    WheelLegLimitProbe(PairGeometry geometry, Config config)
        : geometry_(std::move(geometry))
        , config_(config) {
        if (!std::isfinite(config.start) || !std::isfinite(config.end) || config.start == config.end
            || !std::isfinite(config.speed) || config.speed <= 0)
            throw std::invalid_argument("Invalid angle sweep parameters");
    }

    void start(double time, std::array<double, 2> initial) {
        stage = 1;
        result = {0, 0};
        boundary_q = {nan, nan};
        boundary_hip = {nan, nan};
        boundary_beta = {nan, nan};
        initial_ = initial;
        reference_ = initial;
        last_time_ = stage_start_ = time;
    }

    WheelLegJointSweepPlan::Sample update(double time, std::array<double, 2> measured) {
        const double dt = std::max(0.0, time - last_time_);
        last_time_ = time;
        WheelLegJointSweepPlan::Sample sample;
        sample.segment = stage;
        sample.kind = 5;
        if (stage == 3) {
            sample.q = reference_;
            return sample;
        }
        const double target = stage == 1 ? config_.start : config_.end;
        // Ramp the requested speed in at the start of every stage.
        const double ramp = sweep_blend(0, 1, time - stage_start_, 1).q;
        const double remaining = target - reference_[1];
        const double step =
            std::copysign(std::min(std::abs(remaining), config_.speed * ramp * dt), remaining);
        reference_[1] += step;
        const double velocity = dt > 0 ? step / dt : 0;
        if (std::abs(target - reference_[1]) < 1e-9) {
            const auto end = static_cast<std::size_t>(stage - 1);
            boundary_q[end] = measured[1];
            boundary_hip[end] = measured[0];
            boundary_beta[end] = geometry_.beta(measured[1] - measured[0]).value_or(nan);
            result[end] = 1;
            ++stage;
            stage_start_ = time;
        }
        sample.q = reference_;
        sample.dq = {0, velocity};
        sample.segment = stage;
        const auto ref_beta = geometry_.beta(reference_[1] - reference_[0]);
        sample.beta = ref_beta.value_or(nan);
        sample.beta_valid = ref_beta.has_value();
        sample.dbeta = ref_beta ? velocity / geometry_.at(*ref_beta).first : nan;
        return sample;
    }

    int stage = 0;
    std::array<int, 2> result{};
    std::array<double, 2> boundary_q{nan, nan}, boundary_hip{nan, nan}, boundary_beta{nan, nan};
    double entry_hip() const { return initial_[0]; }
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

private:
    PairGeometry geometry_;
    Config config_;
    std::array<double, 2> initial_{}, reference_{};
    double last_time_ = 0, stage_start_ = 0;
};

} // namespace rmcs_core::controller::identification
