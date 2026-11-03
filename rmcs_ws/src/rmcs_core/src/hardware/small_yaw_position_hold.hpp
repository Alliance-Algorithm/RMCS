#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <stdexcept>

#include "controller/robust/robust_calculator.hpp"

namespace rmcs_core::hardware {

struct SmallYawPositionHoldParameters {
    bool enabled = false;
    double position_gain_nm_rad = 20.0;
    double velocity_gain_nm_s_rad = 2.5;
    double torque_limit_nm = 2.5;
    double torque_slew_nm_s = 75.0;
    double position_deadband_rad = 0.0002;
    bool sliding_enabled = false;
    double sliding_position_gain = 4.0; // 1/s
    double sliding_velocity_limit_rad_s = 0.35;
    controller::robust::RobustVelocityParameters sliding_model;
};

// Hold the relative motor angle captured on each permission transition. The
// short encoder derivative avoids applying the 40 ms drift estimator's delayed
// velocity to this stiffer position loop. The supplied velocity is used during
// the first 10 ms while the short window is being populated.
class SmallYawPositionHold {
public:
    explicit SmallYawPositionHold(SmallYawPositionHoldParameters parameters)
        : parameters_(parameters), sliding_controller_(sliding_parameters(parameters)) {
        if (!std::isfinite(parameters_.position_gain_nm_rad)
            || parameters_.position_gain_nm_rad <= 0.0
            || !std::isfinite(parameters_.velocity_gain_nm_s_rad)
            || parameters_.velocity_gain_nm_s_rad < 0.0
            || !std::isfinite(parameters_.torque_limit_nm)
            || parameters_.torque_limit_nm <= 0.0
            || parameters_.torque_limit_nm > (parameters_.sliding_enabled ? 10.0 : 2.5)
            || !std::isfinite(parameters_.torque_slew_nm_s)
            || parameters_.torque_slew_nm_s <= 0.0
            || !std::isfinite(parameters_.position_deadband_rad)
            || parameters_.position_deadband_rad < 0.0
            || parameters_.position_deadband_rad >= std::numbers::pi
            || (parameters_.sliding_enabled
                && (!std::isfinite(parameters_.sliding_position_gain)
                    || parameters_.sliding_position_gain <= 0.0
                    || !std::isfinite(parameters_.sliding_velocity_limit_rad_s)
                    || parameters_.sliding_velocity_limit_rad_s <= 0.0)))
            throw std::invalid_argument("invalid small yaw position hold parameters");
    }

    double update(double angle, double velocity, bool permitted, double dt) noexcept {
        if (!parameters_.enabled || !permitted || !std::isfinite(angle)
            || !std::isfinite(velocity) || !std::isfinite(dt) || dt <= 0.0 || dt > 0.02) {
            reset();
            return 0.0;
        }
        const double wrapped_angle = std::remainder(angle, kFullTurn);
        if (!reference_valid_) {
            reference_valid_ = true;
            target_angle_ = previous_angle_ = wrapped_angle;
            append_sample({0.0, 0.0});
            return 0.0;
        }

        time_ += dt;
        position_ += std::remainder(wrapped_angle - previous_angle_, kFullTurn);
        previous_angle_ = wrapped_angle;
        position_error_ = std::remainder(target_angle_ - wrapped_angle, kFullTurn);
        if (time_ - newest_sample().time + 1e-12 >= kSamplePeriod)
            append_sample({time_, position_});

        velocity_ = short_window_velocity(velocity);
        const double restoring_error = std::abs(position_error_) <= parameters_.position_deadband_rad
            ? 0.0 : position_error_;
        target_velocity_ = parameters_.sliding_enabled
            ? std::clamp(parameters_.sliding_position_gain * restoring_error,
                         -parameters_.sliding_velocity_limit_rad_s,
                         parameters_.sliding_velocity_limit_rad_s)
            : 0.0;
        // With an unsaturated position reference, the velocity error is the
        // position sliding surface lambda * (target - angle) - velocity.
        const double unconstrained_demand = parameters_.sliding_enabled
            ? sliding_controller_.update(target_velocity_, velocity_, 0.0, dt)
            : parameters_.position_gain_nm_rad * restoring_error
                - parameters_.velocity_gain_nm_s_rad * velocity_;
        if (!std::isfinite(unconstrained_demand)) {
            reset();
            return 0.0;
        }
        const double demand = std::clamp(
            unconstrained_demand, -parameters_.torque_limit_nm, parameters_.torque_limit_nm);

        // Withdraw a superseded command immediately. In particular, a slew
        // limiter must never preserve the old sign after the demand reverses.
        // Reductions also take effect immediately; only torque buildup is limited.
        if (demand == 0.0 || torque_ * demand < 0.0)
            torque_ = 0.0;
        if (std::abs(demand) < std::abs(torque_))
            torque_ = demand;
        const double step = parameters_.torque_slew_nm_s * dt;
        torque_ += std::clamp(demand - torque_, -step, step);
        return torque_;
    }

    void reset() noexcept {
        reference_valid_ = false;
        target_angle_ = position_error_ = torque_ = previous_angle_ = velocity_ = 0.0;
        target_velocity_ = 0.0;
        sliding_controller_.reset();
        time_ = position_ = 0.0;
        next_sample_ = sample_count_ = 0;
    }

    bool reference_valid() const noexcept { return reference_valid_; }
    double target_angle() const noexcept { return target_angle_; }
    double position_error() const noexcept { return position_error_; }
    double velocity() const noexcept { return velocity_; }
    double target_velocity() const noexcept { return target_velocity_; }

private:
    static controller::robust::RobustVelocityParameters sliding_parameters(
        const SmallYawPositionHoldParameters& parameters) {
        if (!parameters.sliding_enabled)
            return {};
        auto model = parameters.sliding_model;
        model.auxiliary_law = controller::robust::AuxiliaryLaw::Sliding;
        model.output_min = -parameters.torque_limit_nm;
        model.output_max = parameters.torque_limit_nm;
        // Position hold has no independently accumulated load state.
        model.integral_gain = model.integral_torque_limit = 0.0;
        return model;
    }

    struct Sample {
        double time;
        double position;
    };
    static constexpr double kFullTurn = 2.0 * std::numbers::pi;
    static constexpr double kVelocityWindow = 0.01;
    static constexpr double kSamplePeriod = 0.001;
    static constexpr std::size_t kCapacity = 32;

    void append_sample(Sample sample) noexcept {
        samples_[next_sample_] = sample;
        next_sample_ = (next_sample_ + 1) % kCapacity;
        sample_count_ = std::min(sample_count_ + 1, kCapacity);
    }

    const Sample& newest_sample() const noexcept {
        return samples_[(next_sample_ + kCapacity - 1) % kCapacity];
    }

    double short_window_velocity(double fallback) const noexcept {
        const double target_time = time_ - kVelocityWindow;
        const auto oldest_index = (next_sample_ + kCapacity - sample_count_) % kCapacity;
        if (samples_[oldest_index].time > target_time + 1e-12)
            return fallback;
        Sample before = samples_[oldest_index];
        double old_position = before.position;
        for (std::size_t offset = 1; offset < sample_count_; ++offset) {
            const auto& after = samples_[(oldest_index + offset) % kCapacity];
            if (after.time >= target_time) {
                const double fraction = std::clamp(
                    (target_time - before.time) / (after.time - before.time), 0.0, 1.0);
                old_position = before.position + fraction * (after.position - before.position);
                break;
            }
            before = after;
            old_position = before.position;
        }
        return (position_ - old_position) / kVelocityWindow;
    }

    SmallYawPositionHoldParameters parameters_;
    controller::robust::RobustVelocityCalculator sliding_controller_;
    bool reference_valid_ = false;
    double target_angle_ = 0.0;
    double position_error_ = 0.0;
    double torque_ = 0.0;
    double velocity_ = 0.0;
    double target_velocity_ = 0.0;
    double previous_angle_ = 0.0;
    double time_ = 0.0;
    double position_ = 0.0;
    std::array<Sample, kCapacity> samples_{};
    std::size_t next_sample_ = 0;
    std::size_t sample_count_ = 0;
};

} // namespace rmcs_core::hardware
