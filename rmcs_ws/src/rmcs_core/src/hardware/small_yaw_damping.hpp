#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <stdexcept>

namespace rmcs_core::hardware {

// The motor's native speed feedback is too coarsely quantized for slow drift.
// Estimate from the 16-bit encoder instead, using a fixed 40 ms angle window.
class SmallYawMotionEstimator {
public:
    double update(double angle_rad, double dt) noexcept {
        if (!std::isfinite(angle_rad) || !std::isfinite(dt) || dt <= 0.0 || dt > 0.02) {
            reset();
            return 0.0;
        }
        const double wrapped_angle = std::remainder(angle_rad, kFullTurn);
        if (!have_angle_) {
            seed(wrapped_angle);
            return 0.0;
        }

        time_ += dt;
        position_ += std::remainder(wrapped_angle - previous_angle_, kFullTurn);
        previous_angle_ = wrapped_angle;

        // Two encoder counts against the established direction distinguish a
        // reversal from the observed one-count idle jitter. Withdraw the old
        // estimate immediately instead of waiting for its window to change sign.
        if (direction_ != 0) {
            const double retreat = direction_ * (direction_extreme_ - position_);
            if (retreat > kMotionResolution) {
                seed(wrapped_angle);
                return 0.0;
            }
            if (direction_ * (position_ - direction_extreme_) > 0.0)
                direction_extreme_ = position_;
        }

        if (time_ - newest_sample().time + 1e-12 >= kSamplePeriod)
            append_sample({time_, position_});

        const double target_time = time_ - kWindow;
        const auto oldest_index = (next_sample_ + kCapacity - sample_count_) % kCapacity;
        if (samples_[oldest_index].time > target_time + 1e-12)
            return 0.0;

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

        const double displacement = position_ - old_position;
        if (std::abs(displacement) <= kMotionResolution) {
            direction_ = 0;
            return 0.0;
        }
        const int direction = displacement > 0.0 ? 1 : -1;
        if (direction != direction_) {
            direction_ = direction;
            direction_extreme_ = position_;
        }
        return displacement / kWindow;
    }

    void reset() noexcept {
        have_angle_ = false;
        previous_angle_ = time_ = position_ = direction_extreme_ = 0.0;
        direction_ = 0;
        next_sample_ = sample_count_ = 0;
    }

private:
    struct Sample {
        double time = 0.0;
        double position = 0.0;
    };
    static constexpr double kFullTurn = 2.0 * std::numbers::pi;
    static constexpr double kWindow = 0.04;
    static constexpr double kSamplePeriod = 0.005;
    static constexpr double kMotionResolution = 1.5 * kFullTurn / 65536.0;
    static constexpr std::size_t kCapacity = 16;

    void append_sample(Sample sample) noexcept {
        samples_[next_sample_] = sample;
        next_sample_ = (next_sample_ + 1) % kCapacity;
        sample_count_ = std::min(sample_count_ + 1, kCapacity);
    }

    const Sample& newest_sample() const noexcept {
        return samples_[(next_sample_ + kCapacity - 1) % kCapacity];
    }

    void seed(double wrapped_angle) noexcept {
        reset();
        have_angle_ = true;
        previous_angle_ = wrapped_angle;
        append_sample({0.0, 0.0});
    }

    std::array<Sample, kCapacity> samples_{};
    std::size_t next_sample_ = 0;
    std::size_t sample_count_ = 0;
    bool have_angle_ = false;
    double previous_angle_ = 0.0;
    double time_ = 0.0;
    double position_ = 0.0;
    int direction_ = 0;
    double direction_extreme_ = 0.0;
};

struct SmallYawDampingParameters {
    bool enabled = false;
    double torque_nm = 2.5;
    double start_velocity_rad_s = 0.006;
    double stop_velocity_rad_s = 0.002;
    double torque_slew_nm_s = 18.0;
};

// All values use the configured motor's physical angle/velocity/torque convention.
// This is a brake, with no position target or standstill holding torque.
class SmallYawDamping {
public:
    explicit SmallYawDamping(SmallYawDampingParameters parameters)
        : parameters_(parameters) {
        if (!std::isfinite(parameters_.torque_nm) || parameters_.torque_nm <= 0.0
            || parameters_.torque_nm > 2.5)
            throw std::invalid_argument("small yaw damping torque must be in (0, 2.5] Nm");
        if (!std::isfinite(parameters_.start_velocity_rad_s)
            || !std::isfinite(parameters_.stop_velocity_rad_s)
            || parameters_.stop_velocity_rad_s < 0.0
            || parameters_.start_velocity_rad_s <= parameters_.stop_velocity_rad_s)
            throw std::invalid_argument("small yaw damping needs 0 <= stop velocity < start velocity");
        if (!std::isfinite(parameters_.torque_slew_nm_s) || parameters_.torque_slew_nm_s <= 0.0)
            throw std::invalid_argument("small yaw damping torque slew must be positive and finite");
    }

    double update(double velocity, bool permitted, double dt) noexcept {
        if (!parameters_.enabled || !permitted || !std::isfinite(velocity)
            || !std::isfinite(dt) || dt <= 0.0
            || std::abs(velocity) <= parameters_.stop_velocity_rad_s) {
            reset();
            return 0.0;
        }

        const int direction = velocity > 0.0 ? 1 : -1;
        // An old braking command would propel the motor after a sign change.
        // Withdraw it immediately, including reversals inside the hysteresis band.
        if (direction != active_direction_)
            reset();
        if (active_direction_ == 0) {
            if (std::abs(velocity) < parameters_.start_velocity_rad_s)
                return 0.0;
            active_direction_ = direction;
        }

        // Only building torque is rate limited. Every stop/fault clears both
        // the output and latch, and a scheduling gap cannot create a torque jump.
        const double maximum_step = parameters_.torque_slew_nm_s * std::min(dt, 0.02);
        torque_magnitude_ = std::min(parameters_.torque_nm, torque_magnitude_ + maximum_step);
        return direction > 0 ? -torque_magnitude_ : torque_magnitude_;
    }

    void reset() noexcept {
        active_direction_ = 0;
        torque_magnitude_ = 0.0;
    }

private:
    SmallYawDampingParameters parameters_;
    int active_direction_ = 0;
    double torque_magnitude_ = 0.0;
};

// LK MG5010E_i36V3, configured reverse(): convert physical Nm into a signed
// raw-current command. Do not change this sign independently of feedback.reverse().
inline std::array<std::byte, 8> encode_small_yaw_damping_torque(
    double torque_nm, double torque_limit_nm = 6.0) noexcept {
    constexpr double kTorquePerRawCurrent = 3.6 * 66.0 / 4096.0;
    // The legacy damping path defaults to 6 Nm. Robust control may explicitly
    // request up to 10 Nm; identification retains its separate 7 Nm validation.
    const double limit = std::isfinite(torque_limit_nm)
        ? std::clamp(torque_limit_nm, 0.0, 10.0) : 6.0;
    const double bounded = std::isfinite(torque_nm)
        ? std::clamp(torque_nm, -limit, limit) : 0.0;
    const auto raw_current = static_cast<int16_t>(std::round(-bounded / kTorquePerRawCurrent));
    const auto encoded = static_cast<uint16_t>(raw_current);
    std::array<std::byte, 8> command{};
    command[0] = std::byte{0xA1};
    command[4] = static_cast<std::byte>(encoded & 0xFFU);
    command[5] = static_cast<std::byte>((encoded >> 8U) & 0xFFU);
    return command;
}

} // namespace rmcs_core::hardware
