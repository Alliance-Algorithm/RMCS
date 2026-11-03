#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>

#include "hardware/joint2_encoder_velocity.hpp"

namespace {

using rmcs_core::hardware::Joint2EncoderVelocityEstimator;
constexpr double kCountRad = 2.0 * std::numbers::pi / 65536.0;
constexpr int64_t kStartNs = 5'000'000'000;
constexpr int64_t kMsNs = 1'000'000;

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

void near(double actual, double expected, const char* message, double tolerance = 1e-10) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
        throw std::runtime_error(std::string{message} + ": actual=" + std::to_string(actual)
                                 + ", expected=" + std::to_string(expected));
}

int wrap(int64_t counts) {
    return static_cast<int>((counts % 65536 + 65536) % 65536);
}

void stationary_quantization_noise_is_attenuated() {
    Joint2EncoderVelocityEstimator estimator;
    uint32_t random = 17;
    double velocity_squared = 0.0;
    double max_estimate = 0.0;
    double max_frame_difference = 0.0;
    int previous_raw = 40000;
    int count = 0;
    for (int ms = 0; ms <= 2000; ++ms) {
        random = random * 1664525U + 1013904223U;
        const int raw = 40000 + static_cast<int>((random >> 16) % 11) - 5;
        const double velocity = estimator.update(raw, kStartNs + ms * kMsNs);
        if (ms >= 20) {
            velocity_squared += velocity * velocity;
            max_estimate = std::max(max_estimate, std::abs(velocity));
            max_frame_difference = std::max(
                max_frame_difference, std::abs((raw - previous_raw) * kCountRad / 0.001));
            ++count;
        } else {
            near(velocity, 0.0, "startup must not produce a short-baseline derivative");
        }
        previous_raw = raw;
    }
    require(max_frame_difference > 0.9, "fixture must reproduce the observed 1 rad/s noise");
    require(max_estimate < 0.05, "stationary jitter must not produce large speed peaks");
    require(std::sqrt(velocity_squared / count) < 0.015, "stationary RMS must remain small");
}

void constant_velocity_and_wrap_work_in_both_directions() {
    for (const int direction : {-1, 1}) {
        Joint2EncoderVelocityEstimator estimator;
        for (int ms = 0; ms <= 300; ++ms) {
            const int raw = wrap(65510 + direction * 3 * ms);
            const double velocity = estimator.update(raw, kStartNs + ms * kMsNs);
            near(velocity, ms < 20 ? 0.0 : direction * 3000.0 * kCountRad,
                 "constant speed must survive a single-turn wrap");
        }
    }
}

void uneven_sample_timing_uses_received_time() {
    Joint2EncoderVelocityEstimator estimator;
    int ms = 0;
    for (int index = 0; index < 100; ++index) {
        ms += index % 3 + 1;
        const double velocity = estimator.update(wrap(65000 + 2 * ms), kStartNs + ms * kMsNs);
        if (ms >= 21)
            near(velocity, 2000.0 * kCountRad, "uneven sample times must retain speed accuracy");
    }
}

void stopping_and_reversing_do_not_retain_old_motion() {
    Joint2EncoderVelocityEstimator estimator;
    for (int ms = 0; ms <= 50; ++ms)
        estimator.update(20000 + ms * 2, kStartNs + ms * kMsNs);
    double velocity = 0.0;
    for (int ms = 51; ms <= 75; ++ms)
        velocity = estimator.update(20100, kStartNs + ms * kMsNs);
    near(velocity, 0.0, "a stationary full window must clear the old speed");
    for (int ms = 76; ms <= 101; ++ms)
        velocity = estimator.update(20100 - (ms - 75) * 2, kStartNs + ms * kMsNs);
    near(velocity, -2000.0 * kCountRad, "a reverse full window must report the new sign");
}

void repeated_and_bad_timestamps_do_not_create_motion() {
    Joint2EncoderVelocityEstimator estimator;
    for (int ms = 0; ms <= 30; ++ms)
        estimator.update(20000 + 3 * ms, kStartNs + ms * kMsNs);
    const double expected = 3000.0 * kCountRad;
    near(estimator.update(1234, kStartNs + 30 * kMsNs), expected,
         "repeated snapshots must not change either the estimate or count history");
    near(estimator.update(20093, kStartNs + 31 * kMsNs), expected,
         "the sample after a duplicate must retain the original raw history");
    near(estimator.update(40000, kStartNs + 29 * kMsNs), 0.0,
         "out-of-order data must clear the estimate");
    for (int ms = 32; ms <= 52; ++ms)
        near(estimator.update(50000, kStartNs + ms * kMsNs), 0.0,
             "recovery must seed a new count history");
    for (const int64_t invalid_time : {int64_t{0}, int64_t{-1}})
        near(estimator.update(50000, invalid_time), 0.0, "invalid times must reset");
    near(estimator.update(-1, kStartNs), 0.0, "negative encoder values must reset");
    near(estimator.update(65536, kStartNs), 0.0, "out-of-range encoder values must reset");
}

void outages_and_explicit_reset_require_fresh_history() {
    Joint2EncoderVelocityEstimator estimator;
    for (int ms = 0; ms <= 30; ++ms)
        estimator.update(10000 + ms * 3, kStartNs + ms * kMsNs);
    near(estimator.update(60000, kStartNs + 130 * kMsNs), 0.0,
         "a 100 ms feedback outage must not unwrap unseen motion");
    for (int ms = 131; ms <= 150; ++ms)
        near(estimator.update(60000, kStartNs + ms * kMsNs), 0.0,
             "outage recovery must not reuse the pre-outage speed");
    estimator.reset();
    near(estimator.update(100, kStartNs + 151 * kMsNs), 0.0,
         "hardware freshness reset must discard accumulated turns");
    for (int ms = 152; ms <= 171; ++ms) {
        const double velocity = estimator.update(100 + 2 * (ms - 151), kStartNs + ms * kMsNs);
        near(velocity, ms < 171 ? 0.0 : 2000.0 * kCountRad,
             "fresh motion must recover only from its new window");
    }
}

} // namespace

int main() {
    try {
        stationary_quantization_noise_is_attenuated();
        constant_velocity_and_wrap_work_in_both_directions();
        uneven_sample_timing_uses_received_time();
        stopping_and_reversing_do_not_retain_old_motion();
        repeated_and_bad_timestamps_do_not_create_motion();
        outages_and_explicit_reset_require_fresh_history();
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << '\n';
        return 1;
    }
    std::cout << "joint2 encoder velocity tests passed\n";
    return 0;
}
