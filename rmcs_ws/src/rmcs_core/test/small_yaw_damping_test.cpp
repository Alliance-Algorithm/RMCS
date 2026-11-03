#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>

#include "hardware/small_yaw_damping.hpp"

namespace {

using rmcs_core::hardware::SmallYawDamping;
using rmcs_core::hardware::SmallYawDampingParameters;
using rmcs_core::hardware::SmallYawMotionEstimator;
using rmcs_core::hardware::encode_small_yaw_damping_torque;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr std::array kInvalid{kNan, kInf, -kInf};
constexpr double kEncoderStep = 2.0 * std::numbers::pi / 65536.0;

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

void near(double actual, double expected, const char* message, double tolerance = 1e-10) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
        throw std::runtime_error(std::string{message} + ": actual=" + std::to_string(actual)
                                 + ", expected=" + std::to_string(expected));
}

template <typename Callable>
void rejects(Callable&& callable, const char* message) {
    try {
        std::forward<Callable>(callable)();
    } catch (const std::invalid_argument&) {
        return;
    }
    throw std::runtime_error(message);
}

SmallYawDampingParameters enabled_parameters() {
    SmallYawDampingParameters parameters;
    parameters.enabled = true;
    // Keep the existing threshold boundary tests independent of the slow-drift defaults.
    parameters.start_velocity_rad_s = 0.05;
    parameters.stop_velocity_rad_s = 0.02;
    return parameters;
}

SmallYawDampingParameters slow_drift_parameters() {
    SmallYawDampingParameters parameters;
    parameters.enabled = true;
    return parameters;
}

int decoded_current(const std::array<std::byte, 8>& command) {
    const unsigned raw = std::to_integer<unsigned>(command[4])
                       | (std::to_integer<unsigned>(command[5]) << 8U);
    return raw < 32768U ? static_cast<int>(raw) : static_cast<int>(raw) - 65536;
}

void both_directions_build_braking_torque() {
    for (const double direction : {-1.0, 1.0}) {
        SmallYawDamping damping{enabled_parameters()};
        for (int cycle = 1; cycle <= 138; ++cycle) {
            const double torque = damping.update(direction * 0.3, true, 0.001);
            near(torque, -direction * (cycle * 0.018), "braking must build at 18 Nm/s");
            require(torque * direction < 0.0, "braking torque must oppose physical velocity");
        }
        near(damping.update(direction * 0.3, true, 0.02), -direction * 2.5,
             "braking must saturate at 2.5 Nm");
    }
}

void thresholds_filter_noise_without_holding_at_rest() {
    for (const double direction : {-1.0, 1.0}) {
        SmallYawDamping damping{enabled_parameters()};
        for (const double speed : {0.0, 0.019, 0.02, 0.03, 0.049})
            near(damping.update(direction * speed, true, 0.01), 0.0,
                 "subthreshold noise must not activate braking");
        near(damping.update(direction * 0.05, true, 0.01), -direction * 0.18,
             "start threshold is inclusive");
        near(damping.update(direction * 0.03, true, 0.01), -direction * 0.36,
             "active braking must persist inside the hysteresis band");
        near(damping.update(direction * 0.02, true, 0.001), 0.0,
             "stop threshold must clear torque immediately");
        for (const double speed : {0.03, 0.049, 0.021, 0.0})
            near(damping.update(direction * speed, true, 0.01), 0.0,
                 "stopped brake must wait for the start threshold again");
    }
}

void reversals_withdraw_old_torque_and_restart_from_zero() {
    for (const double direction : {-1.0, 1.0}) {
        SmallYawDamping damping{enabled_parameters()};
        for (int cycle = 0; cycle < 10; ++cycle)
            damping.update(direction * 0.2, true, 0.02);
        near(damping.update(-direction * 0.03, true, 0.001), 0.0,
             "reversal in the hysteresis band must immediately clear old torque");
        near(damping.update(-direction * 0.049, true, 0.001), 0.0,
             "new direction must independently reach the start threshold");
        near(damping.update(-direction * 0.05, true, 0.001), direction * 0.018,
             "new direction must ramp from zero");
        near(damping.update(direction * 0.5, true, 0.001), -direction * 0.018,
             "direct high-speed reversal must discard old torque before ramping");
    }
}

void permission_loss_and_reset_clear_activation() {
    SmallYawDamping damping{enabled_parameters()};
    for (int cycle = 0; cycle < 10; ++cycle)
        damping.update(0.3, true, 0.02);
    near(damping.update(0.3, false, 0.001), 0.0, "loss of permission must zero full torque");
    near(damping.update(0.03, true, 0.01), 0.0, "repermission must not restore a stale latch");
    near(damping.update(0.3, true, 0.001), -0.018, "repermission must ramp from zero");
    damping.reset();
    near(damping.update(0.03, true, 0.01), 0.0, "explicit reset must clear activation");
    near(damping.update(-0.3, true, 0.001), 0.018, "reset must also clear torque history");
    SmallYawDamping disabled{SmallYawDampingParameters{}};
    for (const double speed : {-100.0, -0.05, 0.0, 0.05, 100.0})
        near(disabled.update(speed, true, 0.02), 0.0, "braking is disabled by default");
}

void invalid_feedback_and_time_fail_closed() {
    for (const double invalid : kInvalid) {
        for (const bool invalid_velocity : {false, true}) {
            SmallYawDamping damping{enabled_parameters()};
            damping.update(0.3, true, 0.02);
            near(damping.update(invalid_velocity ? invalid : 0.3, true,
                                invalid_velocity ? 0.001 : invalid),
                 0.0, "nonfinite speed or dt must immediately clear torque");
            near(damping.update(0.03, true, 0.001), 0.0,
                 "invalid inputs must also clear the activation latch");
            near(damping.update(0.3, true, 0.001), -0.018,
                 "recovery after invalid input must ramp from zero");
        }
    }
    for (const double invalid_dt : {0.0, -0.01}) {
        SmallYawDamping damping{enabled_parameters()};
        damping.update(-0.3, true, 0.02);
        near(damping.update(-0.3, true, invalid_dt), 0.0, "nonpositive dt must zero torque");
        near(damping.update(-0.03, true, 0.001), 0.0, "invalid dt must reset activation");
    }
    SmallYawDamping damping{enabled_parameters()};
    near(damping.update(0.3, true, 100.0), -0.36,
         "a scheduling gap must limit the build step to 20 ms");
}

void parameters_are_validated() {
    using Parameters = SmallYawDampingParameters;
    constexpr std::array fields{&Parameters::torque_nm, &Parameters::start_velocity_rad_s,
                                &Parameters::stop_velocity_rad_s, &Parameters::torque_slew_nm_s};
    for (const auto field : fields) {
        for (const double invalid : kInvalid) {
            auto parameters = enabled_parameters();
            parameters.*field = invalid;
            rejects([&] { const SmallYawDamping damping{parameters}; },
                    "nonfinite configuration must be rejected");
        }
    }
    for (const double torque : {-1.0, 0.0, 2.50001}) {
        auto parameters = enabled_parameters();
        parameters.torque_nm = torque;
        rejects([&] { const SmallYawDamping damping{parameters}; }, "invalid torque limit");
    }
    for (const double slew : {-1.0, 0.0}) {
        auto parameters = enabled_parameters();
        parameters.torque_slew_nm_s = slew;
        rejects([&] { const SmallYawDamping damping{parameters}; }, "invalid torque slew");
    }
    for (const auto& [start, stop] : {std::pair{0.0, 0.0}, std::pair{0.02, 0.02},
                                    std::pair{0.01, 0.02}, std::pair{0.05, -0.01}}) {
        auto parameters = enabled_parameters();
        parameters.start_velocity_rad_s = start;
        parameters.stop_velocity_rad_s = stop;
        rejects([&] { const SmallYawDamping damping{parameters}; }, "invalid hysteresis");
    }
    auto parameters = enabled_parameters();
    parameters.torque_nm = 0.7;
    parameters.stop_velocity_rad_s = 0.0;
    parameters.torque_slew_nm_s = 100.0;
    SmallYawDamping damping{parameters};
    near(damping.update(0.05, true, 0.02), -0.7, "valid configured torque must be honored");
    near(damping.update(0.0, true, 0.02), 0.0, "zero stop threshold must still stop at rest");
}

void reversed_lk_encoding_is_bounded_and_little_endian() {
    const auto positive = encode_small_yaw_damping_torque(2.5);
    const auto negative = encode_small_yaw_damping_torque(-2.5);
    require(decoded_current(positive) == -43, "+2.5 Nm must encode reverse raw current -43");
    require(decoded_current(negative) == 43, "-2.5 Nm must encode reverse raw current +43");
    near(-decoded_current(positive) * (3.6 * 66.0 / 4096.0), 2.4943359375,
         "full positive damping must encode the expected quantized physical Nm");
    near(-decoded_current(negative) * (3.6 * 66.0 / 4096.0), -2.4943359375,
         "full negative damping must encode the expected quantized physical Nm");
    require(positive[4] == std::byte{0xD5} && positive[5] == std::byte{0xFF},
            "negative raw current must be two's-complement little endian");
    require(negative[4] == std::byte{0x2B} && negative[5] == std::byte{0x00},
            "positive raw current must be little endian");
    const auto max_positive = encode_small_yaw_damping_torque(6.0);
    const auto max_negative = encode_small_yaw_damping_torque(-6.0);
    require(decoded_current(max_positive) == -103, "+6 Nm identification sine must encode current -103");
    require(decoded_current(max_negative) == 103, "-6 Nm identification sine must encode current +103");
    near(-decoded_current(max_positive) * (3.6 * 66.0 / 4096.0), 5.9748046875,
         "6 Nm sine must preserve the expected quantized physical torque");
    for (const double torque : {-100.0, -6.0, -4.5, -4.0, -2.5, -0.1, 0.0, 0.1, 2.5, 4.0, 4.5, 6.0, 100.0, kNan, kInf, -kInf}) {
        const auto command = encode_small_yaw_damping_torque(torque);
        require(command[0] == std::byte{0xA1}, "LK torque command opcode");
        for (const int index : {1, 2, 3, 6, 7})
            require(command[index] == std::byte{0}, "unused command bytes must be zero");
        require(std::abs(decoded_current(command)) <= 103, "encoding must bound raw current");
        if (!std::isfinite(torque))
            require(decoded_current(command) == 0, "invalid torque must encode zero current");
    }
    require(decoded_current(encode_small_yaw_damping_torque(100.0)) == -103,
            "finite overrange torque must be clamped");
    require(decoded_current(encode_small_yaw_damping_torque(-100.0)) == 103,
            "negative overrange torque must be clamped symmetrically");

    // Explicit profile limits must reach the CAN frame; falling back to the
    // legacy 6 Nm default would silently defeat the 7 Nm robust-control trial.
    for (const double direction : {-1.0, 1.0}) {
        const auto command = encode_small_yaw_damping_torque(direction * 7.0, 7.0);
        require(decoded_current(command) == -direction * 121,
                "an explicit 7 Nm limit must encode 121 current counts in both directions");
        near(-decoded_current(command) * (3.6 * 66.0 / 4096.0), direction * 7.0189453125,
             "7 Nm must reach the motor within half a current quantization step");
        require(decoded_current(encode_small_yaw_damping_torque(direction * 100.0, 7.0))
                    == decoded_current(command),
                "an explicit 7 Nm profile must still clamp overrange torque");
        require(decoded_current(encode_small_yaw_damping_torque(direction * 7.0, 4.0))
                    == -direction * 69,
                "the encoder must continue to respect a lower configured profile limit");

        const auto ten_nm = encode_small_yaw_damping_torque(direction * 10.0, 10.0);
        require(decoded_current(ten_nm) == -direction * 172,
                "an explicit 10 Nm robust limit must reach the CAN frame in both directions");
        near(-decoded_current(ten_nm) * (3.6 * 66.0 / 4096.0), direction * 9.97734375,
             "10 Nm must encode within half a current quantization step");
        require(decoded_current(encode_small_yaw_damping_torque(direction * 100.0, 10.0))
                    == decoded_current(ten_nm),
                "the 10 Nm robust limit must still clamp overrange torque");
    }
}

void stateful_braking_never_adds_energy() {
    SmallYawDamping damping{enabled_parameters()};
    std::mt19937 random{0x5A11U};
    std::uniform_real_distribution<double> speeds{-3.0, 3.0};
    std::uniform_real_distribution<double> times{0.0001, 0.1};
    constexpr double torque_per_raw_current = 3.6 * 66.0 / 4096.0;
    for (int cycle = 0; cycle < 20000; ++cycle) {
        // Mix abrupt reversals with correlated deceleration through both thresholds.
        const double speed = cycle % 100 < 50 ? speeds(random)
                            : 0.06 * std::sin(cycle * 0.05);
        const bool permitted = cycle % 137 != 0;
        const double torque = damping.update(speed, permitted, times(random));
        require(std::isfinite(torque) && std::abs(torque) <= 2.5,
                "all state transitions must preserve finite bounded torque");
        require(torque * speed <= 0.0, "software brake must never add rotational energy");
        const double quantized = -decoded_current(encode_small_yaw_damping_torque(torque))
                               * torque_per_raw_current;
        require(std::abs(quantized) <= 2.5 && quantized * speed <= 0.0,
                "encoded reverse-motor command must also remain bounded and dissipative");
        if (!permitted || std::abs(speed) <= 0.02)
            near(torque, 0.0, "permission loss and low speed must immediately remove torque");
    }
}

void encoder_staircase_detects_native_zero_speed_drift() {
    for (const double direction : {-1.0, 1.0}) {
        SmallYawMotionEstimator estimator;
        SmallYawDamping damping{slow_drift_parameters()};
        double torque = 0.0;
        // A 1 kHz control loop sees the same 100 Hz encoder angle ten times.
        // This is ~0.0096 rad/s even when the native motor velocity remains zero.
        for (int cycle = 0; cycle <= 250; ++cycle) {
            const double angle = direction * (cycle / 10) * kEncoderStep;
            const double velocity = estimator.update(angle, 0.001);
            torque = damping.update(velocity, true, 0.001);
            require(torque * direction <= 0.0, "staircase drift must receive opposing torque");
            if (cycle >= 50)
                near(velocity, direction * 4.0 * kEncoderStep / 0.04,
                     "40 ms estimate must remain within one encoder count across frames",
                     kEncoderStep / 0.04 + 1e-8);
        }
        near(torque, -direction * 2.5, "slow drift must build full braking by 250 ms");

        const double stopped_angle = direction * 25.0 * kEncoderStep;
        for (int cycle = 1; cycle <= 40; ++cycle) {
            const double velocity = estimator.update(stopped_angle, 0.001);
            torque = damping.update(velocity, true, 0.001);
            require(torque * direction <= 0.0, "stopping estimate must not reverse braking");
        }
        near(torque, 0.0, "stopping must withdraw torque within the 40 ms window");
        for (int cycle = 0; cycle < 100; ++cycle)
            near(damping.update(estimator.update(stopped_angle, 0.001), true, 0.001), 0.0,
                 "standstill must not re-enable torque");
    }
}

void encoder_continuous_slow_drift_and_variable_dt() {
    for (const double direction : {-1.0, 1.0}) {
        SmallYawMotionEstimator estimator;
        double time = 0.0;
        for (int cycle = 0; cycle < 1000; ++cycle) {
            const double dt = cycle % 3 == 0 ? 0.002 : 0.0005;
            time += dt;
            const double velocity = estimator.update(direction * 0.01 * time, dt);
            if (time > 0.05)
                near(velocity, direction * 0.01,
                     "time-based window must estimate 0.01 rad/s at varying loop rates", 1e-8);
        }
    }
}

void encoder_abrupt_stop_clears_all_sampling_phases() {
    for (int phase = 0; phase < 5; ++phase) {
        SmallYawMotionEstimator estimator;
        SmallYawDamping damping{slow_drift_parameters()};
        double angle = 0.0;
        for (int cycle = 0; cycle <= 200 + phase; ++cycle) {
            angle = cycle * 0.0003;
            damping.update(estimator.update(angle, 0.001), true, 0.001);
        }
        double torque = 0.0;
        for (int cycle = 0; cycle < 45; ++cycle)
            torque = damping.update(estimator.update(angle, 0.001), true, 0.001);
        // The oldest endpoint may straddle the stop by one 5 ms sample interval.
        near(torque, 0.0, "abrupt stop must clear by 40 ms plus one sampling interval");
    }
}

void encoder_reversal_immediately_withdraws_reliable_old_direction() {
    for (const double direction : {-1.0, 1.0}) {
        SmallYawMotionEstimator estimator;
        SmallYawDamping damping{slow_drift_parameters()};
        double angle = 0.0;
        for (int cycle = 0; cycle <= 250; ++cycle) {
            angle = direction * (cycle / 10) * kEncoderStep;
            damping.update(estimator.update(angle, 0.001), true, 0.001);
        }
        const double extreme = angle;
        const double first_return = estimator.update(extreme - direction * kEncoderStep, 0.001);
        require(first_return * direction >= 0.0,
                "one opposite count is within the observed encoder uncertainty");
        const double reliable_reverse = estimator.update(extreme - direction * 2 * kEncoderStep, 0.001);
        near(reliable_reverse, 0.0,
             "two opposite counts must clear the old direction without waiting 40 ms");
        near(damping.update(reliable_reverse, true, 0.001), 0.0,
             "reliable encoder reversal must immediately withdraw old torque");
        double torque = 0.0;
        for (int cycle = 1; cycle <= 250; ++cycle) {
            angle = extreme - direction * (2 + cycle / 10) * kEncoderStep;
            const double velocity = estimator.update(angle, 0.001);
            torque = damping.update(velocity, true, 0.001);
            require(torque * direction >= 0.0,
                    "old torque sign must not return while reverse history builds");
        }
        near(torque, direction * 2.5, "sustained reversed drift must build full opposite braking");
    }
}

void encoder_angle_wrap_is_continuous() {
    for (const double direction : {-1.0, 1.0}) {
        SmallYawMotionEstimator estimator;
        for (int cycle = 0; cycle <= 250; ++cycle) {
            const double unwrapped = direction * (std::numbers::pi - 0.0005 + cycle * 0.00001);
            const double wrapped = std::remainder(unwrapped, 2.0 * std::numbers::pi);
            const double velocity = estimator.update(wrapped, 0.001);
            if (cycle >= 40)
                near(velocity, direction * 0.01, "crossing +/- pi must not cause a velocity spike", 1e-8);
        }
    }
}

void encoder_one_count_idle_jitter_never_activates_braking() {
    SmallYawMotionEstimator estimator;
    SmallYawDamping damping{slow_drift_parameters()};
    std::mt19937 random{0x15164U};
    for (int cycle = 0; cycle < 10000; ++cycle) {
        // Reproduce the two adjacent raw values seen with a stationary motor.
        const double angle = (15163 + (random() % 2)) * kEncoderStep;
        const double velocity = estimator.update(angle, 0.001);
        near(velocity, 0.0, "one-count idle jitter must not be reported as motion");
        near(damping.update(velocity, true, 0.001), 0.0,
             "one-count idle jitter must not activate or latch braking");
    }
}

void encoder_invalid_data_and_gaps_reset_history() {
    const auto prime = [](SmallYawMotionEstimator& estimator) {
        for (int cycle = 0; cycle <= 50; ++cycle)
            estimator.update(cycle * 0.00001, 0.001);
    };
    for (const double invalid : kInvalid) {
        for (const bool invalid_angle : {false, true}) {
            SmallYawMotionEstimator estimator;
            prime(estimator);
            near(estimator.update(invalid_angle ? invalid : 0.0005,
                                  invalid_angle ? 0.001 : invalid),
                 0.0, "nonfinite encoder angle and dt must clear motion estimate");
            near(estimator.update(0.5, 0.001), 0.0,
                 "first angle after invalid input must seed fresh history");
            for (int cycle = 1; cycle <= 50; ++cycle)
                near(estimator.update(0.5, 0.001), 0.0,
                     "old motion must not reappear after invalid input");
        }
    }
    for (const double invalid_dt : {-0.001, 0.0, 0.020001, 1.0}) {
        SmallYawMotionEstimator estimator;
        prime(estimator);
        near(estimator.update(0.5, invalid_dt), 0.0,
             "nonpositive dt and gaps beyond 20 ms must fail closed");
        near(estimator.update(0.8, 0.001), 0.0, "gap recovery must begin a fresh window");
        near(estimator.update(0.8, 0.02), 0.0, "20 ms boundary is valid but window still warms up");
        near(estimator.update(0.8, 0.02), 0.0, "stationary recovery must stay at zero");
    }
    SmallYawMotionEstimator estimator;
    prime(estimator);
    estimator.reset();
    near(estimator.update(-0.5, 0.001), 0.0, "explicit reset must discard old angle and velocity");
}

} // namespace

int main() {
    const std::array tests{
        std::pair{"both directions build braking torque", &both_directions_build_braking_torque},
        std::pair{"hysteresis filters noise and stops at rest", &thresholds_filter_noise_without_holding_at_rest},
        std::pair{"reversals cannot propel the new motion", &reversals_withdraw_old_torque_and_restart_from_zero},
        std::pair{"permission loss and reset clear activation", &permission_loss_and_reset_clear_activation},
        std::pair{"invalid feedback and dt fail closed", &invalid_feedback_and_time_fail_closed},
        std::pair{"configuration is validated", &parameters_are_validated},
        std::pair{"reversed LK current encoding", &reversed_lk_encoding_is_bounded_and_little_endian},
        std::pair{"stateful software and wire commands cannot add energy", &stateful_braking_never_adds_energy},
        std::pair{"encoder staircase drift builds torque and stops within one window", &encoder_staircase_detects_native_zero_speed_drift},
        std::pair{"encoder time window supports varying scheduler intervals", &encoder_continuous_slow_drift_and_variable_dt},
        std::pair{"abrupt encoder stop clears every sampling phase", &encoder_abrupt_stop_clears_all_sampling_phases},
        std::pair{"encoder reversal clears the old direction immediately", &encoder_reversal_immediately_withdraws_reliable_old_direction},
        std::pair{"encoder angle wraps remain continuous", &encoder_angle_wrap_is_continuous},
        std::pair{"one-count idle jitter cannot activate braking", &encoder_one_count_idle_jitter_never_activates_braking},
        std::pair{"encoder invalid data and scheduling gaps clear history", &encoder_invalid_data_and_gaps_reset_history}};
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try {
            test();
            std::cout << "PASS: " << name << '\n';
        } catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL: " << name << ": " << error.what() << '\n';
        }
    }
    std::cout << tests.size() - failures << '/' << tests.size() << " cases passed\n";
    return failures == 0 ? 0 : 1;
}
