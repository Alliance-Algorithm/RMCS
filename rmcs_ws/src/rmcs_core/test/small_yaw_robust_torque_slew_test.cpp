#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "hardware/small_yaw_damping.hpp"
#include "hardware/small_yaw_robust_torque_slew.hpp"

namespace {

using rmcs_core::hardware::SmallYawRobustTorqueSlew;
using rmcs_core::hardware::encode_small_yaw_damping_torque;

constexpr double kNmPerCount = 3.6 * 66.0 / 4096.0;
constexpr double kDt = 0.001;
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

void near(double actual, double expected, const char* message, double tolerance = 1e-10) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
        throw std::runtime_error(std::string{message} + ": actual=" + std::to_string(actual)
                                 + ", expected=" + std::to_string(expected));
}

double submitted_torque(double continuous_torque) {
    const auto packet = encode_small_yaw_damping_torque(continuous_torque, 10.0);
    const unsigned raw = std::to_integer<unsigned>(packet[4])
                       | (std::to_integer<unsigned>(packet[5]) << 8U);
    const int current = raw < 32768U ? static_cast<int>(raw) : static_cast<int>(raw) - 65536;
    return -current * kNmPerCount;
}

void ramp_timing_is_independent_of_can_send_period() {
    for (const double direction : {-1.0, 1.0}) {
        for (const double rate : {18.0, 60.0}) {
            for (const int send_period : {2, 3}) {
                for (int phase = 0; phase < send_period; ++phase) {
                    SmallYawRobustTorqueSlew slew;
                    double submitted = 0.0;
                    int last_send_tick = 0;
                    for (int tick = 1; tick <= 600; ++tick) {
                        const double torque = slew.update(direction * 4.5, rate * kDt, false);
                        const double expected = direction * std::min(4.5, rate * tick * kDt);
                        near(torque, expected, "continuous ramp must follow elapsed physical time");
                        if ((tick + phase) % send_period == 0) {
                            submitted = submitted_torque(torque);
                            last_send_tick = tick;
                            near(submitted, expected, "CAN rounding must stay within half a count",
                                 kNmPerCount / 2.0 + 1e-10);
                        }
                        const double held_expected = direction * std::min(4.5, rate * last_send_tick * kDt);
                        near(submitted, held_expected, "submitted telemetry must hold the last encoded value",
                             kNmPerCount / 2.0 + 1e-10);
                        if (rate == 60.0 && tick == 75)
                            near(torque, direction * 4.5, "60 Nm/s must reach 4.5 Nm in 75 ms");
                        if (rate == 60.0 && tick == 75 + send_period)
                            near(submitted, direction * 4.5, "CAN must reach target by the next send",
                                 kNmPerCount / 2.0 + 1e-10);
                    }
                }
            }
        }
    }
}

void subcount_steps_accumulate_without_stalling() {
    // Each update, and even three updates together, is below half a CAN count.
    // Feeding the encoded value back into the slew state would stall at zero.
    for (const double direction : {-1.0, 1.0}) {
        for (const int send_period : {2, 3}) {
            SmallYawRobustTorqueSlew slew;
            double submitted = 0.0;
            int first_nonzero_tick = 0;
            for (int tick = 1; tick <= 1002; ++tick) {
                const double torque = slew.update(direction, 0.001, false);
                if (tick % send_period == 0) {
                    submitted = submitted_torque(torque);
                    if (submitted != 0.0 && first_nonzero_tick == 0)
                        first_nonzero_tick = tick;
                }
            }
            require(first_nonzero_tick >= 29 && first_nonzero_tick <= 32,
                    "small steps must accumulate past one encoder rounding threshold");
            near(submitted, direction, "subcount ramp must eventually reach the target",
                 kNmPerCount / 2.0);
        }
    }
}

void withdrawal_and_reversal_preserve_existing_behavior() {
    for (const double direction : {-1.0, 1.0}) {
        SmallYawRobustTorqueSlew slew;
        for (int tick = 1; tick <= 200; ++tick) {
            const double torque = slew.update(direction * 10.0, 60.0 * kDt, false);
            require(std::abs(torque) <= 10.0, "continuous demand must not overshoot its 10 Nm target");
            require(std::abs(submitted_torque(torque)) <= 10.0,
                    "encoded 10 Nm demand must respect the motor command ceiling");
        }
        near(slew.update(direction, 0.0, false), direction,
             "reducing magnitude must withdraw torque immediately");
        near(slew.update(-direction * 5.0, 0.06, false), 0.0,
             "reversal must clear the previous sign before buildup");
        near(slew.update(-direction * 5.0, 0.06, false), -direction * 0.06,
             "reversed demand must restart with one continuous step");
        near(slew.update(-direction * 5.0, 0.0, true), 0.0,
             "safety loss must remove torque without waiting for a slew step");
        near(slew.update(direction * 5.0, 0.06, false), direction * 0.06,
             "rearming must not restore stale torque");
        near(slew.update(0.0, 0.0, false), 0.0, "zero demand must withdraw torque immediately");
        slew.update(direction * 5.0, 1.0, false);
        slew.reset();
        near(slew.update(direction * 5.0, 0.06, false), direction * 0.06,
             "explicit reset must clear continuous history");
    }
}

void invalid_inputs_clear_continuous_history() {
    for (const double invalid : {kNan, kInf, -kInf}) {
        SmallYawRobustTorqueSlew slew;
        slew.update(3.0, 3.0, false);
        near(slew.update(invalid, 0.06, false), 0.0, "nonfinite demand must fail to zero");
        near(slew.update(3.0, 0.06, false), 0.06, "invalid demand must discard old buildup");
        near(slew.update(3.0, invalid, false), 0.0, "nonfinite slew step must fail to zero");
        near(slew.update(3.0, 0.06, false), 0.06, "invalid slew step must discard old buildup");
        near(slew.update(invalid, invalid, true), 0.0, "hard stop must dominate invalid inputs");
    }
    SmallYawRobustTorqueSlew slew;
    slew.update(3.0, 3.0, false);
    near(slew.update(3.0, -0.01, false), 0.0, "negative slew step must fail to zero");
}

} // namespace

int main() {
    try {
        ramp_timing_is_independent_of_can_send_period();
        subcount_steps_accumulate_without_stalling();
        withdrawal_and_reversal_preserve_existing_behavior();
        invalid_inputs_clear_continuous_history();
        std::cout << "small yaw robust torque slew tests passed\n";
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << '\n';
        return 1;
    }
}
