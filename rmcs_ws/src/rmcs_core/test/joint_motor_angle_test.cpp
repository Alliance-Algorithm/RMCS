#include <array>
#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>

#include "hardware/joint_motor_angle.hpp"

namespace {

using rmcs_core::hardware::MultiTurnEncoderAngle;
constexpr double kRadiansPerCount = 2.0 * std::numbers::pi / 65536.0 / 42.0;

void near(double actual, double expected, const char* message) {
    if (std::abs(actual - expected) > 1e-10)
        throw std::runtime_error(std::string{message} + ": actual=" + std::to_string(actual)
                                 + ", expected=" + std::to_string(expected));
}

double angle_for(MultiTurnEncoderAngle& tracker, int encoder, int zero = 0) {
    return static_cast<double>(tracker.update(encoder, zero)) * kRadiansPerCount;
}

void encoder_rollover_is_continuous_in_both_directions() {
    MultiTurnEncoderAngle forward;
    near(angle_for(forward, 65530), -6.0 * kRadiansPerCount, "initial angle is zero-relative");
    near(angle_for(forward, 5), 5.0 * kRadiansPerCount, "forward 65535-to-zero rollover is continuous");

    MultiTurnEncoderAngle reverse;
    near(angle_for(reverse, 5), 5.0 * kRadiansPerCount, "reverse sequence initial angle");
    near(angle_for(reverse, 65530), -6.0 * kRadiansPerCount, "reverse zero-to-65535 rollover is continuous");
}

void joint_angle_accumulates_motor_revolutions() {
    MultiTurnEncoderAngle tracker;
    angle_for(tracker, 0);
    near(angle_for(tracker, 16384), std::numbers::pi / (2.0 * 42.0), "quarter motor turn");
    near(angle_for(tracker, 32768), std::numbers::pi / 42.0, "half motor turn");
    near(angle_for(tracker, 49152), 3.0 * std::numbers::pi / (2.0 * 42.0), "three-quarter motor turn");
    near(angle_for(tracker, 0), 2.0 * std::numbers::pi / 42.0, "full motor turn must not jump back");
}

void initial_offset_wraps_around_configured_zero_point() {
    MultiTurnEncoderAngle tracker;
    near(angle_for(tracker, 4, 65530), 10.0 * kRadiansPerCount,
         "initial encoder offset crossing zero point must choose the short direction");
    near(angle_for(tracker, 14, 65530), 20.0 * kRadiansPerCount,
         "subsequent counts must accumulate relative to the initial offset");
}

} // namespace

int main() {
    const std::array tests{
        std::pair{"encoder rollover is continuous in both directions", &encoder_rollover_is_continuous_in_both_directions},
        std::pair{"joint angle accumulates motor revolutions", &joint_angle_accumulates_motor_revolutions},
        std::pair{"initial offset wraps around configured zero point", &initial_offset_wraps_around_configured_zero_point}};
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
