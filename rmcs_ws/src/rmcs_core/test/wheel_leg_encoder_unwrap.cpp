#include "controller/chassis/wheel_leg/encoder_unwrap.hpp"
#include "controller/chassis/wheel_leg/joint_coordinates.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <numbers>

using namespace rmcs_core::controller::chassis;

namespace {

void require(bool condition, const char* message) {
    if (condition)
        return;
    std::cerr << message << '\n';
    std::exit(1);
}

bool close(double a, double b, double tolerance = 1e-9) { return std::abs(a - b) <= tolerance; }

} // namespace

int main() {
    constexpr double span = 2 * std::numbers::pi;

    // The current drive configuration uses P_MAX = pi, so the feedback span is
    // 2 pi. The raw angle crosses from the positive to the negative boundary.
    WheelLegEncoderUnwrap unwrap;
    require(unwrap.update(3.0, span), "first frame anchors");
    require(close(unwrap.value(), 3.0), "first value");
    require(!unwrap.update(3.1, span), "small step does not re-anchor");
    require(!unwrap.update(-3.1, span), "drive wrap does not re-anchor");
    require(close(unwrap.value(), 3.1 + span - 6.2), "drive wrap stays continuous");
    require(!unwrap.update(-3.0, span), "post-wrap step does not re-anchor");
    require(close(unwrap.value(), 3.2 + span - 6.2), "post-wrap angle stays continuous");

    // The helper must remain span-driven for a drive using a non-2-pi range.
    constexpr double nonstandard_span = 25.0;
    WheelLegEncoderUnwrap nonstandard;
    require(nonstandard.update(12.4, nonstandard_span), "nonstandard span anchors");
    require(!nonstandard.update(12.49, nonstandard_span), "nonstandard small step");
    require(!nonstandard.update(-12.49, nonstandard_span), "nonstandard wrap");
    require(close(nonstandard.value(), 12.51), "nonstandard span stays continuous");

    // A 2 pi assumption is wrong for the nonstandard span and leaves a residual.
    const double old_step = std::remainder(-12.5 - 12.5, 2 * std::numbers::pi);
    require(
        close(old_step, 8 * std::numbers::pi - 25.0, 1e-12),
        "2 pi assumption residual is 8 pi - 25");
    require(!close(old_step, 0.0, 1e-6), "2 pi assumption is not exact");

    // A step no joint can make between two 1 kHz frames re-anchors to the raw value.
    WheelLegEncoderUnwrap reanchor;
    require(reanchor.update(3.0, span), "first frame anchors before re-anchor test");
    require(reanchor.update(0.0, span), "impossible step re-anchors");
    require(close(reanchor.value(), 0.0), "re-anchored value is the raw value");
    require(!reanchor.update(std::numbers::pi / 4, span), "quarter turn is still a tracked step");

    // continuous_legs only picks the relative branch; it must not add 2 pi to the
    // absolute hip angle any more.
    const LegBounds bounds{{{-0.14, 1.43}, {-0.14, 1.43}}};
    const LegVector measured{12.4, 12.9, -12.4, -12.9};
    const auto continuous = continuous_legs(measured, bounds);
    require(close(continuous[0], 12.4), "left hip keeps its unwrapped value");
    require(close(continuous[1], 12.9), "left crank keeps its unwrapped value");
    require(close(continuous[2], -12.4), "right hip keeps its unwrapped value");
    require(close(continuous[3], -12.9), "right crank keeps its unwrapped value");

    std::cout << "wheel leg encoder unwrap passed\n";
    return 0;
}
