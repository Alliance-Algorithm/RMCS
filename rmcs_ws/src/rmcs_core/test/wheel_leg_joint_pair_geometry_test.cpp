#include <cmath>
#include <stdexcept>

#include "controller/chassis/wheel_leg_joint_pair_geometry.hpp"

using rmcs_core::controller::chassis::WheelLegJointPairGeometry;

namespace {

void require(bool condition) {
    if (!condition)
        throw std::runtime_error("wheel-leg joint-pair geometry test failed");
}

bool near(double actual, double expected, double tolerance = 1e-9) {
    return std::abs(actual - expected) < tolerance;
}

} // namespace

int main() {
    using Geometry = WheelLegJointPairGeometry;
    constexpr auto left = Geometry::Side::kLeft;
    constexpr auto right = Geometry::Side::kRight;
    constexpr double min = Geometry::kDefaultMinDifference;
    constexpr double max = Geometry::kDefaultMaxDifference;
    constexpr double margin = 0.04;

    // Opposite single-turn encoder readings can describe one valid leg shape.
    const auto left_offset = Geometry::feedback_knee_offset(left, 3.1, -3.2, min, max, margin);
    const auto right_offset = Geometry::feedback_knee_offset(right, -3.1, 3.2, min, max, margin);
    require(left_offset && near(*left_offset, Geometry::kPeriod));
    require(right_offset && near(*right_offset, -Geometry::kPeriod));
    require(!Geometry::feedback_knee_offset(left, 2.0, 0.0, min, max, margin));

    // The safe-to-nominal transition must keep both motors on the same route.
    // Independent shortest-arc wrapping reverses the left knee at this pose.
    const auto left_nominal = Geometry::nearest_feasible_target(
        left, -2.1, -3.28, 0.42, -0.13742282595254576, min, max, margin);
    const auto right_nominal = Geometry::nearest_feasible_target(
        right, 2.1, 3.28, -0.42, 0.13741557625658019, min, max, margin);
    require(left_nominal && right_nominal);
    require(left_nominal->hip > -2.1 && left_nominal->knee > -3.28);
    require(right_nominal->hip < 2.1 && right_nominal->knee < 3.28);
    require(near(left_nominal->hip, 0.42));
    require(near(left_nominal->knee, -0.13742282595254576));

    // A common full turn changes neither leg shape nor the nearest target.
    const auto lifted = Geometry::nearest_feasible_target(
        left, 6.4, 5.8, 0.42, -0.13742282595254576, min, max, margin);
    require(lifted && near(lifted->hip, 0.42 + Geometry::kPeriod));
    require(near(lifted->knee, -0.13742282595254576 + Geometry::kPeriod));

    // An RL target outside the mechanical differential range is projected
    // inward while preserving the requested common rotation.
    const auto clamped =
        Geometry::nearest_feasible_target(left, 0.0, -0.5, 1.0, -1.0, min, max, margin);
    require(clamped && clamped->difference_clamped);
    require(near(Geometry::difference(left, clamped->hip, clamped->knee), max - margin));
    require(near((clamped->hip + clamped->knee) / 2.0, 0.0));

    // The controller interpolates both axes by the same fraction. Every
    // intermediate differential must remain inside the mechanical interval.
    for (int step = 0; step <= 100; ++step) {
        const double fraction = step / 100.0;
        const double hip = -2.1 + fraction * (left_nominal->hip + 2.1);
        const double knee = -3.28 + fraction * (left_nominal->knee + 3.28);
        const double difference = Geometry::difference(left, hip, knee);
        require(difference >= min + margin && difference <= max - margin);
    }
}
