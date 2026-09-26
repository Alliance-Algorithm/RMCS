#include <cmath>
#include <numbers>
#include <stdexcept>

#include "controller/chassis/wheel_leg_joint_pair_geometry.hpp"
#include "hardware/device/continuous_angle_tracker.hpp"

using rmcs_core::controller::chassis::WheelLegJointPairGeometry;
using rmcs_core::hardware::device::ContinuousAngleTracker;

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

    // The recorded left-leg pose is outside the normal 110-degree range but
    // still on a unique branch that can be steered inward at reduced torque.
    constexpr double recovery_range = 0.25;
    const double logged_left_hip = -132.67 * std::numbers::pi / 180.0;
    const double logged_left_knee = -222.84 * std::numbers::pi / 180.0;
    const auto logged_offset = Geometry::feedback_knee_offset(
        left, logged_left_hip, logged_left_knee, min, max, recovery_range);
    require(logged_offset && near(*logged_offset, 0.0));
    const auto recovery_target = Geometry::nearest_feasible_target(
        left, logged_left_hip, logged_left_knee, logged_left_hip, logged_left_knee, min, max,
        margin);
    require(recovery_target && recovery_target->difference_clamped);
    require(near(Geometry::difference(left, recovery_target->hip, recovery_target->knee),
                 max - margin));
    require(near((recovery_target->hip + recovery_target->knee) / 2.0,
                 (logged_left_hip + logged_left_knee) / 2.0));
    for (int step = 0; step <= 100; ++step) {
        const double fraction = step / 100.0;
        const double hip = logged_left_hip
                         + fraction * (recovery_target->hip - logged_left_hip);
        const double knee = logged_left_knee
                          + fraction * (recovery_target->knee - logged_left_knee);
        const double difference = Geometry::difference(left, hip, knee);
        require(difference <= Geometry::difference(left, logged_left_hip, logged_left_knee));
        require(difference >= max - margin - 1e-9);
    }
    require(!Geometry::feedback_knee_offset(left, 2.0, 0.0, min, max, recovery_range));

    const auto right_recovery = Geometry::nearest_feasible_target(
        right, 0.0, max + 0.15, 0.0, max + 0.15, min, max, margin);
    require(right_recovery && right_recovery->difference_clamped);
    require(near(Geometry::difference(right, right_recovery->hip, right_recovery->knee),
                 max - margin));
    require(near((right_recovery->hip + right_recovery->knee) / 2.0,
                 (max + 0.15) / 2.0));

    const auto lower_recovery = Geometry::nearest_feasible_target(
        left, min - 0.15, 0.0, min - 0.15, 0.0, min, max, margin);
    require(lower_recovery && lower_recovery->difference_clamped);
    require(near(Geometry::difference(left, lower_recovery->hip, lower_recovery->knee),
                 min + margin));

    ContinuousAngleTracker hip_tracker;
    ContinuousAngleTracker knee_tracker;
    require(near(hip_tracker.update(3.0, Geometry::kPeriod), 3.0));
    require(near(knee_tracker.update(2.4, Geometry::kPeriod), 2.4));
    const double crossed_hip = hip_tracker.update(3.3 - Geometry::kPeriod, Geometry::kPeriod);
    const double crossed_knee = knee_tracker.update(2.7, Geometry::kPeriod);
    require(near(Geometry::difference(left, crossed_hip, crossed_knee), 0.6));

    for (int hip_turns = -3; hip_turns <= 3; ++hip_turns) {
        for (int knee_turns = -3; knee_turns <= 3; ++knee_turns) {
            const double hip = 0.4 + hip_turns * Geometry::kPeriod;
            const double knee = -0.2 + knee_turns * Geometry::kPeriod;
            const auto offset = Geometry::feedback_knee_offset(left, hip, knee, min, max, margin);
            require(offset.has_value());
            require(near(Geometry::difference(left, hip, knee + *offset), 0.6));
        }
    }

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
    require(Geometry::difference(left, 0.42, -0.13742282595254576 - Geometry::kPeriod) > max);

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
