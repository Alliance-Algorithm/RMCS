#include <cmath>
#include <limits>
#include <stdexcept>

#include "controller/chassis/wheel_leg_joint_pair_geometry.hpp"

using namespace rmcs_core::controller::chassis;
using Geometry = WheelLegJointPairGeometry;

namespace {
void require(bool condition, const char* reason) {
    if (!condition)
        throw std::runtime_error(reason);
}
bool near(double a, double b, double tolerance = 1e-9) { return std::abs(a - b) < tolerance; }
} // namespace

int main() {
    constexpr auto left = Geometry::Side::kLeft;
    constexpr auto right = Geometry::Side::kRight;
    constexpr double lo = Geometry::kDefaultMinDifference;
    constexpr double hi = Geometry::kDefaultMaxDifference;
    constexpr double dt = 0.001;
    const WheelLegPairVelocityConfig config;

    // The same physical pose must produce the same command for any encoder
    // representation, including a reset/reconnect with no previous samples.
    const auto calibrated = Geometry::decode(left, -1.6, -2.93);
    require(near(calibrated.difference, 1.33), "calibrated opening");
    require(std::abs(Geometry::inner_angle_degrees(1.33) - 105.02) < 0.02, "calibration geometry");
    for (int h = -3; h <= 3; ++h)
        for (int k = -3; k <= 3; ++k) {
            const auto pose = Geometry::feedback(
                left, -1.6 + h * Geometry::kPeriod, -2.93 + k * Geometry::kPeriod, lo, hi, 0.03);
            require(pose.has_value(), "phase equivalence");
            require(near(pose->difference, calibrated.difference), "no relative turn state");
            require(
                near(Geometry::wrap(pose->orientation - calibrated.orientation), 0.0),
                "no common turn state");
        }
    require(!Geometry::feedback(left, 0.0, 2.0, lo, hi, 0.03), "reject wrong assembly");
    require(!Geometry::feedback(left, NAN, 0.0, lo, hi, 0.03), "reject invalid feedback");

    // Random absolute startup orientation, including every encoder seam. Both
    // legs must stay mirrored and within the physical differential at each step.
    for (int orientation = -16; orientation <= 16; ++orientation) {
        for (double initial_d : {lo, 0.0, 1.22989, 1.33, 1.42847, hi}) {
            for (WheelLegJointPair goal :
                 {WheelLegJointPair{0.0, 0.0}, WheelLegJointPair{-1.6, -2.93},
                  WheelLegJointPair{0.42, -0.13742282595254576}}) {
                auto pose = WheelLegPairPose{orientation * Geometry::kPeriod / 32.0, initial_d};
                const auto target = Geometry::target(left, goal.hip, goal.knee, lo, hi, 0.04);
                WheelLegJointPair command;
                for (int step = 0; step < 7000; ++step) {
                    const auto motors = Geometry::motors(left, pose);
                    const auto sensed = Geometry::decode(left, motors.hip, motors.knee);
                    const auto next =
                        wheel_leg_pair_velocity(left, sensed, target, command, config, dt);
                    const auto mirrored = wheel_leg_pair_velocity(
                        right, {-sensed.orientation, sensed.difference},
                        {-target.orientation, target.difference}, {-command.hip, -command.knee},
                        config, dt);
                    require(
                        near(mirrored.hip, -next.hip, 1e-8)
                            && near(mirrored.knee, -next.knee, 1e-8),
                        "mirror velocity");
                    require(
                        std::max(std::abs(next.hip), std::abs(next.knee)) <= 2.0 + 1e-9,
                        "velocity bound");
                    if (step == 0)
                        require(
                            std::max(std::abs(next.hip), std::abs(next.knee)) <= 0.004 + 1e-9,
                            "gentle start");
                    pose.orientation =
                        Geometry::wrap(pose.orientation + (next.hip + next.knee) * dt / 2.0);
                    pose.difference += (next.hip - next.knee) * dt;
                    require(
                        pose.difference >= lo - 1e-8 && pose.difference <= hi + 1e-8,
                        "path left assembly range");
                    command = next;
                }
                require(
                    std::abs(Geometry::wrap(pose.orientation - target.orientation)) < 0.002,
                    "orientation convergence");
                require(
                    std::abs(pose.difference - target.difference) < 0.002, "opening convergence");
            }
        }
    }

    // The old independent shortest-arc knee error points the wrong way here.
    const auto from = Geometry::decode(left, -2.1, -3.28);
    const auto to = Geometry::target(left, 0.42, -0.13742282595254576, lo, hi, 0.04);
    const auto move = wheel_leg_pair_velocity(left, from, to, {}, config, dt);
    require(move.hip > 0 && move.knee > 0, "coordinated long knee arc");
}
