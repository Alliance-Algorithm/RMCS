#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>

namespace rmcs_core::controller::chassis {

struct WheelLegJointPair {
    double hip = 0.0;
    double knee = 0.0;
};

// S1 x [d_min, d_max]: only the common orientation is circular. The opening
// stays on the V5 assembly branch, without any encoder turn state.
struct WheelLegPairPose {
    double orientation = 0.0;
    double difference = 0.0;
};

class WheelLegJointPairGeometry {
public:
    enum class Side { kLeft, kRight };
    static constexpr double kPeriod = 2.0 * std::numbers::pi;
    // Exact V5 closure at 30/120 degrees, enclosing <3e-5 rad CAD axis rounding.
    static constexpr double kDefaultMinDifference = -0.47376;
    static constexpr double kDefaultMaxDifference = 1.62433;

    static double wrap(double angle) { return std::remainder(angle, kPeriod); }
    static double sign(Side side) { return side == Side::kLeft ? 1.0 : -1.0; }

    static WheelLegPairPose decode(Side side, double hip, double knee) {
        const double d = wrap(sign(side) * (hip - knee));
        return {wrap(hip - sign(side) * d / 2.0), d};
    }

    static std::optional<WheelLegPairPose> feedback(
        Side side, double hip, double knee, double min, double max, double tolerance) {
        if (!std::isfinite(hip) || !std::isfinite(knee))
            return std::nullopt;
        const auto pose = decode(side, hip, knee);
        if (pose.difference < min - tolerance || pose.difference > max + tolerance)
            return std::nullopt;
        return pose;
    }

    static WheelLegPairPose target(
        Side side, double hip, double knee, double min, double max, double margin) {
        auto pose = decode(side, hip, knee);
        pose.difference = std::clamp(pose.difference, min + margin, max - margin);
        return pose;
    }

    static WheelLegJointPair motors(Side side, const WheelLegPairPose& pose) {
        return {wrap(pose.orientation + sign(side) * pose.difference / 2.0),
                wrap(pose.orientation - sign(side) * pose.difference / 2.0)};
    }

    // Diagnostic interpolation; the servo uses the measured motor difference.
    // Left/right averaged V5 closure samples at h=+/-0.42, every five degrees.
    static double inner_angle_degrees(double difference) {
        constexpr std::array<double, 19> kDifference{
            -0.473731601, -0.297071061, -0.140638855, 0.001799635, 0.134193202,
            0.259151808, 0.378464328, 0.493395522, 0.604863832, 0.713551877,
            0.819977446, 0.924540437, 1.027554717, 1.129270232, 1.229888687,
            1.329574849, 1.428464858, 1.526672408, 1.624293425};
        const auto upper = std::upper_bound(kDifference.begin(), kDifference.end(), difference);
        const auto index = std::clamp<std::ptrdiff_t>(upper - kDifference.begin(), 1, 18);
        return 30.0 + 5.0 * (static_cast<double>(index - 1)
                            + (difference - kDifference[index - 1])
                                  / (kDifference[index] - kDifference[index - 1]));
    }
};

struct WheelLegPairVelocityConfig {
    double angle_kp = 6.0;
    double max_velocity = 2.0;
    double max_acceleration = 4.0;
    double min_difference = WheelLegJointPairGeometry::kDefaultMinDifference;
    double max_difference = WheelLegJointPairGeometry::kDefaultMaxDifference;
};

// Shared by the RMCS component and the PhysX harness. No position reference
// advances independently while the hardware cannot follow it.
inline WheelLegJointPair wheel_leg_pair_velocity(
    WheelLegJointPairGeometry::Side side, const WheelLegPairPose& measured,
    const WheelLegPairPose& target, const WheelLegJointPair& previous,
    const WheelLegPairVelocityConfig& config, double dt) {
    using Geometry = WheelLegJointPairGeometry;
    double error = Geometry::wrap(target.orientation - measured.orientation);
    const double previous_common = (previous.hip + previous.knee) / 2.0;
    // Retain the selected route near the antipode despite encoder quantization.
    if (std::abs(error) > std::numbers::pi - 0.1 && error * previous_common < 0.0)
        error += std::copysign(Geometry::kPeriod, previous_common);
    const double common = config.angle_kp * error;
    const double differential = config.angle_kp * (target.difference - measured.difference);
    const auto limit_speed = [&](WheelLegJointPair value) {
        const double scale = std::max(
            1.0, std::max(std::abs(value.hip), std::abs(value.knee)) / config.max_velocity);
        return WheelLegJointPair{value.hip / scale, value.knee / scale};
    };
    const auto desired = limit_speed({common + Geometry::sign(side) * differential / 2.0,
                                      common - Geometry::sign(side) * differential / 2.0});
    const double change =
        std::max(std::abs(desired.hip - previous.hip), std::abs(desired.knee - previous.knee));
    const double fraction = change > 0.0
                              ? std::min(1.0, config.max_acceleration * dt / change)
                              : 1.0;
    const WheelLegJointPair command{
        previous.hip + fraction * (desired.hip - previous.hip),
        previous.knee + fraction * (desired.knee - previous.knee)};

    // Stop outward motion at the mechanical boundary, before the normal ramp.
    const double shape_velocity = std::clamp(
        Geometry::sign(side) * (command.hip - command.knee),
        config.angle_kp * std::min(0.0, config.min_difference - measured.difference),
        config.angle_kp * std::max(0.0, config.max_difference - measured.difference));
    const double common_velocity = (command.hip + command.knee) / 2.0;
    return limit_speed({common_velocity + Geometry::sign(side) * shape_velocity / 2.0,
                        common_velocity - Geometry::sign(side) * shape_velocity / 2.0});
}

} // namespace rmcs_core::controller::chassis
