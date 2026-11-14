#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <span>

#include "joint_coordinates.hpp"
#include "jump_v4.hpp"

namespace rmcs_core::controller::chassis::wheel_leg {

class JumpCommand {
public:
    enum class Phase { kIdle, kCrouch, kPush, kFlight, kRecover };

    explicit JumpCommand(double apex)
        : apex_(apex) {}

    void reset() {
        active_ = false;
        started_ = 0.;
    }

    bool request(double now) {
        if (active_)
            return false;
        started_ = now;
        active_ = true;
        return true;
    }

    void update(double now) {
        if (active_ && (now < started_ || now - started_ >= jump_v4::kRequestDuration))
            reset();
    }

    bool active() const { return active_; }
    double elapsed(double now) const { return active_ ? std::clamp(now - started_, 0., 6.) : 0.; }

    Phase phase(double now) const {
        if (!active_)
            return Phase::kIdle;
        const double t = elapsed(now);
        if (t < jump_v4::kCrouchSeconds)
            return Phase::kCrouch;
        if (t - jump_v4::kCrouchSeconds <= push_duration())
            return Phase::kPush;
        if (t - jump_v4::kCrouchSeconds < push_duration() + 2. * release_velocity() / 9.81)
            return Phase::kFlight;
        return Phase::kRecover;
    }

    double height(double now) const {
        const double t = elapsed(now);
        switch (phase(now)) {
        case Phase::kCrouch: {
            const double p = t / jump_v4::kCrouchSeconds;
            return jump_v4::kStandingHeight
                 + (jump_v4::kCrouchHeight - jump_v4::kStandingHeight)
                       * (3. * p * p - 2. * p * p * p);
        }
        case Phase::kPush: {
            const double p = t - jump_v4::kCrouchSeconds;
            return jump_v4::kCrouchHeight + .5 * release_velocity() / push_duration() * p * p;
        }
        case Phase::kFlight: return jump_v4::kReleaseHeight;
        default: return jump_v4::kStandingHeight;
        }
    }

    std::array<double, 7> context(double now) const {
        if (!active_)
            return {1., 0., 0., 0., 0., 0., 0.};
        return {0., 0., 0., 0., 1., apex_, elapsed(now)};
    }

private:
    double release_velocity() const {
        return std::sqrt(2. * 9.81 * (apex_ - jump_v4::kReleaseHeight));
    }
    double push_duration() const {
        return 2. * (jump_v4::kReleaseHeight - jump_v4::kCrouchHeight) / release_velocity();
    }

    double apex_, started_ = 0.;
    bool active_ = false;
};

inline LegBounds jump_target_bounds(bool active) {
    auto bounds = jump_v4::kBounds;
    if (active)
        for (auto& side : bounds) {
            side[0] -= jump_v4::kOvertravel;
            side[1] += jump_v4::kOvertravel;
        }
    return bounds;
}

inline LegVector
    tuck_torque(const LegVector& q, const LegVector& target, const LegVector& pd, bool flight) {
    LegVector torque{};
    if (!flight)
        return torque;

    const auto interpolate = []<typename Knot>(
                                 std::span<const Knot> knots, double x, double Knot::* key,
                                 double Knot::* value) {
        auto upper =
            std::lower_bound(knots.begin() + 1, knots.end(), x, [key](const Knot& knot, double x) {
                return knot.*key < x;
            });
        upper = std::min(upper, knots.end() - 1);
        const auto& hi = *upper;
        const auto& lo = *(upper - 1);
        return std::lerp(lo.*value, hi.*value, (x - lo.*key) / (hi.*key - lo.*key));
    };
    const auto smooth_ramp = [](double value) {
        const double x = std::clamp(value, 0., 1.);
        return x * x * (3. - 2. * x);
    };

    for (int side = 0; side < 2; ++side) {
        const int hip = 2 * side, knee = hip + 1;
        const double sign = side == 0 ? 1. : -1.;
        const double delta = sign * (q[knee] - q[hip]);
        const std::span<const jump_v4::GeometryKnot> geometry{
            side == 0 ? jump_v4::kLeft : jump_v4::kRight};
        if (delta < geometry.front().delta_rad || delta > geometry.back().delta_rad)
            continue;

        using Geometry = jump_v4::GeometryKnot;
        using Force = jump_v4::ForceKnot;
        const double compression =
            interpolate(geometry, delta, &Geometry::delta_rad, &Geometry::compression_m);
        const std::span<const Force> curve{jump_v4::kForce};
        double force = interpolate(curve, compression, &Force::compression_m, &Force::force_n);
        if (compression < curve.front().compression_m)
            force = curve.front().force_n
                  * std::exp(
                        jump_v4::kLowCompressionRate * (compression - curve.front().compression_m));
        else if (compression > curve.back().compression_m)
            force = curve.back().force_n
                  + jump_v4::kHighCompressionSlope * (compression - curve.back().compression_m);
        const double holding_hip =
            -force
            * interpolate(geometry, delta, &Geometry::delta_rad, &Geometry::hip_length_jacobian);
        const double holding_knee =
            -force
            * interpolate(geometry, delta, &Geometry::delta_rad, &Geometry::knee_length_jacobian);

        const double error = sign * ((q[knee] - q[hip]) - (target[knee] - target[hip]));
        const double inward_pd = -.5 * sign * (pd[knee] - pd[hip]);
        const double gate =
            smooth_ramp((error - jump_v4::kErrorOn) / (jump_v4::kErrorFull - jump_v4::kErrorOn))
            * smooth_ramp(inward_pd / jump_v4::kInwardPdFull);
        const double inward = std::max(0., -.5 * sign * (holding_knee - holding_hip));
        const double magnitude = std::min(inward * jump_v4::kTuckGain, jump_v4::kTuckCap) * gate;
        torque[hip] = sign * magnitude;
        torque[knee] = -sign * magnitude;
    }
    return torque;
}

} // namespace rmcs_core::controller::chassis::wheel_leg
