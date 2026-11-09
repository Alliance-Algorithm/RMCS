#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <span>

#include "joint_coordinates.hpp"
#include "model/jump_v4.hpp"

namespace rmcs_core::controller::chassis::wheel_leg {

inline LegBounds jump_target_bounds(bool requested) {
    auto bounds = wheelbipe_jump_v4::kBounds;
    if (requested)
        for (auto& side : bounds) {
            side[0] -= wheelbipe_jump_v4::kOvertravel;
            side[1] += wheelbipe_jump_v4::kOvertravel;
        }
    return bounds;
}

inline LegVector jump_spring_holding(const LegVector& q) {
    LegVector holding{};
    for (std::size_t side = 0; side < 2; ++side) {
        const std::span<const gas_spring_model::GeometryKnot> geometry{
            side == 0 ? wheelbipe_jump_v4::kLeft : wheelbipe_jump_v4::kRight};
        const double delta = side == 0 ? q[1] - q[0] : q[2] - q[3];
        if (delta < geometry.front().delta_rad || delta > geometry.back().delta_rad)
            continue;
        auto upper = std::lower_bound(
            geometry.begin() + 1, geometry.end(), delta,
            [](const auto& knot, double value) { return knot.delta_rad < value; });
        upper = std::min(upper, geometry.end() - 1);
        const auto& hi = *upper;
        const auto& lo = *(upper - 1);
        const double blend = (delta - lo.delta_rad) / (hi.delta_rad - lo.delta_rad);
        const double compression = std::lerp(lo.compression_m, hi.compression_m, blend);
        const auto& curve = wheelbipe_jump_v4::kForce;
        auto force_upper = std::lower_bound(
            curve.begin() + 1, curve.end(), compression,
            [](const auto& knot, double value) { return knot.compression_m < value; });
        force_upper = std::min(force_upper, curve.end() - 1);
        const auto& fh = *force_upper;
        const auto& fl = *(force_upper - 1);
        double force = std::lerp(
            fl.force_n, fh.force_n,
            (compression - fl.compression_m) / (fh.compression_m - fl.compression_m));
        if (compression < curve.front().compression_m)
            force = curve.front().force_n
                  * std::exp(
                        wheelbipe_jump_v4::kLowCompressionRate
                        * (compression - curve.front().compression_m));
        else if (compression > curve.back().compression_m)
            force = curve.back().force_n
                  + wheelbipe_jump_v4::kHighCompressionSlope
                        * (compression - curve.back().compression_m);
        holding[2 * side] =
            -force * std::lerp(lo.hip_length_jacobian, hi.hip_length_jacobian, blend);
        holding[2 * side + 1] =
            -force * std::lerp(lo.knee_length_jacobian, hi.knee_length_jacobian, blend);
    }
    return holding;
}

struct JumpTuckEffort {
    LegVector torque{};
    std::array<double, 2> gate{};
};

inline double smooth_ramp(double value) {
    const double x = std::clamp(value, 0., 1.);
    return x * x * (3. - 2. * x);
}

inline JumpTuckEffort jump_tuck_effort(
    const LegVector& q, const LegVector& target, const LegVector& pd, bool flight_reference) {
    JumpTuckEffort result;
    if (!flight_reference)
        return result;
    const auto holding = jump_spring_holding(q);
    for (int side = 0; side < 2; ++side) {
        const int hip = 2 * side, aux = hip + 1;
        const double sign = side == 0 ? 1. : -1.;
        const double error = sign * ((q[aux] - q[hip]) - (target[aux] - target[hip]));
        const double inward_pd = -.5 * sign * (pd[aux] - pd[hip]);
        const double gate = smooth_ramp(
                                (error - wheelbipe_jump_v4::kErrorOn)
                                / (wheelbipe_jump_v4::kErrorFull - wheelbipe_jump_v4::kErrorOn))
                          * smooth_ramp(inward_pd / wheelbipe_jump_v4::kInwardPdFull);
        const double inward = std::max(0., -.5 * sign * (holding[aux] - holding[hip]));
        const double magnitude =
            std::min(inward * wheelbipe_jump_v4::kTuckGain, wheelbipe_jump_v4::kTuckCap) * gate;
        result.torque[hip] = sign * magnitude;
        result.torque[aux] = -sign * magnitude;
        result.gate[side] = gate;
    }
    return result;
}

} // namespace rmcs_core::controller::chassis::wheel_leg
