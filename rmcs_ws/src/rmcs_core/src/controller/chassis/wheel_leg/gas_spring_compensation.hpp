#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "gas_spring_model.hpp"

namespace rmcs_core::controller::chassis {

struct GasSpringCompensation {
    bool valid = true;
    std::array<double, 4> torque{};
    std::array<double, 2> beta_deg{}, compression_m{}, force_n{}, weight{};
    std::array<bool, 2> extrapolated{};
};

// Inputs and efforts are in the four active model coordinates. The knee-drive
// angle is not the passive knee hinge angle. Calibration/reversal belongs to
// the caller: tau_api = J_motor_to_model^T * tau_model.
inline GasSpringCompensation gas_spring_compensation(const std::array<double, 4>& q, double gain) {
    GasSpringCompensation result;
    if (!std::ranges::all_of(q, [](double value) { return std::isfinite(value); })
        || !std::isfinite(gain) || gain < 0.0 || gain > 1.0) {
        result.valid = false;
        return result;
    }
    for (std::size_t side = 0; side < 2; ++side) {
        const auto& geometry = side == 0 ? gas_spring_model::kLeft : gas_spring_model::kRight;
        const double delta = side == 0 ? q[1] - q[0] : q[2] - q[3];
        if (delta < geometry.front().delta_rad || delta > geometry.back().delta_rad) {
            result.beta_deg[side] = std::numeric_limits<double>::quiet_NaN();
            continue; // Geometry covers the complete configured soft-limit interval.
        }
        auto upper = std::lower_bound(
            geometry.begin() + 1, geometry.end(), delta,
            [](const auto& knot, double value) { return knot.delta_rad < value; });
        upper = std::min(upper, geometry.end() - 1);
        const auto& high = *upper;
        const auto& low = *(upper - 1);
        const double u = (delta - low.delta_rad) / (high.delta_rad - low.delta_rad);
        const double beta = std::lerp(low.beta_deg, high.beta_deg, u);
        const double compression = std::lerp(low.compression_m, high.compression_m, u);
        const auto& curve = gas_spring_model::kForce;
        const double s =
            std::clamp(compression, curve.front().compression_m, curve.back().compression_m);
        auto force_upper =
            std::lower_bound(curve.begin() + 1, curve.end(), s, [](const auto& knot, double value) {
                return knot.compression_m < value;
            });
        force_upper = std::min(force_upper, curve.end() - 1);
        const auto& f_high = *force_upper;
        const auto& f_low = *(force_upper - 1);
        double force = std::lerp(
            f_low.force_n, f_high.force_n,
            (s - f_low.compression_m) / (f_high.compression_m - f_low.compression_m));
        // User-authorized trend extension keeps the measured section unchanged
        // and joins its endpoint slope with continuous positive axial force.
        const bool low_extension = compression < curve.front().compression_m;
        const bool high_extension = compression > curve.back().compression_m;
        if (low_extension)
            force = curve.front().force_n
                  * std::exp(
                        gas_spring_model::kLowCompressionRate
                        * (compression - curve.front().compression_m));
        if (high_extension)
            force = curve.back().force_n
                  + gas_spring_model::kHighCompressionSlope
                        * (compression - curve.back().compression_m);
        const double weight = gain;
        result.beta_deg[side] = beta;
        result.compression_m[side] = compression;
        result.force_n[side] = force;
        result.weight[side] = weight;
        result.extrapolated[side] = compression < curve.front().compression_m - 1e-8
                                 || compression > curve.back().compression_m + 1e-8;
        // Virtual work: the commanded holding effort opposes the axial
        // extension force, tau_hold = -(dl/dq)^T F. Friction is not included.
        result.torque[2 * side] =
            -weight * force * std::lerp(low.hip_length_jacobian, high.hip_length_jacobian, u);
        result.torque[2 * side + 1] =
            -weight * force * std::lerp(low.knee_length_jacobian, high.knee_length_jacobian, u);
    }
    return result;
}

} // namespace rmcs_core::controller::chassis
