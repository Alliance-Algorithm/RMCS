#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>

#include "leg_transmission.hpp"

namespace rmcs_core::wheelleg {

// Nitrogen gas spring (BKB0.45-080-205) feedforward for the V5 leg.
//
// The spring acts along two pins (one on the thigh, one on the shank) so it
// loads the KNEE. Its force is
//     N(u) = c0 + c2*u^2 + c3*u^3  [N],   u = full_extension - pin_length,
// clamped to the stroke. The knee torque N * d(pin)/d(beta) is tabulated here
// from the model geometry as a function of the interior knee angle beta; the
// crank motor sees it scaled by d(beta)/d(q_crank) (the reduced linkage ratio).
//
// This is the same quantity the MuJoCo bridge computes in apply_gas_spring_()
// (verified: ~14.0 Nm at the standing 95 deg pose), so a controller can cancel
// the gas load with a feedforward instead of letting the leg PD fight it.
struct GasSpring {
    static constexpr double kFullExtension = 0.232;
    static constexpr double kStroke = 0.08;
    static constexpr double kC0 = 280.0;
    static constexpr double kC2 = 122.7359;
    static constexpr double kC3 = 53.9698;

    // Gas-spring torque about the knee [Nm] at the given interior knee angle
    // [rad]. Outside the tabulated 40..105 deg the endpoint value is held.
    [[nodiscard]] static double knee_torque(double beta_interior) {
        constexpr double kDeg = 180.0 / std::numbers::pi;
        const double deg = std::clamp(beta_interior * kDeg, kBetaDeg.front(), kBetaDeg.back());
        const std::size_t upper = static_cast<std::size_t>(
            std::lower_bound(kBetaDeg.begin(), kBetaDeg.end(), deg) - kBetaDeg.begin());
        if (upper == 0)
            return kTau.front();
        if (upper >= kBetaDeg.size())
            return kTau.back();
        const double t = (deg - kBetaDeg[upper - 1]) / (kBetaDeg[upper] - kBetaDeg[upper - 1]);
        return kTau[upper - 1] + t * (kTau[upper] - kTau[upper - 1]);
    }

    // Incremental gas-spring feedforward [Nm], relative to a reference pose.
    //    tau_ff = -(crank_torque(beta_meas) - crank_torque(beta_ref))
    // At beta_meas == beta_ref this is zero, so it does NOT move the steady
    // operating point (the spring is load-bearing); it only cancels the spring's
    // change while the leg moves away from the reference. Multiply by a gain.
    [[nodiscard]] static double crank_delta_torque(
        double beta_meas, double beta_ref, const LegLinkage& linkage) {
        return crank_torque(beta_meas, linkage) - crank_torque(beta_ref, linkage);
    }

    // Gas-spring torque on the CRANK [Nm]: knee torque times the reduced
    // linkage ratio d(beta)/d(q_crank). Add the NEGATIVE of this as feedforward
    // to cancel the spring.
    [[nodiscard]] static double crank_torque(double beta_interior, const LegLinkage& linkage) {
        const double beta = std::clamp(
            beta_interior, linkage.knee_min(), linkage.knee_max());
        constexpr double kDelta = 1e-3;
        double cp = 0.0;
        double cm = 0.0;
        double d_beta_d_crank = 0.0;
        if (linkage.crank_from_knee(beta + kDelta, cp)
            && linkage.crank_from_knee(beta - kDelta, cm)
            && std::abs(cp - cm) > 1e-9)
            d_beta_d_crank = 2.0 * kDelta / (cp - cm);
        return knee_torque(beta) * d_beta_d_crank;
    }

private:
    // Interior knee angle [deg] and the corresponding gas-spring knee torque
    // [Nm], sampled from the model (pin anchors thigh (0.01559,-0.0486,-0.002)
    // / shank (0.05075,0.005,-0.002), axis-free magnitude).
    static constexpr std::array<double, 14> kBetaDeg = {
        40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0,
        75.0, 80.0, 85.0, 90.0, 95.0, 100.0, 105.0};
    static constexpr std::array<double, 14> kTau = {
        10.01, 10.52, 11.47, 12.28, 12.95, 13.47, 13.86,
        14.12, 14.26, 14.29, 14.21, 14.03, 13.75, 13.59};
};

} // namespace rmcs_core::wheelleg
