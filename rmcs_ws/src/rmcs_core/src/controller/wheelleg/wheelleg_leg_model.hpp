#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>

#include "controller/wheelleg/leg_transmission.hpp"

namespace rmcs_core::wheelleg {

// ===========================================================================
// Single source of truth for the V5.0 wheel-leg reduced model.
//
// The whole point: this is a PURE function of the two coaxial joint angles.
// Real hardware and simulation feed it through the SAME calibration and get
// identical (alpha, beta, leg-length) — there is no sim/real-specific maths.
//
//   raw joint angles (per side)                 --[calibration]-->
//   abstract reduced inputs                      --[forward F]-->
//   leg shape (alpha, beta, shank, leg length)   --[inverse F^-1]-->
//   raw joint targets
//
// Conventions (frozen from WheelLegBenchTestController, verified healthy on
// the real robot in mode 5):
//   alpha     reduced hip:  alpha = wrap_pi(q_hip/hip_sign + hip_zero)
//   c         relative crank (thigh frame): c = crank_sign*(q_crank - q_hip)
//   beta      interior knee (straight = pi), mechanically 40..105 deg
//   shank     phi = pi - beta - alpha
//   psi1      thigh K->H direction = beta + shank
//   K (knee)  = L1*(-cos psi1, -sin psi1)
//   A (axle)  = K + L2*(cos shank, sin shank)
//   alpha_L   hip->axle line angle to horizontal = atan2(A_z, A_x)
//   L0        hip->axle length = hypot(A_x, A_z)
//
// The crank<->knee map is the tabulated six-bar table in LegLinkage (no closed
// form). The table lives in the model/assembly frame; the DM encoder zero is
// the motor zero (knee 105 deg), so all encoder-frame access goes through
// LegLinkage::knee_from_encoder_crank / encoder_crank_from_knee.
// ===========================================================================

[[nodiscard]] inline double wrap_to_pi(double angle) {
    return std::remainder(angle, 2.0 * std::numbers::pi);
}

// Per-side calibration: the ONLY hardware-dependent part. Real robot and sim
// each own a set of these; the maths below is identical for both.
struct LegCalibration {
    double hip_sign = 1.0;
    double hip_zero = 0.0;
    double crank_sign = 1.0;
};

// Canonical calibration: the SINGLE zero calibration used by both the real
// robot and the simulation. Source of truth = wheel-leg-infantry-bench.yaml
// (measured on the real robot, mode 5 healthy).
//   - both hip encoders share one zero (thigh perpendicular to the chassis),
//     mirrors via hip_sign;
//   - the knee/crank encoders are zeroed at interior knee 105 deg (crank sign
//     mirrors left/right).
struct CanonicalCalibration {
    static constexpr double kHipSign[2] = {-1.0, 1.0};
    static constexpr double kHipZero[2] = {-1.5707963267948966, -1.5707963267948966};
    static constexpr double kCrankSign[2] = {-1.0, 1.0};
    [[nodiscard]] static LegCalibration side(int i) {
        return LegCalibration{kHipSign[i], kHipZero[i], kCrankSign[i]};
    }
};

// Forward result of the solver.
struct LegShape {
    double alpha = 0.0;      // reduced hip
    double crank_rel = 0.0;  // relative crank (encoder frame)
    double beta = 0.0;       // interior knee, clamped to the mechanical limits
    double beta_raw = 0.0;   // interior knee, unclamped (for diagnostics)
    double shank = 0.0;      // phi
    double axle_angle = 0.0; // hip->axle line angle to horizontal
    double leg_length = 0.0; // L0
    double knee_x = 0.0;     // knee position relative to the hip (fwd x, up z)
    double knee_z = 0.0;
    double axle_x = 0.0;     // axle (wheel) position relative to the hip
    double axle_z = 0.0;
};

class WheelLegLegModel {
public:
    WheelLegLegModel(
        double thigh_length, double shank_length, double knee_limit_min, double knee_limit_max,
        const LegLinkage& linkage)
        : thigh_length_(thigh_length)
        , shank_length_(shank_length)
        , knee_limit_min_(knee_limit_min)
        , knee_limit_max_(knee_limit_max)
        , linkage_(&linkage) {}

    // Forward F: raw joint angles -> reduced leg shape.
    [[nodiscard]] LegShape forward(
        double q_hip_raw, double q_crank_raw, const LegCalibration& calib) const {
        LegShape out;
        out.crank_rel = calib.crank_sign * (q_crank_raw - q_hip_raw);
        out.beta_raw = linkage_->knee_from_encoder_crank(out.crank_rel);
        out.beta = std::clamp(out.beta_raw, knee_limit_min_, knee_limit_max_);
        out.alpha = wrap_to_pi(q_hip_raw / calib.hip_sign + calib.hip_zero);
        out.shank = std::numbers::pi - out.beta - out.alpha;

        fk(out.beta, out.shank, out.knee_x, out.knee_z, out.axle_x, out.axle_z);
        out.axle_angle = std::atan2(out.axle_z, out.axle_x);
        out.leg_length = std::hypot(out.axle_x, out.axle_z);
        return out;
    }

    // Sagittal FK for a given interior knee beta and shank phi: knee and axle
    // positions relative to the hip (forward x, up z). Single implementation used
    // by forward() and the estimator's base-height / CoM model.
    void fk(
        double beta, double shank, double& knee_x, double& knee_z, double& axle_x,
        double& axle_z) const {
        const double psi1 = beta + shank;
        knee_x = -thigh_length_ * std::cos(psi1);
        knee_z = -thigh_length_ * std::sin(psi1);
        axle_x = knee_x + shank_length_ * std::cos(shank);
        axle_z = knee_z + shank_length_ * std::sin(shank);
    }

    // Inverse F^-1: reduced (alpha, beta) -> raw joint targets (single turn; the
    // caller aligns to the multi-turn encoder).
    bool joint_targets(
        double alpha, double beta, const LegCalibration& calib, double& q_hip_target,
        double& q_crank_target) const {
        double crank_rel = 0.0;
        if (!linkage_->encoder_crank_from_knee(
                std::clamp(beta, knee_limit_min_, knee_limit_max_), crank_rel))
            return false;
        q_hip_target = calib.hip_sign * (alpha - calib.hip_zero);
        q_crank_target = q_hip_target + calib.crank_sign * crank_rel;
        return true;
    }

    // Hip->axle line angle for a given (beta, alpha) without the full FK; the
    // plan/teleop layers use this to go from a knee target to a leg-angle target.
    [[nodiscard]] double axle_line(double beta, double alpha) const {
        return std::atan2(
                   shank_length_ * std::sin(beta), thigh_length_ - shank_length_ * std::cos(beta))
             - alpha;
    }

    [[nodiscard]] double knee_from_encoder_crank(double c_encoder) const {
        return linkage_->knee_from_encoder_crank(c_encoder);
    }

    // Interior knee <-> leg length (cosine law): the reduced-model relation the
    // trajectory / planning layer works in.
    [[nodiscard]] double leg_length_from_knee(double beta) const {
        return std::sqrt(
            thigh_length_ * thigh_length_ + shank_length_ * shank_length_
            - 2.0 * thigh_length_ * shank_length_ * std::cos(beta));
    }
    [[nodiscard]] double knee_from_leg_length(double leg_length) const {
        const double c = (thigh_length_ * thigh_length_ + shank_length_ * shank_length_
                          - leg_length * leg_length)
            / (2.0 * thigh_length_ * shank_length_);
        return std::acos(std::clamp(c, -1.0, 1.0));
    }

    [[nodiscard]] bool encoder_crank_from_knee(double beta, double& c_encoder) const {
        return linkage_->encoder_crank_from_knee(beta, c_encoder);
    }

    [[nodiscard]] double thigh_length() const { return thigh_length_; }
    [[nodiscard]] double shank_length() const { return shank_length_; }
    [[nodiscard]] double knee_limit_min() const { return knee_limit_min_; }
    [[nodiscard]] double knee_limit_max() const { return knee_limit_max_; }

private:
    double thigh_length_;
    double shank_length_;
    double knee_limit_min_;
    double knee_limit_max_;
    const LegLinkage* linkage_;
};

} // namespace rmcs_core::wheelleg
