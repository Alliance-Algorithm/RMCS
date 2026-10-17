#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <numbers>

namespace rmcs_core::wheelleg {

// V5.0 wheel-leg transmission.
//
// The V5.0 leg is a Stephenson six-bar: two coaxial hip motors drive the thigh
// (``L_joint1``/``R_joint1``) and the crank (``L_crank``/``R_crank``); the crank
// drives the shank through ``LL_joint2 -> LL_joint3 -> LL_link4``. Unlike the
// V4 four-bar this has no simple closed form, so the map between the reduced
// knee angle and the crank is tabulated from the MuJoCo closed-chain model
// (``models/wheelleg/calibrate_kinematics.py``) and interpolated linearly.
//
// Conventions:
//   * reduced knee  beta : interior thigh-shank angle, straight = pi.
//   * relative crank    c : crank angle measured in the thigh frame,
//                           ``c = q_crank - q_hip`` (same for both legs).
//   * the tabulated crank is the native joint angle at ``q_hip = 0``.
//
// Mechanical limits and motor zero:
//   * the knee is mechanically limited to 40..110 deg (interior angle); it
//     cannot straighten (max 110 deg), by design.
//   * both coaxial hip motors (thigh and crank) are zeroed at the same pose:
//     knee interior angle = 110 deg (the real-robot encoder zero,
//     `encoder_zero_inner_knee_deg`) AND thigh perpendicular to the chassis.
//     The model shows the thigh is perpendicular at ``q_hip = 0``, so the hip
//     motor zero is ``q_hip = 0`` and the crank motor zero is ``crank(110deg)``.
class LegLinkage {
public:
    [[nodiscard]] double knee_min() const { return kBetaGrid.front(); }
    [[nodiscard]] double knee_max() const { return kBetaGrid.back(); }
    [[nodiscard]] double assembly_knee() const { return kAssemblyKnee; }
    [[nodiscard]] double assembly_crank() const { return kAssemblyCrank; }

    // Physical knee limits (interior angle), in radians.
    static constexpr double kKneeLimitMinDeg = 40.0;
    static constexpr double kKneeLimitMaxDeg = 110.0;
    static constexpr double kKneeMotorZeroDeg = 110.0;

    [[nodiscard]] static constexpr double knee_limit_min() {
        return kKneeLimitMinDeg * std::numbers::pi / 180.0;
    }
    [[nodiscard]] static constexpr double knee_limit_max() {
        return kKneeLimitMaxDeg * std::numbers::pi / 180.0;
    }
    [[nodiscard]] static constexpr double knee_motor_zero() {
        return kKneeMotorZeroDeg * std::numbers::pi / 180.0;
    }

    // Relative crank at the motor zero (encoder reads 0 here).
    [[nodiscard]] double crank_motor_zero() const {
        double crank = 0.0;
        (void)crank_from_knee(knee_motor_zero(), crank);
        return crank;
    }

    // Reduced knee -> relative crank. Returns false outside [knee_min, knee_max].
    [[nodiscard]] bool crank_from_knee(double knee, double& crank) const {
        if (knee < knee_min() || knee > knee_max())
            return false;
        crank = interpolate(kBetaGrid, kCrankGrid, knee);
        return true;
    }

    // Relative crank -> reduced knee (clamped to the tabulated range).
    [[nodiscard]] double knee_from_crank(double crank) const {
        const double clamped = std::clamp(crank, kCrankGrid.front(), kCrankGrid.back());
        return interpolate(kCrankGrid, kBetaGrid, clamped);
    }

    // The table above is in the MODEL frame (crank = 0 at the CAD assembly,
    // knee = 45 deg). The real DM encoders are zeroed at the MOTOR zero (knee
    // 105 deg), so the encoder relative crank c = q_crank - q_hip differs from
    // the tabulated crank by crank_motor_zero(). The real-hardware controllers
    // MUST go through these encoder-frame helpers, not the raw table.
    [[nodiscard]] double knee_from_encoder_crank(double c_encoder) const {
        return knee_from_crank(c_encoder + crank_motor_zero());
    }
    [[nodiscard]] bool encoder_crank_from_knee(double knee, double& c_encoder) const {
        double crank = 0.0;
        if (!crank_from_knee(knee, crank))
            return false;
        c_encoder = crank - crank_motor_zero();
        return true;
    }

private:
    template <std::size_t N>
    static double interpolate(
        const std::array<double, N>& xs, const std::array<double, N>& ys, double x) {
        const std::size_t upper = static_cast<std::size_t>(
            std::lower_bound(xs.begin(), xs.end(), x) - xs.begin());
        if (upper == 0)
            return ys.front();
        if (upper >= N)
            return ys.back();
        const double x0 = xs[upper - 1];
        const double x1 = xs[upper];
        const double t = (x - x0) / (x1 - x0);
        return ys[upper - 1] + t * (ys[upper] - ys[upper - 1]);
    }

    static constexpr std::size_t kSize = 101;
    static constexpr double kAssemblyKnee = 0.787329;
    static constexpr double kAssemblyCrank = 0.0;

    // beta uniform in [0.326383, 2.586387].
    static constexpr std::array<double, kSize> kBetaGrid = {
        0.326383, 0.348983, 0.371583, 0.394183, 0.416783, 0.439383, 0.461983, 0.484583, 0.507183,
        0.529783, 0.552383, 0.574983, 0.597583, 0.620183, 0.642783, 0.665383, 0.687983, 0.710583,
        0.733183, 0.755783, 0.778383, 0.800983, 0.823584, 0.846184, 0.868784, 0.891384, 0.913984,
        0.936584, 0.959184, 0.981784, 1.004384, 1.026984, 1.049584, 1.072184, 1.094784, 1.117384,
        1.139984, 1.162584, 1.185184, 1.207784, 1.230384, 1.252984, 1.275584, 1.298184, 1.320784,
        1.343384, 1.365985, 1.388585, 1.411185, 1.433785, 1.456385, 1.478985, 1.501585, 1.524185,
        1.546785, 1.569385, 1.591985, 1.614585, 1.637185, 1.659785, 1.682385, 1.704985, 1.727585,
        1.750185, 1.772785, 1.795385, 1.817985, 1.840585, 1.863185, 1.885785, 1.908386, 1.930986,
        1.953586, 1.976186, 1.998786, 2.021386, 2.043986, 2.066586, 2.089186, 2.111786, 2.134386,
        2.156986, 2.179586, 2.202186, 2.224786, 2.247386, 2.269986, 2.292586, 2.315186, 2.337786,
        2.360386, 2.382986, 2.405586, 2.428186, 2.450787, 2.473387, 2.495987, 2.518587, 2.541187,
        2.563787, 2.586387};

    // relative crank (native joint at q_hip = 0) for each kBetaGrid entry.
    static constexpr std::array<double, kSize> kCrankGrid = {
        -1.850000, -1.486884, -1.277137, -1.120978, -0.994545, -0.887366, -0.793800, -0.710363,
        -0.634808, -0.565499, -0.501285, -0.441311, -0.384908, -0.331530, -0.280773, -0.232300,
        -0.185822, -0.141099, -0.097952, -0.056207, -0.015719, 0.023633,  0.061954,  0.099336,
        0.135849,  0.171583,  0.206584,  0.240922,  0.274630,  0.307767,  0.340369,  0.372461,
        0.404086,  0.435272,  0.466043,  0.496423,  0.526433,  0.556094,  0.585422,  0.614435,
        0.643147,  0.671573,  0.699723,  0.727606,  0.755236,  0.782624,  0.809777,  0.836696,
        0.863397,  0.889886,  0.916158,  0.942230,  0.968100,  0.993772,  1.019255,  1.044541,
        1.069644,  1.094554,  1.119283,  1.143822,  1.168180,  1.192350,  1.216334,  1.240134,
        1.263737,  1.287153,  1.310376,  1.333393,  1.356210,  1.378823,  1.401219,  1.423392,
        1.445340,  1.467054,  1.488524,  1.509740,  1.530686,  1.551353,  1.571727,  1.591793,
        1.611535,  1.630933,  1.649966,  1.668598,  1.686813,  1.704583,  1.721883,  1.738662,
        1.754860,  1.770493,  1.785406,  1.799669,  1.813056,  1.825600,  1.837208,  1.847746,
        1.857053,  1.864945,  1.871347,  1.875673,  1.880000};
};

} // namespace rmcs_core::wheelleg
