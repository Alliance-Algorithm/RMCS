#pragma once

#include <array>
#include <cstddef>
#include <numbers>
#include <string_view>

#include "full_vehicle_v202.hpp"

namespace rmcs_core::controller::chassis {

// V6 observation and actuator conventions shared by compatible ONNX actors.
// Model selection belongs to the startup model_path parameter.
struct DeployedPolicyContract {
    static constexpr std::string_view kName = "v6";
    // Legacy default only; the paired bridge supplies the model's policy cadence.
    static constexpr double kPolicyFrequencyHz = 50.0;
    // Minimum feedback cadence; the deployed PD period is configured separately.
    static constexpr double kControlFrequencyHz = 200.0;
    static constexpr double kNominalHeight = 0.305;
    static constexpr double kLegKp = 160.0;
    static constexpr double kLegKd = 2.5;
    static constexpr double kLegTorqueLimit = 40.0;
    static constexpr double kWheelKp = 0.6;
    static constexpr double kWheelTorqueLimit = 4.5;
    static constexpr double kHeightMin = 0.23;
    static constexpr double kHeightMax = 0.43;
    static constexpr std::array<double, 6> kNominalPosition{
        -0.42, 0.13742282595395358, 0.42, -0.1374155762580851, 0.0, 0.0};
};

// Retained for explicit legacy control profiles.
struct LegacyPolicyContract : DeployedPolicyContract {
    static constexpr std::string_view kName = "v5";
    static constexpr double kLegKp = 60.0;
    static constexpr double kLegKd = 2.0;
    static constexpr double kWheelKp = 0.2;
    static constexpr double kHeightMin = kNominalHeight;
    static constexpr double kHeightMax = kNominalHeight;
    static constexpr std::array<double, 6> kNominalPosition{
        0.42, -0.13742282595254576, -0.42, 0.13741557625658019, 0.0, 0.0};
};

struct PolicyProfile {
    std::string_view name;
    std::array<double, 6> nominal;
    double leg_kp, leg_kd, wheel_kp, wheel_torque_limit;
    double height_min, height_max;
    double forward_limit, negative_yaw_limit, positive_yaw_limit;
};

inline constexpr PolicyProfile kV6PolicyProfile{
    DeployedPolicyContract::kName,
    DeployedPolicyContract::kNominalPosition,
    DeployedPolicyContract::kLegKp,
    DeployedPolicyContract::kLegKd,
    DeployedPolicyContract::kWheelKp,
    DeployedPolicyContract::kWheelTorqueLimit,
    DeployedPolicyContract::kHeightMin,
    DeployedPolicyContract::kHeightMax,
    0.5,
    1.0,
    1.0};

inline constexpr PolicyProfile kV5PolicyProfile{
    LegacyPolicyContract::kName,
    LegacyPolicyContract::kNominalPosition,
    LegacyPolicyContract::kLegKp,
    LegacyPolicyContract::kLegKd,
    LegacyPolicyContract::kWheelKp,
    LegacyPolicyContract::kWheelTorqueLimit,
    LegacyPolicyContract::kHeightMin,
    LegacyPolicyContract::kHeightMax,
    3.0,
    1.05,
    12.566370614359172 + 1e-3};

inline constexpr PolicyProfile kFullVehicleV202PolicyProfile{
    "full_vehicle_v202",
    full_vehicle_v202::kNominalPosition,
    160.0,
    2.5,
    .5,
    4.5,
    .23,
    .42,
    2.0,
    10.471975511965978,
    10.471975511965978};

// Frozen 2026-11-03 training interfaces. Geometry is numerically identical to
// full_vehicle_v202; the friction asset has its own authenticated file identity.
inline constexpr std::string_view kFrictionV202ModelSpecSha256 =
    "8a9d759525c48955e8965b2e14b9e158c1d8fc9f3d68f33e4024a3c94bf8f812";
inline constexpr std::string_view kFrictionV202GasModelSha256 =
    "698f67ccc7d6364b73f1b426ed3e6386ffc9e11912ef95a89d716eb7ee0e06b9";
inline constexpr PolicyProfile kProne200PolicyProfile = [] {
    auto profile = kFullVehicleV202PolicyProfile;
    profile.name = "prone_motion_v4_200hz";
    return profile;
}();
inline constexpr PolicyProfile kWheelbipePolicyProfile = [] {
    auto profile = kFullVehicleV202PolicyProfile;
    profile.name = "wheelbipe_v202";
    profile.leg_kp = 60.0;
    profile.leg_kd = 2.0;
    profile.forward_limit = 2.7;
    profile.negative_yaw_limit = profile.positive_yaw_limit = 2.0 * std::numbers::pi;
    return profile;
}();

inline constexpr bool full_vehicle_profile(std::string_view name) {
    return name == kFullVehicleV202PolicyProfile.name || name == kProne200PolicyProfile.name
        || name == kWheelbipePolicyProfile.name;
}

inline constexpr bool root_drive_profile(std::string_view name) {
    return name == kV6PolicyProfile.name || full_vehicle_profile(name);
}

} // namespace rmcs_core::controller::chassis
