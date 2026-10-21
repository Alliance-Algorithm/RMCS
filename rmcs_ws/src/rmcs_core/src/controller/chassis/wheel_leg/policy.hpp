#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <string_view>

namespace rmcs_core::controller::chassis {

// V6 observation and actuator conventions shared by compatible ONNX actors.
// Model selection belongs to the startup model_path parameter.
struct DeployedPolicyContract {
    static constexpr std::string_view kName = "v6";
    // Legacy default only; the paired bridge supplies the model's policy cadence.
    static constexpr double kPolicyFrequencyHz = 50.0;
    // Minimum feedback cadence; deployed PD and recovery periods are configured separately.
    static constexpr double kControlFrequencyHz = 200.0;
    static constexpr double kPolicyPeriodSeconds = 1.0 / kPolicyFrequencyHz;
    static constexpr double kControlPeriodSeconds = 1.0 / kControlFrequencyHz;
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

// Retained solely for the V5 recovery regression and explicit legacy profiles.
// Its geometry and recovery evidence do not qualify the V6 candidate.
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
    bool recovery_supported;
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
    1.0,
    true};

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
    12.566370614359172 + 1e-3,
    true};

// Six physical actuators; observation shape and actor channel order come from YAML.
using PolicyAction = std::array<float, 6>;

} // namespace rmcs_core::controller::chassis
