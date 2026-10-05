#pragma once

#include <array>
#include <cstddef>
#include <expected>
#include <memory>
#include <string>
#include <string_view>

namespace rmcs::rl {

// V6 observation and actuator conventions shared by compatible ONNX actors.
// Model selection belongs to the startup model_path parameter.
struct DeployedPolicyContract {
    static constexpr std::string_view kName = "v6";
    static constexpr double kPolicyFrequencyHz = 50.0;
    // Frozen training/recovery reference cadence; host PD is configured separately.
    static constexpr double kControlFrequencyHz = 200.0;
    static constexpr double kPolicyPeriodSeconds = 1.0 / kPolicyFrequencyHz;
    static constexpr double kControlPeriodSeconds = 1.0 / kControlFrequencyHz;
    static constexpr double kNominalHeight = 0.305;
    static constexpr double kLegKp = 160.0;
    static constexpr double kLegKd = 2.5;
    static constexpr double kLegTorqueLimit = 40.0;
    static constexpr double kWheelKp = 0.6;
    static constexpr double kWheelTorqueLimit = 4.5;
    static constexpr double kForwardSlew = 1.5;
    static constexpr double kYawSlew = 4.0;
    static constexpr double kHeightMin = 0.23;
    static constexpr double kHeightMax = 0.43;
    static constexpr float kLegActionLimit = 3.0f;
    static constexpr float kWheelActionLimit = 9.0f;
    static constexpr double kLegActionScale = 0.25;
    static constexpr double kWheelActionScale = 10.0;
    static constexpr double kWheelSpeedLimit = kWheelActionScale * kWheelActionLimit;
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
    static constexpr double kForwardSlew = 0.6;
    static constexpr double kHeightMin = kNominalHeight;
    static constexpr double kHeightMax = kNominalHeight;
    static constexpr std::array<double, 6> kNominalPosition{
        0.42, -0.13742282595254576, -0.42, 0.13741557625658019, 0.0, 0.0};
};

struct PolicyProfile {
    std::string_view name;
    std::array<double, 6> nominal;
    double leg_kp, leg_kd, wheel_kp, wheel_torque_limit;
    double forward_slew, height_min, height_max;
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
    DeployedPolicyContract::kForwardSlew,
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
    LegacyPolicyContract::kForwardSlew,
    LegacyPolicyContract::kHeightMin,
    LegacyPolicyContract::kHeightMax,
    3.0,
    1.05,
    12.566370614359172 + 1e-3,
    true};

struct ObservationLayout {
    static constexpr std::size_t kCommand = 0;
    static constexpr std::size_t kHeight = kCommand + 3;
    static constexpr std::size_t kAngularVelocity = kHeight + 1;
    static constexpr std::size_t kProjectedGravity = kAngularVelocity + 3;
    static constexpr std::size_t kJointPosition = kProjectedGravity + 3;
    static constexpr std::size_t kJointVelocity = kJointPosition + 6;
    static constexpr std::size_t kPreviousAction = kJointVelocity + 6;
    static constexpr std::size_t kContext = kPreviousAction + 6;
    static constexpr std::size_t kSize = kContext + 7;

    static constexpr std::size_t kNormal = 0;
    static constexpr std::size_t kJumpRequest = 4;
    static constexpr std::size_t kJumpApex = 5;
    static constexpr std::size_t kJumpElapsed = 6;
};
static_assert(ObservationLayout::kSize == 35);

using PolicyObservation = std::array<float, ObservationLayout::kSize>;
using PolicyAction = std::array<float, 6>;

inline constexpr auto kObservationNames = std::to_array<std::string_view>({
    "command/forward",
    "command/lateral",
    "command/yaw",
    "height",
    "angular_velocity/x",
    "angular_velocity/y",
    "angular_velocity/z",
    "projected_gravity/x",
    "projected_gravity/y",
    "projected_gravity/z",
    "joint_position/left_hip",
    "joint_position/left_knee_drive",
    "joint_position/right_hip",
    "joint_position/right_knee_drive",
    "joint_position/left_wheel",
    "joint_position/right_wheel",
    "joint_velocity/left_hip",
    "joint_velocity/left_knee_drive",
    "joint_velocity/right_hip",
    "joint_velocity/right_knee_drive",
    "joint_velocity/left_wheel",
    "joint_velocity/right_wheel",
    "previous_action/left_hip",
    "previous_action/left_knee_drive",
    "previous_action/right_hip",
    "previous_action/right_knee_drive",
    "previous_action/left_wheel",
    "previous_action/right_wheel",
    "context/normal",
    "context/1",
    "context/2",
    "context/3",
    "context/jump_request",
    "context/jump_apex",
    "context/jump_elapsed",
});
static_assert(kObservationNames.size() == ObservationLayout::kSize);

class OnnxPolicy {
public:
    explicit OnnxPolicy(const std::string& model_path);
    ~OnnxPolicy();
    OnnxPolicy(const OnnxPolicy&) = delete;
    OnnxPolicy& operator=(const OnnxPolicy&) = delete;

    std::expected<PolicyAction, std::string> run(const PolicyObservation& observation);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace rmcs::rl
