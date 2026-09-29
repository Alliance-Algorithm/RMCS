#pragma once

#include <array>
#include <cstddef>
#include <expected>
#include <memory>
#include <string>
#include <string_view>

namespace rmcs::rl {

// Contract of the bundled v5_flat_12486 ONNX, not the current V6 training
// contract. A 35D tensor alone cannot identify observation or actuator semantics.
// See docs/zh-cn/wheel_leg_refactoring_20260929.md for the version boundary.
struct DeployedPolicyContract {
    static constexpr double kPolicyFrequencyHz = 50.0;
    static constexpr double kControlFrequencyHz = 200.0;
    static constexpr double kPolicyPeriodSeconds = 1.0 / kPolicyFrequencyHz;
    static constexpr double kControlPeriodSeconds = 1.0 / kControlFrequencyHz;
    static constexpr double kNominalHeight = 0.305;
    static constexpr double kLegKp = 60.0;
    static constexpr double kLegKd = 2.0;
    static constexpr double kLegTorqueLimit = 40.0;
    static constexpr double kWheelKp = 0.2;
    static constexpr float kLegActionLimit = 3.0f;
    static constexpr float kWheelActionLimit = 9.0f;
    static constexpr double kLegActionScale = 0.25;
    static constexpr double kWheelActionScale = 10.0;
    static constexpr double kWheelSpeedLimit = kWheelActionScale * kWheelActionLimit;
    static constexpr std::array<double, 6> kNominalPosition{
        0.42, -0.13742282595254576, -0.42, 0.13741557625658019, 0.0, 0.0};
};

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
