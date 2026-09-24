#pragma once

#include <array>
#include <cstddef>
#include <expected>
#include <memory>
#include <string>
#include <string_view>

namespace rmcs::rl {

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
