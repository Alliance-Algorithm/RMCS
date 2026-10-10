#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <limits>

#include <eigen3/Eigen/Core>

namespace rmcs_msgs {

// Optional feedback used only by the foldable-sentry motion observer. Times are
// host steady-clock reception times, not device-clock or ROS timestamps.
struct ChassisMotionFeedback {
    using Clock = std::chrono::steady_clock;

    // Left front, left back, right back, right front; output-shaft rad/s.
    std::array<double, 4> wheel_velocity{};
    std::array<Clock::time_point, 4> wheel_stamp{};
    std::array<std::uint64_t, 4> wheel_sequence{};
    double yaw_rate = 0.0;
    Clock::time_point imu_stamp{};
    std::uint64_t imu_sequence = 0;
};

enum class MotionKind { UNKNOWN, STATIONARY, TRANSLATING, ROTATING, COMBINED };
enum class MotionQuality { INVALID, PREDICTED, WHEEL_ONLY, FUSED };

struct ChassisMotionState {
    using Clock = std::chrono::steady_clock;

    Clock::time_point timestamp{};
    // [vx (m/s), vy (m/s), yaw rate (rad/s)] at the chassis centre in base_link.
    // This mixed-unit vector must not be transformed as a 3D linear velocity.
    Eigen::Vector3d velocity = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix3d covariance =
        Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
    MotionKind kind = MotionKind::UNKNOWN;
    MotionQuality quality = MotionQuality::INVALID;

    // Predictions remain available for diagnosis but are not fresh feedback.
    [[nodiscard]] bool usable() const noexcept {
        return quality == MotionQuality::WHEEL_ONLY || quality == MotionQuality::FUSED;
    }
};

} // namespace rmcs_msgs
