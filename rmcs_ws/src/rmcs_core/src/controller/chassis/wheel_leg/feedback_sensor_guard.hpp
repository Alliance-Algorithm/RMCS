#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace rmcs_core::controller::chassis {

enum class FeedbackSensorIssue : int {
    kNone,
    kMissing,
    kFuture,
    kExpired,
};

struct FeedbackSampleStamp {
    std::uint64_t sequence = 0;
    std::uint64_t steady_ns = 0;
};

struct FeedbackSensorGuardConfig {
    double motor_age_seconds = 0.02;
    double imu_age_seconds = 0.02;
};

struct FeedbackSensorStatus {
    bool valid = false;
    FeedbackSensorIssue issue = FeedbackSensorIssue::kMissing;
    // Bits 0..5: P-order motors; 6: orientation/gyro. Acceleration is diagnostic only.
    std::uint16_t invalid_mask = 0xff;
    double maximum_motor_age_ms = -1.0;
    double imu_age_ms = -1.0;
    double acceleration_age_ms = -1.0;
};

// Validate only the arrival of feedback used by control. Sequence changes,
// cross-sensor skew and acceleration diagnostics do not gate the policy.
class FeedbackSensorGuard {
public:
    explicit FeedbackSensorGuard(FeedbackSensorGuardConfig config = {})
        : config_{config} {
        for (double value : {config.motor_age_seconds, config.imu_age_seconds})
            if (!std::isfinite(value) || value <= 0 || value > 0.05)
                throw std::invalid_argument("Feedback sensor limits must be in (0, 50 ms]");
    }

    FeedbackSensorStatus
        update(std::uint64_t now_ns, const std::array<FeedbackSampleStamp, 8>& samples) const noexcept {
        FeedbackSensorStatus status;
        status.issue = FeedbackSensorIssue::kNone;
        status.invalid_mask = 0;
        for (std::size_t i = 0; i < samples.size(); ++i) {
            const auto [sequence, stamp] = samples[i];
            FeedbackSensorIssue issue = FeedbackSensorIssue::kNone;
            const double age = stamp > 0 && stamp <= now_ns ? (now_ns - stamp) * 1e-6 : -1.0;
            if (i < 6)
                status.maximum_motor_age_ms = std::max(status.maximum_motor_age_ms, age);
            else if (i == 6)
                status.imu_age_ms = age;
            else
                status.acceleration_age_ms = age;
            if (i == 7)
                continue;
            const double maximum_age = i < 6 ? config_.motor_age_seconds : config_.imu_age_seconds;
            if (now_ns == 0 || sequence == 0 || stamp == 0)
                issue = FeedbackSensorIssue::kMissing;
            else if (stamp > now_ns)
                issue = FeedbackSensorIssue::kFuture;
            else if ((now_ns - stamp) * 1e-9 > maximum_age)
                issue = FeedbackSensorIssue::kExpired;
            if (issue != FeedbackSensorIssue::kNone) {
                status.invalid_mask |= std::uint16_t{1} << i;
                if (status.issue == FeedbackSensorIssue::kNone)
                    status.issue = issue;
            }
        }
        status.valid = status.invalid_mask == 0;
        return status;
    }

private:
    FeedbackSensorGuardConfig config_;
};

} // namespace rmcs_core::controller::chassis
