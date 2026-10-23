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
    kReversed,
    kInconsistentSequence,
    kSkew,
};

struct FeedbackSampleStamp {
    std::uint64_t sequence = 0;
    std::uint64_t steady_ns = 0;
};

struct FeedbackSensorGuardConfig {
    double motor_age_seconds = 0.02;
    double imu_age_seconds = 0.02;
    double acceleration_age_seconds = 0.03;
    double maximum_skew_seconds = 0.01;
};

struct FeedbackSensorStatus {
    bool valid = false;
    FeedbackSensorIssue issue = FeedbackSensorIssue::kMissing;
    // Bits 0..5: P-order motors; 6: orientation/gyro; 7: acceleration.
    std::uint16_t invalid_mask = 0xff;
    double maximum_motor_age_ms = -1.0;
    double imu_age_ms = -1.0;
    double acceleration_age_ms = -1.0;
};

// Repeated snapshots are allowed while fresh.
// All comparisons use the hardware callback's steady clock, not scheduled ticks.
class FeedbackSensorGuard {
public:
    explicit FeedbackSensorGuard(FeedbackSensorGuardConfig config = {})
        : config_{config} {
        for (double value :
             {config.motor_age_seconds, config.imu_age_seconds, config.acceleration_age_seconds,
              config.maximum_skew_seconds})
            if (!std::isfinite(value) || value <= 0 || value > 0.05)
                throw std::invalid_argument("Feedback sensor limits must be in (0, 50 ms]");
    }

    void reset() noexcept { previous_ = {}; }

    FeedbackSensorStatus
        update(std::uint64_t now_ns, const std::array<FeedbackSampleStamp, 8>& samples) noexcept {
        FeedbackSensorStatus status;
        status.issue = FeedbackSensorIssue::kNone;
        status.invalid_mask = 0;
        std::uint64_t oldest = now_ns, newest = 0;
        for (std::size_t i = 0; i < samples.size(); ++i) {
            const auto [sequence, stamp] = samples[i];
            const auto [old_sequence, old_stamp] = previous_[i];
            FeedbackSensorIssue issue = FeedbackSensorIssue::kNone;
            const double age = stamp > 0 && stamp <= now_ns ? (now_ns - stamp) * 1e-6 : -1.0;
            if (i < 6)
                status.maximum_motor_age_ms = std::max(status.maximum_motor_age_ms, age);
            else if (i == 6)
                status.imu_age_ms = age;
            else
                status.acceleration_age_ms = age;
            const double maximum_age = i < 6  ? config_.motor_age_seconds
                                     : i == 6 ? config_.imu_age_seconds
                                              : config_.acceleration_age_seconds;
            if (now_ns == 0 || sequence == 0 || stamp == 0)
                issue = FeedbackSensorIssue::kMissing;
            else if (stamp > now_ns)
                issue = FeedbackSensorIssue::kFuture;
            else if ((now_ns - stamp) * 1e-9 > maximum_age)
                issue = FeedbackSensorIssue::kExpired;
            else if (old_sequence != 0 && (sequence < old_sequence || stamp < old_stamp))
                issue = FeedbackSensorIssue::kReversed;
            else if (old_sequence != 0 && ((sequence == old_sequence) != (stamp == old_stamp)))
                issue = FeedbackSensorIssue::kInconsistentSequence;
            if (issue != FeedbackSensorIssue::kNone) {
                status.invalid_mask |= std::uint16_t{1} << i;
                if (status.issue == FeedbackSensorIssue::kNone)
                    status.issue = issue;
            } else {
                oldest = std::min(oldest, stamp);
                newest = std::max(newest, stamp);
            }
        }
        if (status.invalid_mask == 0 && (newest - oldest) * 1e-9 > config_.maximum_skew_seconds) {
            status.invalid_mask = 0xff;
            status.issue = FeedbackSensorIssue::kSkew;
        }
        status.valid = status.invalid_mask == 0;
        // Commit the complete snapshot only after validation.
        if (status.valid)
            previous_ = samples;
        return status;
    }

private:
    FeedbackSensorGuardConfig config_;
    std::array<FeedbackSampleStamp, 8> previous_{};
};

} // namespace rmcs_core::controller::chassis
