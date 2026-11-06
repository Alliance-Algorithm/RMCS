#pragma once

#include <chrono>
#include <cstdint>

namespace rmcs_core::hardware::device {

class LkMotorDriveScheduler {
public:
    using Clock = std::chrono::steady_clock;

    enum class Action { kNone, kClearError, kEnable, kDisable, kTorque, kErrorPoll };
    enum class Phase { kDisabled, kClearing, kEnabling, kActive };

    /// @brief Latest status 1 reply of the motor (0x9A poll / 0x9B echo).
    struct Feedback {
        bool valid = false;
        std::uint8_t motor_state = 0x10;
        std::uint8_t error_state = 0;
        Clock::time_point received_at{};
    };

    /// @brief Advance one command cycle (the board loop runs at 1 kHz).
    Action next(bool requested, const Feedback& feedback, Clock::time_point now) noexcept {
        if (!requested) {
            phase_ = Phase::kDisabled;
            phase_started_ = now;
            next_frame_ = now;
            clear_confirmed_ = false;
            saw_status_ = false;
            return Action::kDisable;
        }

        if (phase_ == Phase::kDisabled) {
            enter_(Phase::kClearing, now);
            clear_confirmed_ = false;
            saw_status_ = false;
        }

        if (phase_ == Phase::kClearing) {
            const bool replied = feedback.valid && feedback.received_at >= phase_started_;
            saw_status_ = saw_status_ || replied;
            if (replied && feedback.error_state == 0) {
                clear_confirmed_ = true;
                enter_(Phase::kEnabling, now);
            } else if (!saw_status_ && now - phase_started_ >= kClearTimeout) {
                // Motor ignores 0x9A/0x9B: run anyway, clear_confirmed() stays false.
                enter_(Phase::kEnabling, now);
            } else {
                if (now < next_frame_)
                    return Action::kNone;
                next_frame_ = now + kFrameInterval;
                return Action::kClearError;
            }
        }

        if (phase_ == Phase::kEnabling) {
            if (now - phase_started_ < kEnableDuration) {
                if (now < next_frame_)
                    return Action::kNone;
                next_frame_ = now + kFrameInterval;
                return Action::kEnable;
            }
            enter_(Phase::kActive, now);
            next_poll_ = now + kPollInterval;
        }

        // Phase::kActive
        if (feedback.valid && feedback.error_state != 0) {
            if (now - last_rearm_ < kRearmCooldown)
                return Action::kDisable;
            last_rearm_ = now;
            ++rearm_count_;
            enter_(Phase::kClearing, now);
            clear_confirmed_ = false;
            saw_status_ = false;
            return Action::kClearError;
        }
        if (now >= next_poll_) {
            next_poll_ = now + kPollInterval;
            return Action::kErrorPoll;
        }
        return Action::kTorque;
    }

    Phase phase() const noexcept { return phase_; }
    bool active() const noexcept { return phase_ == Phase::kActive; }
    bool clear_confirmed() const noexcept { return clear_confirmed_; }
    unsigned rearm_count() const noexcept { return rearm_count_; }

    static const char* phase_name(Phase phase) noexcept {
        switch (phase) {
        case Phase::kDisabled: return "disabled";
        case Phase::kClearing: return "clearing";
        case Phase::kEnabling: return "enabling";
        case Phase::kActive: return "active";
        }
        return "unknown";
    }
    const char* phase_name() const noexcept { return phase_name(phase_); }

private:
    void enter_(Phase phase, Clock::time_point now) noexcept {
        phase_ = phase;
        phase_started_ = now;
        next_frame_ = now;
    }

    static constexpr auto kFrameInterval = std::chrono::milliseconds{10};
    static constexpr auto kClearTimeout = std::chrono::milliseconds{300};
    static constexpr auto kEnableDuration = std::chrono::milliseconds{100};
    static constexpr auto kPollInterval = std::chrono::milliseconds{100};
    static constexpr auto kRearmCooldown = std::chrono::milliseconds{500};

    Phase phase_ = Phase::kDisabled;
    Clock::time_point phase_started_{};
    Clock::time_point next_frame_{};
    Clock::time_point next_poll_{};
    Clock::time_point last_rearm_{};
    unsigned rearm_count_ = 0;
    bool clear_confirmed_ = false;
    bool saw_status_ = false;
};

} // namespace rmcs_core::hardware::device
