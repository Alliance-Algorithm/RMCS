#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace rmcs_core::hardware::device {

class DmJointEnableSequence {
public:
    using Clock = std::chrono::steady_clock;
    enum class Command { kNone, kClearError, kEnable, kDisable };
    enum class Phase { kDisabled, kClearing, kEnabling, kActive, kFailed };
    enum class Failure { kNone, kStartupTimeout, kRunningUnavailable };

    // Fixed order: LH, LK, RH, RK. Freshness includes the driver's fault check.
    struct Feedback {
        bool fresh = false;
        int status = 0;
        Clock::time_point received_at{};
    };
    struct Step {
        Command system = Command::kNone;
        bool control_active = false;
        std::uint8_t system_mask = 0;
    };

    Step update(
        bool requested, bool controller_healthy, const std::array<Feedback, 4>& feedback,
        Clock::time_point now) {
        if (!requested) {
            if (phase_ != Phase::kDisabled) {
                *this = DmJointEnableSequence{};
                next_system_ = now;
            }
            if (now < next_system_)
                return {};
            const auto mask = side_mask_();
            advance_slot_(now);
            return {Command::kDisable, false, mask};
        }
        if (phase_ == Phase::kDisabled) {
            *this = DmJointEnableSequence{};
            phase_ = Phase::kClearing;
            started_ = now;
            next_system_ = now;
        }
        if (phase_ == Phase::kFailed)
            return {};

        const bool ready = controller_healthy && all_ready_(feedback);
        if (phase_ == Phase::kActive) {
            if (!ready) {
                phase_ = Phase::kFailed;
                failure_ = Failure::kRunningUnavailable;
                pending_mask_ = unavailable_mask_(feedback);
                return {};
            }
            return {Command::kNone, true, 0};
        }
        remaining_ms_ = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                             kStartupTimeout - (now - started_))
                                             .count());
        if (now - started_ >= kStartupTimeout) {
            phase_ = Phase::kFailed;
            failure_ = Failure::kStartupTimeout;
            remaining_ms_ = 0;
            return {};
        }

        if (phase_ == Phase::kClearing && now - started_ >= kClearDuration) {
            phase_ = Phase::kEnabling;
            next_side_ = 0;
            next_system_ = now;
        }
        if (phase_ == Phase::kEnabling) {
            pending_mask_ = 0;
            for (std::size_t i = 0; i < feedback.size(); ++i)
                if (!feedback[i].fresh || feedback[i].status != 1 || enable_attempts_[i] == 0
                    || feedback[i].received_at <= last_enable_[i])
                    pending_mask_ |= 1u << i;
            if (ready && pending_mask_ == 0) {
                if (!confirming_) {
                    confirming_ = true;
                    stable_since_ = now;
                }
                bool recent_samples = true;
                for (const auto& motor : feedback)
                    recent_samples &= motor.received_at >= stable_since_ + kStableDuration;
                if (now - stable_since_ >= kStableDuration && recent_samples) {
                    phase_ = Phase::kActive;
                    remaining_ms_ = 0;
                    return {Command::kNone, true, 0};
                }
            } else {
                confirming_ = false;
            }
        }
        if (now < next_system_)
            return {};
        const auto selected = side_mask_();
        advance_slot_(now);
        if (phase_ == Phase::kClearing)
            return {Command::kClearError, false, selected};

        std::uint8_t retry_mask = 0;
        for (std::size_t i = 0; i < feedback.size(); ++i) {
            if (!(selected & pending_mask_ & (1u << i)))
                continue;
            // A motor fault needs explicit disarm/clear, not repeated FC.
            if (feedback[i].status > 1)
                continue;
            if (enable_attempts_[i] != 0 && now - last_enable_[i] < kEnableRetryInterval)
                continue;
            retry_mask |= 1u << i;
            last_enable_[i] = now;
            ++enable_attempts_[i];
        }
        return {retry_mask ? Command::kEnable : Command::kNone, false, retry_mask};
    }

    Phase phase() const { return phase_; }
    Failure failure() const { return failure_; }
    int remaining_ms() const { return remaining_ms_; }
    std::uint8_t pending_mask() const { return pending_mask_; }
    const std::array<unsigned, 4>& enable_attempts() const { return enable_attempts_; }
    const char* phase_name() const {
        switch (phase_) {
        case Phase::kDisabled: return "disabled";
        case Phase::kClearing: return "clearing";
        case Phase::kEnabling: return "enabling";
        case Phase::kActive: return "active";
        case Phase::kFailed:
            return failure_ == Failure::kStartupTimeout ? "startup_timeout" : "running_unavailable";
        }
        return "unknown";
    }

private:
    static std::uint8_t unavailable_mask_(const std::array<Feedback, 4>& feedback) {
        std::uint8_t mask = 0;
        for (std::size_t i = 0; i < feedback.size(); ++i)
            if (!feedback[i].fresh || feedback[i].status != 1)
                mask |= 1u << i;
        return mask;
    }
    static bool all_ready_(const std::array<Feedback, 4>& feedback) {
        return unavailable_mask_(feedback) == 0;
    }
    std::uint8_t side_mask_() const { return next_side_ == 0 ? 0x3 : 0xc; }
    void advance_slot_(Clock::time_point now) {
        // One motor per bus; the next side follows in another USB transfer.
        next_system_ =
            now + (next_side_ == 0 ? std::chrono::milliseconds{5} : std::chrono::milliseconds{16});
        next_side_ ^= 1;
    }

    static constexpr auto kClearDuration = std::chrono::milliseconds{50};
    static constexpr auto kEnableRetryInterval = std::chrono::milliseconds{100};
    static constexpr auto kStableDuration = std::chrono::milliseconds{50};
    static constexpr auto kStartupTimeout = std::chrono::seconds{2};
    Phase phase_ = Phase::kDisabled;
    Failure failure_ = Failure::kNone;
    Clock::time_point started_{};
    Clock::time_point next_system_{};
    Clock::time_point stable_since_{};
    std::array<Clock::time_point, 4> last_enable_{};
    std::array<unsigned, 4> enable_attempts_{};
    std::uint8_t pending_mask_ = 0xf;
    unsigned next_side_ = 0;
    int remaining_ms_ = 0;
    bool confirming_ = false;
};

} // namespace rmcs_core::hardware::device
