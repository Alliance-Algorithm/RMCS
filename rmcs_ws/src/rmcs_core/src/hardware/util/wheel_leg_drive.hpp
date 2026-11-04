#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::hardware {

template <typename Now, typename... Stamps>
bool wheel_leg_feedback_fresh(Now read_now, std::int64_t timeout_ns, const Stamps&... stamps) {
    static_assert(sizeof...(Stamps) > 0);
    // Snapshot callback-owned timestamps BEFORE reading the comparison clock.
    // Reading the clock first lets a concurrent CAN/DR16/IMU callback publish
    // a newer timestamp that is then incorrectly rejected as future feedback.
    const std::array captured{stamps.load(std::memory_order_relaxed)...};
    const auto now = read_now();
    for (const auto stamp : captured) {
        if (stamp <= 0 || now < stamp || now - stamp >= timeout_ns)
            return false;
    }
    return true;
}

template <typename Now, typename... Stamps>
bool wheel_leg_feedback_available(Now read_now, const Stamps&... stamps) {
    static_assert(sizeof...(Stamps) > 0);
    // Availability is the one-time startup gate. Once every channel has
    // supplied a valid sample, the controller may continue using the last
    // sample; age remains exposed through wheel_leg_feedback_fresh().
    const std::array captured{stamps.load(std::memory_order_relaxed)...};
    const auto now = read_now();
    for (const auto stamp : captured)
        if (stamp <= 0 || now < stamp)
            return false;
    return true;
}

class WheelLegDmCommandScheduler {
public:
    enum class Action { kNone, kClearError, kEnable, kDisable, kMit, kFeedbackPoll };

    Action next(bool enable, bool entered_both_down = false) noexcept {
        // A fresh DOWN edge is an explicit disarm command even if this graph
        // never requested enable (e.g. passive observation or an aborted arm).
        if (entered_both_down && !enable) {
            enabled_ = false;
            resend_remaining_ = kResendCycles;
            heartbeat_cycles_ = 0;
        }
        if (enable != enabled_) {
            enabled_ = enable;
            resend_remaining_ = kResendCycles;
            heartbeat_cycles_ = 0;
            if (enable)
                return Action::kClearError;
        }
        if (resend_remaining_ > 0) {
            // After the first FC, one zero-gain/zero-torque MIT frame clears
            // any old internal PD target when status=1 survived a quick reset.
            // Keep it separate from FC to avoid overflowing the board TX FIFO.
            const bool neutralize = enabled_ && resend_remaining_ == kResendCycles - 1;
            --resend_remaining_;
            if (neutralize)
                return Action::kFeedbackPoll;
            return enabled_ ? Action::kEnable : Action::kDisable;
        }
        if (++heartbeat_cycles_ >= kHeartbeatCycles) {
            heartbeat_cycles_ = 0;
            return enabled_ ? Action::kEnable : Action::kDisable;
        }
        if (enabled_)
            return Action::kMit;
        // Disabled DM drives on this rig respond to a neutral MIT frame.
        // Keep the same 500 Hz side slots between 0xFD heartbeats.
        return Action::kFeedbackPoll;
    }

private:
    // Each side advances at 500 Hz in the 1 kHz board command loop.
    static constexpr int kResendCycles = 50;
    static constexpr int kHeartbeatCycles = 250;
    int resend_remaining_ = kResendCycles;
    int heartbeat_cycles_ = 0;
    bool enabled_ = false;
};

// Retry a gated clear/enable system command at 10 Hz in a 500 Hz side slot. The
// caller validates remote state, drive health and feedback availability separately.
class WheelLegDmFaultClearScheduler {
public:
    bool next(bool requested) noexcept {
        if (!requested) {
            remaining_ = 0;
            return false;
        }
        if (remaining_ > 0) {
            --remaining_;
            return false;
        }
        remaining_ = 49;
        return true;
    }

private:
    int remaining_ = 0;
};

struct WheelLegDmPairFeedback {
    int hip_status = -1;
    int knee_status = -1;
    int hip_fault = -1;
    int knee_fault = -1;
    // True after both motors have supplied at least one valid status frame.
    // Age is a diagnostic signal and is not an arm/stop gate here.
    bool available = false;
};

constexpr bool wheel_leg_dm_pair_safe_to_arm(const WheelLegDmPairFeedback& pair) noexcept {
    return pair.available && pair.hip_fault == 0 && pair.knee_fault == 0
        && (pair.hip_status == 0 || pair.hip_status == 1)
        && (pair.knee_status == 0 || pair.knee_status == 1);
}

constexpr bool wheel_leg_dm_pair_confirmed_disabled(const WheelLegDmPairFeedback& pair) noexcept {
    return pair.available && pair.hip_fault == 0 && pair.knee_fault == 0 && pair.hip_status == 0
        && pair.knee_status == 0;
}

constexpr bool wheel_leg_dm_pair_enabled(const WheelLegDmPairFeedback& pair) noexcept {
    return pair.available && pair.hip_fault == 0 && pair.knee_fault == 0 && pair.hip_status == 1
        && pair.knee_status == 1;
}

// Send FB immediately on the first request, avoid FC while a fault persists,
// and retry a lost FC until status=1. MIT torque is never used as an FC retry.
class WheelLegNormalActionGate {
public:
    using Action = WheelLegDmCommandScheduler::Action;

    std::array<Action, 2> apply(
        std::array<Action, 2> actions, const std::array<bool, 2>& requested,
        const std::array<WheelLegDmPairFeedback, 2>& pairs) noexcept {
        for (std::size_t side = 0; side < actions.size(); ++side) {
            if (!requested[side]) {
                fault_clear_[side].next(false);
                enable_retry_[side].next(false);
            } else if (actions[side] == Action::kNone) {
                // The other side owns this tick. Do not create an unscheduled retry.
                continue;
            } else if (!wheel_leg_dm_pair_safe_to_arm(pairs[side])) {
                actions[side] = fault_clear_[side].next(true) ? Action::kClearError : Action::kNone;
                enable_retry_[side].next(false);
            } else {
                fault_clear_[side].next(false);
                if (actions[side] == Action::kMit && !wheel_leg_dm_pair_enabled(pairs[side]))
                    actions[side] =
                        enable_retry_[side].next(true) ? Action::kEnable : Action::kNone;
                else
                    enable_retry_[side].next(false);
            }
        }
        return actions;
    }

    void reset() noexcept {
        for (auto& scheduler : fault_clear_)
            scheduler.next(false);
        for (auto& scheduler : enable_retry_)
            scheduler.next(false);
    }

private:
    std::array<WheelLegDmFaultClearScheduler, 2> fault_clear_;
    std::array<WheelLegDmFaultClearScheduler, 2> enable_retry_;
};

class WheelLegDmSideSchedulers {
public:
    using Action = WheelLegDmCommandScheduler::Action;

    template <typename Send>
    static void dispatch(
        const std::array<Action, 2>& actions, const std::array<std::size_t, 4>& axis_order,
        Send&& send) {
        for (const auto axis : axis_order)
            if (const auto action = actions[axis / 2]; action != Action::kNone)
                send(axis, action);
        // Queue every FD before its neutral MIT in the same board transfer.
        // Keep system frames first for the board's non-retrying CAN TX FIFO;
        // the neutral frame also clears any previously latched internal PD.
        for (const auto axis : axis_order)
            if (actions[axis / 2] == Action::kDisable)
                send(axis, Action::kFeedbackPoll);
    }

    std::array<Action, 2>
        next(const std::array<bool, 2>& enable, bool entered_both_down = false) noexcept {
        scheduled_side_ = 1 - scheduled_side_;
        std::array actions{Action::kNone, Action::kNone};
        for (std::size_t side = 0; side < actions.size(); ++side) {
            // A stop bypasses the alternating slots. Neither a remote stop nor
            // a feedback timeout may leave the other side waiting for its turn.
            if (!enable[side] && (previous_enable_[side] || entered_both_down))
                actions[side] = schedulers_[side].next(false, entered_both_down);
            if (side == scheduled_side_ && actions[side] == Action::kNone)
                actions[side] = schedulers_[side].next(enable[side]);
        }
        previous_enable_ = enable;
        return actions;
    }

    std::size_t scheduled_side() const noexcept { return scheduled_side_; }

private:
    std::array<WheelLegDmCommandScheduler, 2> schedulers_;
    std::array<bool, 2> previous_enable_{};
    std::size_t scheduled_side_ = 1;
};

} // namespace rmcs_core::hardware
