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

constexpr bool wheel_leg_drive_allowed(
    bool dr16_fresh, rmcs_msgs::Switch left, rmcs_msgs::Switch right, bool require_request,
    bool requested) noexcept {
    return dr16_fresh && left == rmcs_msgs::Switch::MIDDLE && right == rmcs_msgs::Switch::MIDDLE
        && (!require_request || requested);
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
        // Disabled DM drives on this rig respond to a neutral MIT frame. Poll
        // at 100 Hz so CAN ages remain observable between 0xFD heartbeats.
        if (heartbeat_cycles_ % kFeedbackPollCycles == 0)
            return Action::kFeedbackPoll;
        return Action::kNone;
    }

private:
    static constexpr int kResendCycles = 100;
    static constexpr int kHeartbeatCycles = 500;
    static constexpr int kFeedbackPollCycles = 10;
    int resend_remaining_ = kResendCycles;
    int heartbeat_cycles_ = 0;
    bool enabled_ = false;
};

// Retry a gated clear/enable system command at 10 Hz while requested. The
// caller validates remote state, drive health and fresh feedback separately.
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
        remaining_ = 99;
        return true;
    }

private:
    int remaining_ = 0;
};

// No bound side means the existing RL/reference graph: both paired drives arm.
// Invalid bound values are never interpreted as that default.
constexpr bool wheel_leg_valid_selected_side(bool bound, int side) noexcept {
    return !bound || side == 0 || side == 1;
}

constexpr std::array<bool, 2>
    wheel_leg_side_enable_requests(bool enable, bool bound, int side) noexcept {
    return {enable && (!bound || side == 0), enable && (!bound || side == 1)};
}

struct WheelLegDmPairFeedback {
    int hip_status = -1;
    int knee_status = -1;
    int hip_fault = -1;
    int knee_fault = -1;
    bool fresh = false;
};

constexpr bool wheel_leg_dm_pair_safe_to_arm(const WheelLegDmPairFeedback& pair) noexcept {
    return pair.fresh && pair.hip_fault == 0 && pair.knee_fault == 0
        && (pair.hip_status == 0 || pair.hip_status == 1)
        && (pair.knee_status == 0 || pair.knee_status == 1);
}

constexpr bool wheel_leg_dm_pair_confirmed_disabled(const WheelLegDmPairFeedback& pair) noexcept {
    return pair.fresh && pair.hip_fault == 0 && pair.knee_fault == 0 && pair.hip_status == 0
        && pair.knee_status == 0;
}

// Before the first enable, both drives must report disabled: do not inherit
// possibly live motor-internal PD from an earlier session. During our own
// enable/resend sequence, status 0 or 1 is acceptable until the first MIT.
constexpr bool wheel_leg_dm_pair_safe_for_request(
    const WheelLegDmPairFeedback& pair, bool previously_requested) noexcept {
    return previously_requested ? wheel_leg_dm_pair_safe_to_arm(pair)
                                : wheel_leg_dm_pair_confirmed_disabled(pair);
}

// FB is sent as soon as a fresh double-MIDDLE request arrives; an already
// disabled status is not a precondition for clearing faults or asking for FC.
// Nonzero torque remains guarded by confirmed enabled feedback and first MIT.
constexpr bool wheel_leg_normal_enable_request(
    bool dr16_fresh, rmcs_msgs::Switch left, rmcs_msgs::Switch right, bool require_request,
    bool requested, bool feedback_fresh,
    const std::array<WheelLegDmPairFeedback, 2>& pairs) noexcept {
    return wheel_leg_drive_allowed(dr16_fresh, left, right, require_request, requested)
        && feedback_fresh && pairs[0].fresh && pairs[1].fresh;
}

constexpr bool wheel_leg_dm_pair_enabled(const WheelLegDmPairFeedback& pair) noexcept {
    return pair.fresh && pair.hip_fault == 0 && pair.knee_fault == 0 && pair.hip_status == 1
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

constexpr bool wheel_leg_wheel_only_allowed(
    bool remote_fresh, rmcs_msgs::Switch left, rmcs_msgs::Switch right, bool requested,
    bool feedback_fresh, const std::array<WheelLegDmPairFeedback, 2>& pairs) noexcept {
    return wheel_leg_drive_allowed(remote_fresh, left, right, true, requested) && feedback_fresh
        && wheel_leg_dm_pair_confirmed_disabled(pairs[0])
        && wheel_leg_dm_pair_confirmed_disabled(pairs[1]);
}

constexpr bool wheel_leg_dm_pair_clearable_fault(const WheelLegDmPairFeedback& pair) noexcept {
    if (!pair.fresh)
        return false;
    const auto valid = [](int status, int fault) {
        return (status == 0 && fault == 0) || (status >= 8 && status <= 14 && fault == status);
    };
    return valid(pair.hip_status, pair.hip_fault) && valid(pair.knee_status, pair.knee_fault)
        && (pair.hip_fault != 0 || pair.knee_fault != 0);
}

constexpr bool wheel_leg_selected_pair_clear_allowed(
    bool dr16_fresh, rmcs_msgs::Switch left, rmcs_msgs::Switch right, bool enable_requested,
    bool clear_requested, int selected_side,
    const std::array<WheelLegDmPairFeedback, 2>& pairs) noexcept {
    if (!clear_requested
        || !wheel_leg_drive_allowed(dr16_fresh, left, right, true, enable_requested)
        || !wheel_leg_valid_selected_side(true, selected_side))
        return false;
    const auto active = static_cast<std::size_t>(selected_side);
    return wheel_leg_dm_pair_clearable_fault(pairs[active])
        && wheel_leg_dm_pair_confirmed_disabled(pairs[1 - active]);
}

class WheelLegDmSideSchedulers {
public:
    using Action = WheelLegDmCommandScheduler::Action;

    static constexpr std::array<std::size_t, 4>
        transmit_order(bool side_bound, int selected_side) noexcept {
        // Normal graphs use [LH,RH,LK,RK]. Identification queues the parked
        // pair first so its disables precede every active-pair command.
        if (!side_bound || !wheel_leg_valid_selected_side(true, selected_side))
            return {0, 2, 1, 3};
        const auto active = static_cast<std::size_t>(selected_side);
        const auto inactive = 1 - active;
        return {2 * inactive, 2 * inactive + 1, 2 * active, 2 * active + 1};
    }

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
        return {
            schedulers_[0].next(enable[0], entered_both_down),
            schedulers_[1].next(enable[1], entered_both_down)};
    }

    // Wheel-only experiments park both pairs. Keep their 100 Hz neutral
    // feedback polls five ticks apart: two IDs on each CAN bus must not all
    // compete in the same board transfer. Never defer an explicit disable or
    // replace any system/active command with a neutral poll.
    static std::array<Action, 2>
        stagger_disabled_polls(std::array<Action, 2> actions, std::size_t tick) noexcept {
        for (std::size_t side = 0; side < actions.size(); ++side)
            if (actions[side] == Action::kNone || actions[side] == Action::kFeedbackPoll)
                actions[side] = tick % 10 == 5 * side ? Action::kFeedbackPoll : Action::kNone;
        return actions;
    }

private:
    std::array<WheelLegDmCommandScheduler, 2> schedulers_;
};

constexpr std::uint8_t
    wheel_leg_dm_tx_kind(WheelLegDmCommandScheduler::Action action, bool position_pd) noexcept {
    using Action = WheelLegDmCommandScheduler::Action;
    if (action == Action::kNone)
        return 0;
    if (action == Action::kMit)
        return position_pd ? 3 : 1;
    if (action == Action::kFeedbackPoll)
        return 4;
    return 2;
}

} // namespace rmcs_core::hardware
