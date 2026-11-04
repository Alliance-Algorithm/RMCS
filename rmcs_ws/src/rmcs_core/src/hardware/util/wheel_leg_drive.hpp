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
        for (const auto axis : axis_order)
            if (actions[axis / 2] == Action::kDisable)
                send(axis, Action::kFeedbackPoll);
    }

    std::array<Action, 2>
        next(const std::array<bool, 2>& enable, bool entered_both_down = false) noexcept {
        scheduled_side_ = 1 - scheduled_side_;
        std::array actions{Action::kNone, Action::kNone};
        for (std::size_t side = 0; side < actions.size(); ++side) {
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
