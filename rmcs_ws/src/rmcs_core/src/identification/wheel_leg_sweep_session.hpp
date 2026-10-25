#pragma once

#include <cstdint>

#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::identification {

enum class SweepState : int {
    kResetRequired = 0,
    kResetReady = 1,
    kStandby = 2,
    kRunning = 3,
    kComplete = 4,
    kInterrupted = 5,
    kFault = 6
};
enum class SweepStop : int {
    kNone = 0,
    kBothDown = 1,
    kRemoteLost = 2,
    kInvalidSwitch = 3,
    kOperator = 4,
    kModeChanged = 5,
    kFinished = 6,
    kFeedbackTimeout = 7,
    kNonfinite = 8,
    kDriveFault = 9,
    kEnableLost = 10
};

// Level-triggered remote handling, like ChassisController. No arming sequence,
// debounce, drive acknowledgement timeout, or latched restart interlock.
class WheelLegSweepSession {
public:
    explicit WheelLegSweepSession(bool joints)
        : joints_(joints) {}

    bool update(
        double now, bool remote_fresh, rmcs_msgs::Switch left, rmcs_msgs::Switch right,
        SweepStop health) {
        using rmcs_msgs::Switch;
        if (!remote_fresh || left == Switch::UNKNOWN || right == Switch::UNKNOWN) {
            fault(SweepStop::kRemoteLost);
            return false;
        }
        if (left == Switch::DOWN && right == Switch::DOWN) {
            state = SweepState::kResetReady;
            reason = SweepStop::kBothDown;
            mode = 0;
            completed = false;
            return false;
        }
        if (health != SweepStop::kNone) {
            fault(health);
            return false;
        }
        const bool middle = left == Switch::MIDDLE && right == Switch::MIDDLE;
        const int selected = joints_ ? (left == Switch::UP && right == Switch::MIDDLE   ? 1
                                        : left == Switch::MIDDLE && right == Switch::UP ? 2
                                        : left == Switch::UP && right == Switch::UP     ? 3
                                                                                        : 0)
                                     : (middle ? 0 : 3);
        if (!selected) {
            reason = state == SweepState::kRunning ? SweepStop::kOperator : SweepStop::kNone;
            state = SweepState::kStandby;
            mode = 0;
            completed = false;
            return false;
        }
        if (state == SweepState::kRunning && mode == selected)
            return false;
        mode = selected;
        ++run_id;
        start_time = now;
        completed = false;
        reason = SweepStop::kNone;
        state = SweepState::kRunning;
        return true;
    }

    void complete() {
        state = SweepState::kComplete;
        completed = true;
        reason = SweepStop::kFinished;
    }

    void fault(SweepStop why) {
        state = SweepState::kFault;
        reason = why;
        completed = false;
    }

    bool enabled() const {
        return state == SweepState::kStandby || state == SweepState::kRunning
            || state == SweepState::kComplete || state == SweepState::kInterrupted;
    }
    SweepState state = SweepState::kResetRequired;
    SweepStop reason = SweepStop::kNone;
    int mode = 0;
    std::uint64_t run_id = 0;
    double start_time = 0;
    bool completed = false;

private:
    bool joints_;
};

} // namespace rmcs_core::controller::identification
