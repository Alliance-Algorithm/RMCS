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
//
// Two opt-in refinements exist for long identification sweeps: `latch_run` keeps a
// started run going until DOWN/DOWN (a released stick no longer aborts and re-anchors
// the trajectory), and `remote_timeout_s` holds the last valid switches through a
// short receiver gap instead of faulting on a single stale frame. Both default to the
// original level-triggered behaviour.
class WheelLegSweepSession {
public:
    explicit WheelLegSweepSession(bool joints)
        : joints_(joints) {}

    bool update(
        double now, bool remote_fresh, rmcs_msgs::Switch left, rmcs_msgs::Switch right,
        SweepStop health) {
        using rmcs_msgs::Switch;
        if (remote_fresh && left != Switch::UNKNOWN && right != Switch::UNKNOWN) {
            last_left_ = left;
            last_right_ = right;
            last_fresh_time_ = now;
        } else if (!(remote_timeout_s > 0) || now - last_fresh_time_ > remote_timeout_s) {
            fault(SweepStop::kRemoteLost);
            latched_ = false;
            return false;
        } else {
            // Hold the last valid command through a short receiver gap.
            left = last_left_;
            right = last_right_;
        }
        if (left == Switch::DOWN && right == Switch::DOWN) {
            state = SweepState::kResetReady;
            reason = SweepStop::kBothDown;
            mode = 0;
            completed = false;
            latched_ = false;
            return false;
        }
        if (health != SweepStop::kNone) {
            fault(health);
            latched_ = false;
            return false;
        }
        const bool middle_now = left == Switch::MIDDLE && right == Switch::MIDDLE;
        if (latched_) {
            // A latched run ignores switch jitter, including a release back to middle
            // and any other mode combination. DOWN/DOWN remains the explicit abort.
            if (state == SweepState::kRunning)
                return false;
            if (state == SweepState::kComplete) {
                if (middle_now) {
                    state = SweepState::kStandby;
                    reason = SweepStop::kNone;
                    completed = false;
                    latched_ = false;
                }
                return false;
            }
        }
        const int selected = joints_ ? (left == Switch::UP && right == Switch::MIDDLE   ? 1
                                        : left == Switch::MIDDLE && right == Switch::UP ? 2
                                        : left == Switch::UP && right == Switch::UP     ? 3
                                                                                        : 0)
                                     : (middle_now ? 0 : 3);
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
        latched_ = latch_run;
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
    // Opt-in: run until DOWN/DOWN instead of following the live switches.
    bool latch_run = false;
    // Opt-in: hold the last valid switches for at most this long (0 = fault at once).
    double remote_timeout_s = 0;

private:
    bool joints_;
    bool latched_ = false;
    rmcs_msgs::Switch last_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_right_ = rmcs_msgs::Switch::UNKNOWN;
    double last_fresh_time_ = 0;
};

} // namespace rmcs_core::controller::identification
