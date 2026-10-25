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

// Remote interlock only. Physical drive acknowledgement and sensor health are
// supplied by the controller; no motion envelope is part of this state machine.
class WheelLegSweepSession {
public:
    explicit WheelLegSweepSession(bool joints)
        : joints_(joints) {}

    bool update(
        double now, bool remote_fresh, rmcs_msgs::Switch left, rmcs_msgs::Switch right,
        SweepStop health, bool drives_ready) {
        using rmcs_msgs::Switch;
        if (!remote_fresh || left == Switch::UNKNOWN || right == Switch::UNKNOWN) {
            fault(SweepStop::kRemoteLost);
            return false;
        }
        if (left == Switch::DOWN && right == Switch::DOWN) {
            state = SweepState::kResetReady;
            reason = SweepStop::kBothDown;
            mode = candidate_ = 0;
            completed = false;
            ready_seen_ = false;
            allow_start_ = false;
            return false;
        }
        const bool middle = left == Switch::MIDDLE && right == Switch::MIDDLE;
        // A DOWN/DOWN reset is already acknowledged. The two physical switches
        // reach MIDDLE on separate DR16 frames: keep waiting with zero output
        // through those intermediate positions. Once armed, a DOWN switch still
        // disarms immediately and requires another complete DOWN/DOWN reset.
        if (state == SweepState::kResetReady && !middle)
            return false;
        if ((left != Switch::MIDDLE && left != Switch::UP)
            || (right != Switch::MIDDLE && right != Switch::UP)) {
            fault(SweepStop::kInvalidSwitch);
            return false;
        }
        if (state == SweepState::kResetRequired || state == SweepState::kFault)
            return false;
        if (state == SweepState::kResetReady) {
            state = SweepState::kStandby;
            arm_time_ = now;
            allow_start_ = true;
        }
        if (health != SweepStop::kNone) {
            fault(health);
            return false;
        }
        if ((ready_seen_ && !drives_ready) || (!drives_ready && now - arm_time_ > 2.0)) {
            fault(SweepStop::kEnableLost);
            return false;
        }
        ready_seen_ = ready_seen_ || drives_ready;
        const int selected =
            left == Switch::UP ? (right == Switch::UP ? 3 : 1) : (right == Switch::UP ? 2 : 0);
        if (state == SweepState::kRunning) {
            if (selected != mode) {
                state = SweepState::kInterrupted;
                reason = middle ? SweepStop::kOperator : SweepStop::kModeChanged;
                allow_start_ = false;
                candidate_ = 0;
            }
            return false;
        }
        if (middle) {
            state = SweepState::kStandby;
            allow_start_ = true;
            candidate_ = 0;
            return false;
        }
        if (!allow_start_ || !drives_ready || (!joints_ && selected != 3)) {
            candidate_ = 0;
            return false;
        }
        if (candidate_ != selected) {
            candidate_ = selected;
            candidate_time_ = now;
        }
        if (now - candidate_time_ < .300 - 1e-9)
            return false;
        mode = selected;
        ++run_id;
        start_time = now;
        completed = false;
        reason = SweepStop::kNone;
        state = SweepState::kRunning;
        allow_start_ = false;
        candidate_ = 0;
        return true;
    }

    void complete() {
        state = SweepState::kComplete;
        completed = true;
        reason = SweepStop::kFinished;
        allow_start_ = false;
    }

    void fault(SweepStop why) {
        state = SweepState::kFault;
        reason = why;
        completed = false;
        allow_start_ = ready_seen_ = false;
        candidate_ = 0;
    }

    bool enabled() const {
        return state == SweepState::kStandby || state == SweepState::kRunning
            || state == SweepState::kComplete || state == SweepState::kInterrupted;
    }
    bool ready_seen() const { return ready_seen_; }
    SweepState state = SweepState::kResetRequired;
    SweepStop reason = SweepStop::kNone;
    int mode = 0;
    std::uint64_t run_id = 0;
    double start_time = 0;
    bool completed = false;

private:
    bool joints_, allow_start_ = false, ready_seen_ = false;
    int candidate_ = 0;
    double candidate_time_ = 0, arm_time_ = 0;
};

} // namespace rmcs_core::controller::identification
