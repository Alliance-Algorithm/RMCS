#pragma once

#include <chrono>

#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::identification {

// Both switches must stay DOWN for the dwell, but their later transition to
// MIDDLE need not happen on the same executor tick. Mixed DOWN/MIDDLE never
// enables an actuator; UP, UNKNOWN and remote loss cancel the pending arm.
class WheelLegArmDwell {
public:
    using Clock = std::chrono::steady_clock;
    using Switch = rmcs_msgs::Switch;

    bool update(
        Clock::time_point now, bool remote_fresh, Switch left, Switch right,
        double dwell_s) noexcept {
        if (!remote_fresh || (left != Switch::DOWN && left != Switch::MIDDLE)
            || (right != Switch::DOWN && right != Switch::MIDDLE)) {
            reset();
            return false;
        }
        if (left == Switch::DOWN && right == Switch::DOWN) {
            if (down_since_ == Clock::time_point{})
                down_since_ = now;
            if (now >= down_since_
                && std::chrono::duration<double>(now - down_since_).count() >= dwell_s)
                ready_ = true;
            return false;
        }
        if (left == Switch::MIDDLE && right == Switch::MIDDLE) {
            const bool armed = ready_;
            reset();
            return armed;
        }
        if (!ready_)
            down_since_ = {};
        return false;
    }

    void reset() noexcept {
        ready_ = false;
        down_since_ = {};
    }

private:
    Clock::time_point down_since_{};
    bool ready_ = false;
};

} // namespace rmcs_core::controller::identification
