#pragma once

#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::chassis {

// A simultaneous remote toggle is not required: hold the disarmed request
// across DOWN/MIDDLE intermediate positions, then arm only at double MIDDLE.
class WheelLegArmSequence {
public:
    bool update(
        rmcs_msgs::Switch left, rmcs_msgs::Switch right, bool allow_spin_switch = false) noexcept {
        using rmcs_msgs::Switch;
        if (left == Switch::DOWN && right == Switch::DOWN) {
            pending_ = true;
            armed_ = false;
        } else if (left == Switch::MIDDLE && right == Switch::MIDDLE) {
            if (pending_) {
                armed_ = true;
                pending_ = false;
            }
        } else if (allow_spin_switch && armed_ && left == Switch::MIDDLE && right == Switch::DOWN) {
            // This combination can continue an armed RL session, but cannot start one.
        } else {
            armed_ = false;
            if ((left != Switch::DOWN && left != Switch::MIDDLE)
                || (right != Switch::DOWN && right != Switch::MIDDLE))
                pending_ = false;
        }
        return armed_;
    }

    bool armed() const noexcept { return armed_; }

private:
    bool pending_ = false;
    bool armed_ = false;
};

} // namespace rmcs_core::controller::chassis
