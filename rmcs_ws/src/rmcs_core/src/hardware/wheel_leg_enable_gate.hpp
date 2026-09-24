#pragma once

#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::hardware {

constexpr bool wheel_leg_drive_allowed(
    bool dr16_fresh, rmcs_msgs::Switch left, rmcs_msgs::Switch right, bool require_request,
    bool requested) noexcept {
    return dr16_fresh && left == rmcs_msgs::Switch::MIDDLE && right == rmcs_msgs::Switch::MIDDLE
        && (!require_request || requested);
}

} // namespace rmcs_core::hardware
