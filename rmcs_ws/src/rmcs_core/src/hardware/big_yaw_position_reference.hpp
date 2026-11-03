#pragma once

#include <cmath>
#include <cstdint>

namespace rmcs_core::hardware {

// Check an actual CAN sample interval rather than the controller's update
// period. Keep the historical one-radian/s envelope unless configured for a
// faster mode; the sample-gap and encoder-tolerance protections stay fixed.
inline bool big_yaw_feedback_continuous(
    double angle_delta, int64_t elapsed_ns, double max_velocity_rad_s = 1.0) noexcept {
    return elapsed_ns > 0 && elapsed_ns <= 50'000'000
        && std::isfinite(angle_delta) && std::isfinite(max_velocity_rad_s)
        && max_velocity_rad_s > 0.0
        && std::abs(angle_delta) <= max_velocity_rad_s * (elapsed_ns * 1e-9) + 0.03;
}

// Keep position-fault recovery independent of command permission: a fault blocks
// commands, but valid stationary feedback plus double-down must still rearm it.
class BigYawPositionReference {
public:
    void mark_fault() noexcept {
        tracking_fault_ = true;
        origin_valid_ = false;
    }

    bool update(bool both_down, bool position_valid, double angle, double velocity) noexcept {
        const bool feedback_valid = position_valid && std::isfinite(angle)
                                 && std::isfinite(velocity);
        if (both_down && feedback_valid && std::abs(velocity) <= 0.1
            && (tracking_fault_ || !origin_valid_)) {
            origin_ = angle;
            origin_valid_ = true;
            tracking_fault_ = false;
        }
        return feedback_valid && !tracking_fault_ && origin_valid_;
    }

    bool tracking_fault() const noexcept { return tracking_fault_; }
    bool origin_valid() const noexcept { return origin_valid_; }
    // Only meaningful while origin_valid() is true.
    double origin() const noexcept { return origin_; }

private:
    bool tracking_fault_ = false;
    bool origin_valid_ = false;
    double origin_ = 0.0;
};

} // namespace rmcs_core::hardware
