#pragma once

#include <cmath>
#include <limits>
#include <numbers>

namespace rmcs_core::controller::chassis::wheel_leg {

inline double recovery_yaw_error(double yaw) { return -std::remainder(yaw, std::numbers::pi); }

inline bool recovery_yaw_aligned(double yaw) {
    constexpr double tolerance = 10. * std::numbers::pi / 180.;
    return std::isfinite(yaw) && std::abs(recovery_yaw_error(yaw)) <= tolerance;
}

class RecoveryYawAlignment {
public:
    void reset() { initialized_ = false; }

    double update(double yaw, double now) {
        if (!std::isfinite(yaw) || !std::isfinite(now)) {
            reset();
            return std::numeric_limits<double>::quiet_NaN();
        }
        if (!initialized_) {
            error_ = recovery_yaw_error(yaw);
            progress_error_ = error_;
            progress_time_ = now;
            initialized_ = true;
        } else {
            error_ -= std::remainder(yaw - last_yaw_, 2. * std::numbers::pi);
        }
        last_yaw_ = yaw;
        constexpr double progress = std::numbers::pi / 180.;
        constexpr double tolerance = 10. * std::numbers::pi / 180.;
        if (std::abs(error_) <= tolerance
            || std::abs(progress_error_) - std::abs(error_) >= progress) {
            progress_error_ = error_;
            progress_time_ = now;
        } else if (now - progress_time_ >= .5) {
            error_ += error_ > 0. ? -std::numbers::pi : std::numbers::pi;
            progress_error_ = error_;
            progress_time_ = now;
        }
        return error_;
    }

private:
    bool initialized_ = false;
    double last_yaw_ = 0., error_ = 0., progress_error_ = 0., progress_time_ = 0.;
};

} // namespace rmcs_core::controller::chassis::wheel_leg
