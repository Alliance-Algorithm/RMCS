#pragma once

#include <cmath>
#include <limits>
#include <numbers>

namespace rmcs_core::controller::gimbal::angle_wrap {

inline double wrap_to_pi(double angle) {
    if (std::isnan(angle))
        return angle;
    if (!std::isfinite(angle))
        return std::numeric_limits<double>::quiet_NaN();

    constexpr double two_pi = 2.0 * std::numbers::pi;
    angle = std::fmod(angle + std::numbers::pi, two_pi);
    if (angle < 0.0)
        angle += two_pi;
    return angle - std::numbers::pi;
}

inline double wrap_to_2pi(double angle) {
    if (std::isnan(angle))
        return angle;
    if (!std::isfinite(angle))
        return std::numeric_limits<double>::quiet_NaN();

    constexpr double two_pi = 2.0 * std::numbers::pi;
    angle = std::fmod(angle, two_pi);
    if (angle < 0.0)
        angle += two_pi;
    return angle;
}

} // namespace rmcs_core::controller::gimbal::angle_wrap
