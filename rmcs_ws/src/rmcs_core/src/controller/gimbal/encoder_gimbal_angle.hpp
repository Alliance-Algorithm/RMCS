#pragma once

#include <cmath>
#include <numbers>
#include <numeric>

namespace rmcs_core::controller::gimbal {

inline double encoder_angle_to_clockwise_yaw(double encoder_angle) {
    constexpr double two_pi = 2.0 * std::numbers::pi;
    double clockwise_yaw = std::fmod(two_pi - encoder_angle, two_pi);
    if (clockwise_yaw < 0.0)
        clockwise_yaw += two_pi;
    return clockwise_yaw;
}

inline double clockwise_yaw_control_error(double target_clockwise_yaw, double current_clockwise_yaw) {
    // The motor feedback is reversed, so a positive clockwise target shift
    // must become a negative error for the PID/motor command chain.
    return current_clockwise_yaw - target_clockwise_yaw;
}

inline double
    encoder_angle_to_bounded_pitch(double encoder_angle, double upper_limit, double lower_limit) {
    const double center = std::midpoint(upper_limit, lower_limit);
    return center + std::remainder(encoder_angle - center, 2.0 * std::numbers::pi);
}

} // namespace rmcs_core::controller::gimbal
