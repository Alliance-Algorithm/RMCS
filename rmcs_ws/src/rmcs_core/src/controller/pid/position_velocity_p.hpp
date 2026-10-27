#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace rmcs_core::controller::pid {

// Model coordinates: rad, rad/s and Nm. Neither loop integrates error.
struct PositionVelocityPGains {
    double position_kp = 40.0; // 1/s
    double velocity_kp = 4.0;  // Nm s/rad
    double velocity_limit = 8.5; // rad/s, includes trajectory feedforward

    bool valid() const {
        return std::isfinite(position_kp) && position_kp > 0.0
            && std::isfinite(velocity_kp) && velocity_kp > 0.0
            && std::isfinite(velocity_limit) && velocity_limit > 0.0
            && std::isfinite(position_kp * velocity_kp);
    }
};

inline double position_velocity_p_target(
    const PositionVelocityPGains& gains, double reference, double position,
    double reference_velocity = 0.0) {
    const double target = reference_velocity + gains.position_kp * (reference - position);
    // Do not let a clamp turn invalid feedback or overflow into finite effort.
    if (!gains.valid() || !std::isfinite(reference) || !std::isfinite(position)
        || !std::isfinite(reference_velocity) || !std::isfinite(target))
        return std::numeric_limits<double>::quiet_NaN();
    return std::clamp(target, -gains.velocity_limit, gains.velocity_limit);
}

inline double position_velocity_p_effort(
    const PositionVelocityPGains& gains, double velocity_target, double measured_velocity) {
    if (!gains.valid() || !std::isfinite(velocity_target) || !std::isfinite(measured_velocity))
        return std::numeric_limits<double>::quiet_NaN();
    return gains.velocity_kp * (velocity_target - measured_velocity);
}

} // namespace rmcs_core::controller::pid
