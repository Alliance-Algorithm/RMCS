#pragma once

#include <algorithm>
#include <cmath>

namespace rmcs_core::hardware {

inline bool gimbal_velocity_stopping(bool coordinated, double joystick, double target_velocity) {
    // A centered stick freezes the heading target in coordinated mode. The
    // large axis may still need to take over the small axis's deflection.
    return coordinated ? (!std::isfinite(target_velocity) || std::abs(target_velocity) <= 1e-9)
                       : (!std::isfinite(joystick) || std::abs(joystick) <= 0.05);
}

inline double limit_gimbal_torque(
    double requested, double previous, double maximum_step, bool stopping, double measured_velocity) {
    if (!std::isfinite(requested) || !std::isfinite(previous) || !std::isfinite(maximum_step)
        || maximum_step < 0.0 || !std::isfinite(measured_velocity))
        return 0.0;

    const auto braking_only = [measured_velocity](double torque) {
        return measured_velocity > 0.0 ? std::min(torque, 0.0)
             : measured_velocity < 0.0 ? std::max(torque, 0.0) : 0.0;
    };
    if (stopping)
        requested = braking_only(requested);
    // Withdrawing propulsion is immediate; building braking torque still obeys slew.
    // Reapply this projection each cycle so braking cannot drive through zero speed.
    if (stopping || (requested < 0.0 && measured_velocity > 0.0)
        || (requested > 0.0 && measured_velocity < 0.0))
        previous = braking_only(previous);

    return std::clamp(requested, previous - maximum_step, previous + maximum_step);
}

} // namespace rmcs_core::hardware
