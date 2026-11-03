#pragma once

#include <algorithm>
#include <cmath>

namespace rmcs_core::controller::gimbal {

class VelocityReference {
public:
    void update(double desired_velocity, double max_acceleration, double dt, bool active) {
        if (!active || !std::isfinite(desired_velocity) || desired_velocity == 0.0
            || !std::isfinite(max_acceleration) || max_acceleration <= 0.0
            || !std::isfinite(dt) || dt <= 0.0) {
            velocity_ = 0.0;
            acceleration_ = 0.0;
            return;
        }
        // A stop or reversal must not retain the old direction's velocity reference.
        if ((desired_velocity > 0.0 && velocity_ < 0.0)
            || (desired_velocity < 0.0 && velocity_ > 0.0))
            velocity_ = 0.0;
        const double max_delta = max_acceleration * dt;
        const double delta = std::clamp(desired_velocity - velocity_, -max_delta, max_delta);
        velocity_ += delta;
        acceleration_ = delta / dt;
    }

    double velocity() const { return velocity_; }
    double acceleration() const { return acceleration_; }

private:
    double velocity_{0.0};
    double acceleration_{0.0};
};

} // namespace rmcs_core::controller::gimbal
