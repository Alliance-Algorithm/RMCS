#pragma once

#include <algorithm>
#include <cmath>

namespace rmcs_core::hardware {

// Keep the continuous slew state independent of the quantized CAN command.
class SmallYawRobustTorqueSlew {
public:
    double update(double requested, double maximum_step, bool force_stop) {
        if (force_stop || !std::isfinite(requested) || !std::isfinite(maximum_step)
            || maximum_step < 0.0) {
            reset();
        } else if (requested * torque_ < 0.0) {
            torque_ = 0.0;
        } else if (std::abs(requested) < std::abs(torque_)) {
            torque_ = requested;
        } else {
            torque_ += std::clamp(requested - torque_, -maximum_step, maximum_step);
        }
        return torque_;
    }

    void reset() { torque_ = 0.0; }

private:
    double torque_{0.0};
};

} // namespace rmcs_core::hardware
