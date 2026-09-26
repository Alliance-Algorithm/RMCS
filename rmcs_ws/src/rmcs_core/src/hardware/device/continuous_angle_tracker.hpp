#pragma once

#include <cmath>

namespace rmcs_core::hardware::device {

class ContinuousAngleTracker {
public:
    void reset() {
        initialized_ = false;
        previous_raw_ = 0.0;
        continuous_ = 0.0;
    }

    double update(double raw, double wrap_period) {
        if (!initialized_) {
            initialized_ = true;
            previous_raw_ = continuous_ = raw;
            return continuous_;
        }

        double delta = raw - previous_raw_;
        if (wrap_period > 0.0 && std::abs(delta) > wrap_period / 2.0) {
            const double wrapped_delta = std::remainder(delta, wrap_period);
            // A large but ambiguous step is left unchanged for the caller's fault check.
            if (std::abs(wrapped_delta) < 0.5)
                delta = wrapped_delta;
        }
        previous_raw_ = raw;
        continuous_ += delta;
        return continuous_;
    }

private:
    bool initialized_ = false;
    double previous_raw_ = 0.0;
    double continuous_ = 0.0;
};

} // namespace rmcs_core::hardware::device
