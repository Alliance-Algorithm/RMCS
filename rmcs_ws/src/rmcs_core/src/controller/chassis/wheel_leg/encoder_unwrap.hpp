#pragma once

#include <cmath>
#include <numbers>

namespace rmcs_core::controller::chassis {

// DM MIT feedback wraps at +/- P_MAX, so one wrap moves the raw angle by
// 2 * P_MAX. The span is configuration-dependent and must not be hard-coded to
// 2 pi; callers provide the configured span for each drive.
class WheelLegEncoderUnwrap {
public:
    // Requires a finite raw angle and span > pi. Returns true when this frame
    // re-anchored the continuous angle to the raw value: the first frame, or a
    // step no joint can make between two frames (a drive restart or a
    // corrupted frame).
    bool update(double raw, double span) {
        const double step = std::remainder(raw - raw_, span);
        const bool anchor = !valid_ || !(std::abs(step) <= kMaxStep);
        value_ = anchor ? raw : value_ + step;
        raw_ = raw;
        valid_ = true;
        return anchor;
    }

    bool valid() const { return valid_; }
    double value() const { return value_; }
    double raw() const { return raw_; }

    static constexpr double kMaxStep = std::numbers::pi / 2;

private:
    double value_ = 0, raw_ = 0;
    bool valid_ = false;
};

} // namespace rmcs_core::controller::chassis
