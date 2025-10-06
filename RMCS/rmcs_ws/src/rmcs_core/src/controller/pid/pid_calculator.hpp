#pragma once

#include <cmath>

#include <algorithm>
#include <limits>

namespace rmcs_core::controller::pid {

class PidCalculator {
public:
    PidCalculator() { reset(); };

    PidCalculator(double kp, double ki, double kd)
        : kp(kp)
        , ki(ki)
        , kd(kd) {
        reset();
    }

    virtual ~PidCalculator() = default;

    void reset() {
        last_err_     = nan;
        err_integral_ = 0;
    }

    double update(double err) {
        if (!std::isfinite(err)) {
            return nan;
        } else {
            double control = kp * err;

            if (err < integral_split_max && err > integral_split_min) {
                control += ki * err_integral_;
                err_integral_ = std::clamp(err_integral_ + err, integral_min, integral_max);
            } else
                err_integral_ = 0;

            if (!std::isnan(last_err_))
                control += kd * (err - last_err_);
            last_err_ = err;

            return std::clamp(control, output_min, output_max);
        }
    }

    double kp, ki, kd;
    double integral_min = -inf, integral_max = inf;
    double integral_split_min = -inf, integral_split_max = inf;
    double output_min = -inf, output_max = inf;

private:
    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    double last_err_, err_integral_;
};

} // namespace rmcs_core::controller::pid