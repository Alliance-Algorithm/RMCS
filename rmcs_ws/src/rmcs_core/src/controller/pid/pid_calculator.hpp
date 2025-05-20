#pragma once

#include <cmath>
#include <algorithm>
#include <limits>

namespace rmcs_core::controller::pid {

class PidCalculator {
public:
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
        }

        if (err >= deadzone_min && err <= deadzone_max) {
            err_integral_ = 0;
            err = 0;
            last_err_ = err;
            return 0.0;
        }

        double control = kp * err;

        if (err > integral_split_max || err < integral_split_min) {
            err_integral_ = 0;
        } else {
            err_integral_ = std::clamp(err_integral_ + err, integral_min, integral_max);
            control += ki * err_integral_;
        }

        if (!std::isnan(last_err_)) {
            control += kd * (err - last_err_);
        }
        last_err_ = err;

        return std::clamp(control, output_min, output_max);
    }

public:
    double kp, ki, kd;
    double integral_min = -inf, integral_max = inf;
    double integral_split_min = -inf, integral_split_max = inf;
    double output_min = -inf, output_max = inf;
    double deadzone_min = 0.0, deadzone_max = 0.0;  

private:
    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    double last_err_, err_integral_;
};

} // namespace rmcs_core::controller::pid
