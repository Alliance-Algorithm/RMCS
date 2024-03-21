#pragma once

#include <cmath>
#include <limits>

class PidCalculator {
public:
    PidCalculator(double kp = 0, double ki = 0, double kd = 0)
        : kp(kp)
        , ki(ki)
        , kd(kd) {}

    virtual ~PidCalculator() = default;

    double update(double err) {
        if (std::isnan(err)) {
            return nan;
        } else {
            double control = kp * err + ki * err_integral_;
            err_integral_  = limit(err_integral_ + err, integral_min, integral_max);

            if (!std::isnan(last_err_))
                control += kd * (err - last_err_);
            last_err_ = err;

            return limit(control, output_min, output_max);
        }
    }

    double kp, ki, kd;
    double integral_min = -inf, integral_max = inf;
    double output_min = -inf, output_max = inf;

protected:
    double limit(double value, double min, double max) {
        if (value < min)
            return min;
        if (value > max)
            return max;
        return value;
    }

    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    double last_err_ = nan, err_integral_ = 0;
};