#pragma once

#include <cmath>

#include <algorithm>
#include <limits>
#include <optional>
#include <string>

#include <rclcpp/node.hpp>

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
        last_err_ = nan;
        last_measurement_ = nan;
        err_integral_ = 0;
    }

    // Derivative on the error. A reference that steps between calls injects a
    // derivative kick; prefer the measurement overload for stepped references.
    double update(double err) {
        if (!std::isfinite(err)) {
            reset();
            return nan;
        }
        double control = accumulate_(err);

        if (!std::isnan(last_err_))
            control += kd * (err - last_err_);
        last_err_ = err;

        return std::clamp(control, output_min, output_max);
    }

    // Derivative on the measurement. `measurement` is the signal err refers to,
    // so d(reference - measurement)/dt equals -d(measurement)/dt while the
    // reference is held. Stepped references therefore cannot inject a kick.
    double update(double err, double measurement) {
        if (!std::isfinite(err) || !std::isfinite(measurement)) {
            reset();
            return nan;
        }
        double control = accumulate_(err);

        if (!std::isnan(last_measurement_))
            control += kd * (last_measurement_ - measurement);
        last_err_ = err;
        last_measurement_ = measurement;

        return std::clamp(control, output_min, output_max);
    }

    double kp, ki, kd;
    double integral_min = -inf, integral_max = inf;
    double integral_split_min = -inf, integral_split_max = inf;
    double output_min = -inf, output_max = inf;

private:
    // Proportional term plus the integral accumulated by the previous cycle,
    // then this error is integrated for the next cycle.
    double accumulate_(double err) {
        double control = kp * err;

        if (err < integral_split_max && err > integral_split_min) {
            control += ki * err_integral_;
            err_integral_ = std::clamp(err_integral_ + err, integral_min, integral_max);
        } else
            err_integral_ = 0;

        return control;
    }

    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    double last_err_ = nan, last_measurement_ = nan, err_integral_ = 0.0;
};

inline auto make_pid_calculator(
    rclcpp::Node& node, const std::string& prefix, //
    std::optional<double> kp_default = std::nullopt,
    std::optional<double> ki_default = std::nullopt,
    std::optional<double> kd_default = std::nullopt) {

    const auto parameter_or_default =
        [&node](const std::string& name, std::optional<double> default_value) {
            if (default_value.has_value() && !node.has_parameter(name))
                node.declare_parameter<double>(name, *default_value);
            return node.get_parameter(name).as_double();
        };

    auto calculator = PidCalculator{
        parameter_or_default(prefix + "kp", kp_default),
        parameter_or_default(prefix + "ki", ki_default),
        parameter_or_default(prefix + "kd", kd_default),
    };

    node.get_parameter(prefix + "integral_min", calculator.integral_min);
    node.get_parameter(prefix + "integral_max", calculator.integral_max);
    node.get_parameter(prefix + "integral_split_min", calculator.integral_split_min);
    node.get_parameter(prefix + "integral_split_max", calculator.integral_split_max);
    node.get_parameter(prefix + "output_min", calculator.output_min);
    node.get_parameter(prefix + "output_max", calculator.output_max);
    return calculator;
}

} // namespace rmcs_core::controller::pid
