#pragma once

#include <cmath>

#include <numbers>

#include <eigen3/Eigen/Dense>

namespace rmcs_core::filter {

template <int variable_number = 1>
requires(variable_number > 0) class LowPassFilter {
    using Value =
        std::conditional_t<variable_number == 1, double, Eigen::Vector<double, variable_number>>;

    static auto get_nan() {
        if constexpr (variable_number == 1)
            return std::numeric_limits<double>::quiet_NaN();
        else
            return Eigen::Vector<double, variable_number>::Constant(
                std::numeric_limits<double>::quiet_NaN());
    }

    static void exclude_nan(double& output, double input) {
        output = std::isnan(output) ? input : output;
    }

    template <int n>
    static void
        exclude_nan(Eigen::Vector<double, n>& output, const Eigen::Vector<double, n>& input) {
        output = (output.array().isNaN()).select(input, output);
    }

public:
    explicit LowPassFilter(double alpha)
        : alpha_(alpha)
        , previous_output_(get_nan()) {}

    LowPassFilter(double cutoff_frequency, double sampling_frequency)
        : previous_output_(get_nan()) {
        set_cutoff(cutoff_frequency, sampling_frequency);
    }

    void reset() { previous_output_ = get_nan(); }

    Value update(const Value& input) {
        Value output = alpha_ * input + (1.0 - alpha_) * previous_output_;
        exclude_nan(output, input);
        previous_output_ = output;
        return output;
    }

    void set_alpha(double alpha) { alpha_ = alpha; }

    void set_cutoff(double cutoff_freq, double sampling_freq) {
        double dt = 1.0 / sampling_freq;
        double rc = 1.0 / (2 * std::numbers::pi * cutoff_freq);
        alpha_    = dt / (dt + rc);
    }

private:
    double alpha_;
    Value previous_output_;
};

} // namespace rmcs_core::filter