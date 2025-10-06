#pragma once

#include "filter_base.hpp"

namespace rmcs_core::filter_test {

template <int variable_number = 1>
requires(variable_number > 0) class LowPassFilter : public filter_test::FilterBase<variable_number> {
public:
    using Value = typename filter_test::FilterBase<variable_number>::Value;

    explicit LowPassFilter(double alpha = 0.2)
        : alpha_(alpha) {
        this->reset();
    }

    LowPassFilter(double cutoff_frequency, double sampling_frequency) {
        set_cutoff(cutoff_frequency, sampling_frequency);
        this->reset();
    }

    void reset() override { 
        this->previous_output_ = this->get_nan(); 
    }

    Value update(const Value& input) override {
        Value output = alpha_ * input + (1.0 - alpha_) * this->previous_output_;
        this->exclude_nan(output, input);
        this->previous_output_ = output;
        return output;
    }

    void configure(const std::any& config) override {
        if (auto alpha = std::any_cast<double>(&config)) {
            alpha_ = *alpha;
        }
        // 可以扩展其他配置方式
    }

    void set_alpha(double alpha) { alpha_ = alpha; }

    void set_cutoff(double cutoff_freq, double sampling_freq) {
        double dt = 1.0 / sampling_freq;
        double rc = 1.0 / (2 * std::numbers::pi * cutoff_freq);
        alpha_ = dt / (dt + rc);
    }

    double get_alpha() const { return alpha_; }

private:
    double alpha_;
};
};

 // namespace rmcs_core::filter