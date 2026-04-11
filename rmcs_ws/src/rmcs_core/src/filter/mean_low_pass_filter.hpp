#pragma once

#include <algorithm>
#include <cstddef>
#include <deque>

#include <eigen3/Eigen/Dense>

#include "filter/low_pass_filter.hpp"

namespace rmcs_core::filter {

template <int variable_number = 1>
requires(variable_number > 0) class MeanLowPassFilter {
    using Value =
        std::conditional_t<variable_number == 1, double, Eigen::Vector<double, variable_number>>;

    static Value zero_value() {
        if constexpr (variable_number == 1)
            return 0.0;
        else
            return Eigen::Vector<double, variable_number>::Zero();
    }

public:
    MeanLowPassFilter(std::size_t window_size, double alpha)
        : window_size_(std::max<std::size_t>(1, window_size))
        , running_sum_(zero_value())
        , low_pass_filter_(alpha) {}

    MeanLowPassFilter(std::size_t window_size, double cutoff_frequency, double sampling_frequency)
        : window_size_(std::max<std::size_t>(1, window_size))
        , running_sum_(zero_value())
        , low_pass_filter_(cutoff_frequency, sampling_frequency) {}

    void reset() {
        samples_.clear();
        running_sum_ = zero_value();
        low_pass_filter_.reset();
    }

    Value update(const Value& input) {
        Value filtered_value = low_pass_filter_.update(input);

        if (samples_.size() == window_size_) {
            running_sum_ -= samples_.front();
            samples_.pop_front();
        }

        samples_.push_back(filtered_value);
        running_sum_ += filtered_value;

        return running_sum_ / static_cast<double>(samples_.size());
    }

    void set_alpha(double alpha) { low_pass_filter_.set_alpha(alpha); }

    void set_cutoff(double cutoff_frequency, double sampling_frequency) {
        low_pass_filter_.set_cutoff(cutoff_frequency, sampling_frequency);
    }

    void set_window_size(std::size_t window_size) {
        window_size_ = std::max<std::size_t>(1, window_size);
        reset();
    }

private:
    std::size_t window_size_;
    std::deque<Value> samples_;
    Value running_sum_;
    LowPassFilter<variable_number> low_pass_filter_;
};

} // namespace rmcs_core::filter
