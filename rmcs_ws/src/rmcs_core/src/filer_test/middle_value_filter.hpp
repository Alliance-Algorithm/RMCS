#pragma once

#include "filter_base.hpp"

namespace rmcs_core::filter_test {

template <int variable_number = 1>
requires(variable_number > 0) class MedianFilter : public filter_test::FilterBase<variable_number>{
public:
    using Value = typename filter_test::FilterBase<variable_number>::Value;

    explicit MedianFilter(size_t window_size = 7)
        : window_size_(window_size) {
        // 确保窗口大小为奇数
        if (window_size_ % 2 == 0) {
            window_size_ += 1;
        }
        this->reset();
    }

    void reset() override {
        buffer_.clear();
        this->previous_output_ = this->get_nan();
    }

    Value update(const Value& input) override {
        // 将新数据加入缓冲区
        buffer_.push_back(input);
        
        // 保持缓冲区大小不超过窗口大小
        if (buffer_.size() > window_size_) {
            buffer_.pop_front();
        }
        
        // 处理NaN值并计算中值
        Value output = compute_median(input);
        this->exclude_nan(output, input);
        this->previous_output_ = output;
        return output;
    }

    void configure(const std::any& config) override {
        if (auto window_size = std::any_cast<size_t>(&config)) {
            set_window_size(*window_size);
        }
    }

    void set_window_size(size_t window_size) {
        window_size_ = (window_size % 2 == 0) ? window_size + 1 : window_size;
        reset();
    }

    size_t get_window_size() const { return window_size_; }

private:
    Value compute_median(const Value& input) {
        if constexpr (variable_number == 1) {
            // 标量版本
            if (std::isnan(input)) {
                return this->get_nan();
            }
            
            std::vector<double> sorted_buffer(buffer_.begin(), buffer_.end());
            std::sort(sorted_buffer.begin(), sorted_buffer.end());
            
            return sorted_buffer.empty() ? input : sorted_buffer[sorted_buffer.size() / 2];
        } else {
            // 向量版本
            Value output;
            
            for (int i = 0; i < variable_number; ++i) {
                if (std::isnan(input[i])) {
                    output[i] = std::numeric_limits<double>::quiet_NaN();
                    continue;
                }
                
                std::vector<double> dimension_values;
                for (const auto& val : buffer_) {
                    if (!std::isnan(val[i])) {
                        dimension_values.push_back(val[i]);
                    }
                }
                
                if (!dimension_values.empty()) {
                    std::sort(dimension_values.begin(), dimension_values.end());
                    output[i] = dimension_values[dimension_values.size() / 2];
                } else {
                    output[i] = input[i];
                }
            }
            
            return output;
        }
    }

    size_t window_size_;
    std::deque<Value> buffer_;
};

} 