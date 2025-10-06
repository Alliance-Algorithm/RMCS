#pragma once

#include <cmath>
#include <numbers>
#include <algorithm>
#include <deque>
#include <any>
#include <vector>
#include <limits>
#include <eigen3/Eigen/Dense>

namespace rmcs_core::filter_test {

template <int variable_number = 1>
requires(variable_number > 0) class FilterBase {
public:
    using Value = std::conditional_t<
        variable_number == 1, 
        double, 
        Eigen::Vector<double, variable_number>
    >;

    virtual ~FilterBase() = default;
    
    // 统一的接口
    virtual Value update(const Value& input) = 0;
    virtual void reset() = 0;
    virtual void configure(const std::any& config) = 0;
    
    // 公共工具方法
    static auto get_nan() {
        if constexpr (variable_number == 1)
            return std::numeric_limits<double>::quiet_NaN();
        else
            return Eigen::Vector<double, variable_number>::Constant(
                std::numeric_limits<double>::quiet_NaN());
    }

protected:
    // NaN处理工具方法
    static void exclude_nan(double& output, double input) {
        output = std::isnan(output) ? input : output;
    }

    template <int n>
    static void exclude_nan(Eigen::Vector<double, n>& output, const Eigen::Vector<double, n>& input) {
        output = (output.array().isNaN()).select(input, output);
    }

    Value previous_output_;
};

} // namespace rmcs_core::filter