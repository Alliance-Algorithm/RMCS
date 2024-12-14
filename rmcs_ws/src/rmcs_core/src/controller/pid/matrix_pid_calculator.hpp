#pragma once

#include <cmath>

#include <algorithm>
#include <limits>

#include <eigen3/Eigen/Dense>

namespace rmcs_core::controller::pid {

template <size_t n, bool use_matrix_gain = false>
class MatrixPidCalculator {
    using Vector = Eigen::Vector<double, n>;
    using Gain   = std::conditional<use_matrix_gain, Eigen::Matrix<double, n, n>, double>::type;

public:
    MatrixPidCalculator(Gain kp, Gain ki, Gain kd)
        : kp(std::move(kp))
        , ki(std::move(ki))
        , kd(std::move(kd)) {
        integral_min.setConstant(-inf);
        integral_max.setConstant(inf);
        output_min.setConstant(-inf);
        output_max.setConstant(inf);
        reset();
    }

    virtual ~MatrixPidCalculator() = default;

    void reset() {
        last_err_.setConstant(nan);
        err_integral_ = Vector::Zero();
    }

    Vector update(Vector err) {
        if (!std::isfinite(err[0])) {
            Vector nan_vector;
            nan_vector.setConstant(nan);
            return nan_vector;
        } else {
            Vector control = kp * err + ki * err_integral_;
            err_integral_  = clamp(err_integral_ + err, integral_min, integral_max);

            if (!std::isnan(last_err_[0]))
                control += kd * (err - last_err_);
            last_err_ = err;

            return clamp(control, output_min, output_max);
        }
    }

    Gain kp, ki, kd;
    Vector integral_min, integral_max;
    Vector output_min, output_max;

private:
    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    static Vector clamp(const Vector& value, const Vector& min, const Vector& max) {
        return value.cwiseMax(min).cwiseMin(max);
    }

    Vector last_err_, err_integral_;
};

} // namespace rmcs_core::controller::pid