// filter/alpha_beta_angle_filter.hpp
#pragma once

#include <cmath>
#include <limits>
#include <numbers>
#include <utility>

namespace rmcs_core::filter {

class AlphaBetaAngleFilter {
public:
    AlphaBetaAngleFilter(double dt, double alpha, double beta)
        : dt_(dt), alpha_(alpha), beta_(beta) {}

    void reset() {
        initialized_ = false;
        angle_ = std::numeric_limits<double>::quiet_NaN();
        velocity_ = 0.0;
    }

    void set_gains(double alpha, double beta) {
        alpha_ = alpha;
        beta_ = beta;
    }

    std::pair<double, double> update(double measurement_rad) {
        if (!std::isfinite(measurement_rad)) {
            reset();
            return {std::numeric_limits<double>::quiet_NaN(), 0.0};
        }

        if (!initialized_) {
            initialized_ = true;
            angle_ = wrap_angle_(measurement_rad);
            velocity_ = 0.0;
            return {angle_, velocity_};
        }

        const double angle_pred = wrap_angle_(angle_ + velocity_ * dt_);
        const double residual = wrap_angle_(measurement_rad - angle_pred);

        angle_ = wrap_angle_(angle_pred + alpha_ * residual);
        velocity_ = velocity_ + (beta_ * residual) / dt_;

        return {angle_, velocity_};
    }

    double angle() const { return angle_; }
    double velocity() const { return velocity_; }
    bool initialized() const { return initialized_; }

private:
    static double wrap_angle_(double x) {
        return std::remainder(x, 2.0 * std::numbers::pi);
    }

    double dt_;
    double alpha_;
    double beta_;

    bool initialized_ = false;
    double angle_ = std::numeric_limits<double>::quiet_NaN();
    double velocity_ = 0.0;
};

} // namespace rmcs_core::filter
