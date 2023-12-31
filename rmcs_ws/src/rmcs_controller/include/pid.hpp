#pragma once

#include <cmath>
#include <numeric>

class PID {

public:
    PID()
        : PID(0.0, 0.0, 0.0) {}
    PID(double kp, double ki, double kd, double limit = std::numeric_limits<double>::quiet_NaN())
        : kp_(kp)
        , ki_(ki)
        , kd_(kd)
        , last_err_(std::numeric_limits<double>::quiet_NaN())
        , err_sum_(0.0)
        , setpoint_(std::numeric_limits<double>::quiet_NaN())
        , output_min_(-limit)
        , output_max_(limit)
        , kp_output_min_(std::numeric_limits<double>::quiet_NaN())
        , kp_output_max_(std::numeric_limits<double>::quiet_NaN())
        , ki_output_min_(std::numeric_limits<double>::quiet_NaN())
        , ki_output_max_(std::numeric_limits<double>::quiet_NaN())
        , kd_output_min_(std::numeric_limits<double>::quiet_NaN())
        , kd_output_max_(std::numeric_limits<double>::quiet_NaN()) {}
    virtual ~PID() {}

    double setPoint() { return setpoint_; }
    void setPoint(double setpoint) {
        last_err_ = std::numeric_limits<double>::quiet_NaN();
        setpoint_ = setpoint;
    }

    void setKp(double kp) { kp_ = kp; }
    void setKi(double ki) { ki_ = ki; }
    void setKd(double kd) { kd_ = kd; }

    void setKp(double kp, double limit) {
        kp_ = kp;
        setKpLimit(limit);
    }
    void setKi(double ki, double limit) {
        ki_ = ki;
        setKiLimit(limit);
    }
    void setKd(double kd, double limit) {
        kd_ = kd;
        setKdLimit(limit);
    }

    void setLimit(double limit) {
        output_min_ = -limit;
        output_max_ = limit;
    }
    void setKpLimit(double limit) {
        kp_output_min_ = -limit;
        kp_output_max_ = limit;
    }
    void setKiLimit(double limit) {
        ki_output_min_ = -limit;
        ki_output_max_ = limit;
    }
    void setKdLimit(double limit) {
        kd_output_min_ = -limit;
        kd_output_max_ = limit;
    }

    void setLimit(double limit_min, double limit_max) {
        output_min_ = limit_min;
        output_max_ = limit_max;
    }
    void setKpLimit(double limit_min, double limit_max) {
        kp_output_min_ = limit_min;
        kp_output_max_ = limit_max;
    }
    void setKiLimit(double limit_min, double limit_max) {
        ki_output_min_ = limit_min;
        ki_output_max_ = limit_max;
    }
    void setKdLimit(double limit_min, double limit_max) {
        kd_output_min_ = limit_min;
        kd_output_max_ = limit_max;
    }

    void clearErrorSum() { err_sum_ = 0.0; }

    double update(double measurement) {
        if (std::isnan(setpoint_))
            return 0.0;
        double err    = setpoint_ - measurement;
        double result = setLimit(kp_ * err, kp_output_min_, kp_output_max_)
                      + setLimit(ki_ * err_sum_, ki_output_min_, ki_output_max_);
        err_sum_ += err;
        if (!std::isnan(last_err_))
            result += setLimit(kd_ * (err - last_err_), kd_output_min_, kd_output_max_);
        last_err_ = err;
        return setLimit(result, output_min_, output_max_);
    }

protected:
    double setLimit(double value, double limit_min, double limit_max) {
        if (!std::isnan(limit_min) && value < limit_min)
            value = limit_min;
        else if (!std::isnan(limit_max) && value > limit_max)
            value = limit_max;
        return value;
    }

private:
    double kp_, ki_, kd_;
    double last_err_, err_sum_;
    double setpoint_;

    double output_min_, output_max_;
    double kp_output_min_, kp_output_max_;
    double ki_output_min_, ki_output_max_;
    double kd_output_min_, kd_output_max_;
};
