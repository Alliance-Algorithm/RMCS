#pragma once

#include <cmath>
#include <numeric>

class PID {

public:
    PID()
        : PID(0.0, 0.0, 0.0) {}
    PID(double kp, double ki, double kd)
        : kp_(kp)
        , ki_(ki)
        , kd_(kd)
        , kp_min_(std::numeric_limits<double>::quiet_NaN())
        , kp_max_(std::numeric_limits<double>::quiet_NaN())
        , ki_min_(std::numeric_limits<double>::quiet_NaN())
        , ki_max_(std::numeric_limits<double>::quiet_NaN())
        , kd_min_(std::numeric_limits<double>::quiet_NaN())
        , kd_max_(std::numeric_limits<double>::quiet_NaN())
        , last_err_(std::numeric_limits<double>::quiet_NaN())
        , err_sum_(0.0)
        , setpoint_(std::numeric_limits<double>::quiet_NaN()) {}
    virtual ~PID() {}

    double setPoint() { return setpoint_; }

    void setPoint(double setpoint) {
        last_err_ = std::numeric_limits<double>::quiet_NaN();
        setpoint_ = setpoint;
    }

    void setKp(double kp, double limit = std::numeric_limits<double>::quiet_NaN()) {
        kp_ = kp;
        setKpLimit(limit);
    }

    void setKi(double ki, double limit = std::numeric_limits<double>::quiet_NaN()) {
        ki_ = ki;
        setKiLimit(limit);
        clearErrorSum();
    }

    void setKd(double kd, double limit = std::numeric_limits<double>::quiet_NaN()) {
        kd_ = kd;
        setKdLimit(limit);
    }

    void setKpLimit(double limit) {
        if (!std::isnan(limit)) {
            kp_min_ = -limit;
            kp_max_ = limit;
        }
    }

    void setKiLimit(double limit) {
        if (!std::isnan(limit)) {
            ki_min_ = -limit;
            ki_max_ = limit;
        }
    }

    void setKdLimit(double limit) {
        if (!std::isnan(limit)) {
            kd_min_ = -limit;
            kd_max_ = limit;
        }
    }

    void setKpLimit(double limit_min, double limit_max) {
        if (!std::isnan(limit_min))
            kp_min_ = limit_min;
        if (!std::isnan(limit_max))
            kp_max_ = limit_max;
    }

    void setKiLimit(double limit_min, double limit_max) {
        if (!std::isnan(limit_min))
            ki_min_ = limit_min;
        if (!std::isnan(limit_max))
            ki_max_ = limit_max;
    }

    void setKdLimit(double limit_min, double limit_max) {
        if (!std::isnan(limit_min))
            kd_min_ = limit_min;
        if (!std::isnan(limit_max))
            kd_max_ = limit_max;
    }

    void clearErrorSum() { err_sum_ = 0.0; }

    double update(double measurement) {
        if (std::isnan(setpoint_)) {
            return 0.0;
        }
        double err = setpoint_ - measurement;
        double result =
            setLimit(kp_ * err, kp_min_, kp_max_) + setLimit(ki_ * err_sum_, ki_min_, ki_max_);
        err_sum_ += err;
        if (std::isnan(last_err_))
            last_err_ = err;
        else {
            result += setLimit(kd_ * (err - last_err_), kd_min_, kd_max_);
            last_err_ = err;
        }
        return result;
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
    double kp_min_, kp_max_, ki_min_, ki_max_, kd_min_, kd_max_;
    double last_err_, err_sum_;
    double setpoint_;
};
