#pragma once

#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>

#include <eigen3/Eigen/Geometry>
#include <rmcs_msgs/board_clock.hpp>

namespace rmcs_core::hardware::device {

// Transfers accelerometer specific force from a rigidly mounted sensor to the body origin.
// Position, force and angular velocity are all expressed in body axes.
class ImuLeverArm {
public:
    using TimePoint = rmcs_msgs::BoardClock::time_point;
    static constexpr double kMaximumSampleInterval = 1.0 / 1000.0;

    explicit ImuLeverArm(Eigen::Vector3d position, double filter_tau_s)
        : position_(std::move(position))
        , filter_tau_s_(filter_tau_s) {
        if (!position_.allFinite() || !std::isfinite(filter_tau_s_) || filter_tau_s_ < 0.0)
            throw std::invalid_argument("IMU position must be finite and filter tau nonnegative");
    }

    bool enabled() const { return !position_.isZero(0.0); }

    void reset() {
        previous_.reset();
        current_.reset();
        angular_acceleration_.setZero();
    }

    bool observe(const Eigen::Vector3d& omega, TimePoint time) {
        if (!omega.allFinite() || (current_ && time <= current_->time))
            return false;
        if (!current_) {
            current_ = GyroSample{omega, time};
            return true;
        }
        const double dt = std::chrono::duration<double>{time - current_->time}.count();
        if (dt > kMaximumSampleInterval) {
            reset();
            current_ = GyroSample{omega, time};
            return true;
        }
        const Eigen::Vector3d derivative = (omega - current_->omega) / dt;
        if (previous_)
            angular_acceleration_ +=
                dt / (filter_tau_s_ + dt) * (derivative - angular_acceleration_);
        else
            angular_acceleration_ = derivative;
        previous_ = current_;
        current_ = GyroSample{omega, time};
        return true;
    }

    std::optional<Eigen::Vector3d>
        correct(const Eigen::Vector3d& specific_force, TimePoint time) const {
        if (!enabled())
            return specific_force;
        // Never extrapolate a stale angular derivative across an unobserved interval.
        if (!previous_ || !current_ || time < previous_->time || time > current_->time)
            return std::nullopt;
        const double fraction = static_cast<double>((time - previous_->time).count())
                              / static_cast<double>((current_->time - previous_->time).count());
        const Eigen::Vector3d omega =
            previous_->omega + fraction * (current_->omega - previous_->omega);
        const Eigen::Vector3d corrected = specific_force - angular_acceleration_.cross(position_)
                                        - omega.cross(omega.cross(position_));
        return corrected.allFinite() ? std::optional<Eigen::Vector3d>{corrected} : std::nullopt;
    }

private:
    struct GyroSample {
        Eigen::Vector3d omega;
        TimePoint time;
    };

    const Eigen::Vector3d position_;
    const double filter_tau_s_;
    std::optional<GyroSample> previous_, current_;
    Eigen::Vector3d angular_acceleration_ = Eigen::Vector3d::Zero();
};

} // namespace rmcs_core::hardware::device
