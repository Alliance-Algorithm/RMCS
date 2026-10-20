#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <eigen3/Eigen/Dense>

namespace rmcs_core::deformable_rl {

// Sensor-only support-plane estimate. Equal-radius wheels must be in contact;
// this is not a contact sensor and cannot identify arbitrary airborne poses.
class SuspensionGuard {
public:
    struct Config {
        double enter_deg = 3.0;
        double exit_deg = 3.5;
        double plane_tolerance_m = 0.006;
        double confirmation_s = 0.1;
        double attitude_tau_s = 0.25;
        double acceleration_tau_s = 0.05;
        double acceleration_fusion_scale = 0.5;
    };

    struct Result {
        bool flat_hold = false;
        bool flat_candidate = false;
        bool encoder_flat_candidate = false;
        bool sensor_ready = false;
        Eigen::Vector3d gravity_b = Eigen::Vector3d::Constant(kNaN);
        Eigen::Vector3d linear_acceleration_b = Eigen::Vector3d::Constant(kNaN);
        double grade_deg = kNaN;
        double plane_error_m = kNaN;
    };

    explicit SuspensionGuard(Config config) : config_(config) {
        if (!std::isfinite(config.enter_deg) || !std::isfinite(config.exit_deg)
            || !(0.0 < config.enter_deg && config.enter_deg <= config.exit_deg && config.exit_deg < 90.0)
            || !positive(config.plane_tolerance_m) || !positive(config.attitude_tau_s)
            || !positive(config.acceleration_tau_s) || !positive(config.acceleration_fusion_scale)
            || !std::isfinite(config.confirmation_s) || config.confirmation_s < 0.0)
            throw std::invalid_argument("invalid deformable suspension guard parameters");
    }

    void reset() {
        initialized_ = false;
        sensor_valid_time_ = 0.0;
        release_flat_hold();
        acceleration_.setZero();
        force_filter_.setZero();
        velocity_.setZero();
        up_ = Eigen::Vector3d::UnitZ();
    }

    void release_flat_hold() {
        flat_hold_ = false;
        candidate_time_ = 0.0;
    }

    Result update(const Eigen::Quaterniond& attitude, const Eigen::Vector3d& gyro,
        const Eigen::Vector3d& specific_force, const Eigen::Vector3d& encoder_twist,
        const Eigen::Vector4d& q, double dt) {
        Result result;
        if (!positive(dt) || dt > 0.05 || !attitude.coeffs().allFinite()
            || attitude.norm() < 1e-9 || !gyro.allFinite() || !specific_force.allFinite()
            || !encoder_twist.allFinite() || !q.allFinite()
            || specific_force.norm() < 0.25 * kGravity || specific_force.norm() > 2.5 * kGravity) {
            reset();
            return result;
        }
        const Eigen::Vector3d velocity{encoder_twist.x(), encoder_twist.y(), 0.0};
        if (!initialized_) {
            up_ = attitude.normalized().conjugate() * Eigen::Vector3d::UnitZ();
            velocity_ = velocity;
            acceleration_ = gyro.cross(velocity);
            force_filter_ = specific_force;
            initialized_ = true;
        } else {
            // d(v_body)/dt + omega x v_body is inertial acceleration in body axes.
            const Eigen::Vector3d derivative = (velocity - velocity_) / dt;
            const double alpha = -std::expm1(-dt / config_.acceleration_tau_s);
            acceleration_ += alpha * (derivative + gyro.cross(velocity) - acceleration_);
            // Match filter phase on both signals before subtracting acceleration.
            force_filter_ += alpha * (specific_force - force_filter_);
            velocity_ = velocity;
            const double angle = gyro.norm() * dt;
            if (angle > 1e-12)
                up_ = Eigen::AngleAxisd(-angle, gyro.normalized()) * up_;
        }
        result.linear_acceleration_b = acceleration_;
        const Eigen::Vector3d corrected = force_filter_ - result.linear_acceleration_b;
        const double magnitude = corrected.norm();
        if (!std::isfinite(magnitude) || magnitude < 0.65 * kGravity || magnitude > 1.35 * kGravity) {
            // Free fall, impacts, or inconsistent odometry cannot authorize flat hold.
            reset();
            return result;
        }
        const double relative_acceleration = result.linear_acceleration_b.norm()
            / config_.acceleration_fusion_scale;
        const double alpha = -std::expm1(-dt / config_.attitude_tau_s)
            / (1.0 + relative_acceleration * relative_acceleration);
        up_ = ((1.0 - alpha) * up_ + alpha * corrected / magnitude).normalized();
        result.gravity_b = -up_;

        Eigen::Matrix<double, 4, 3> matrix;
        Eigen::Vector4d height;
        constexpr double sx[4] = {1, 1, -1, -1}, sy[4] = {-1, 1, 1, -1};
        constexpr double inv_sqrt2 = 0.7071067811865475;
        for (int i = 0; i < 4; ++i) {
            const double radial = 0.12921 / inv_sqrt2 + 0.029108 * std::cos(q[i])
                + 0.13694 * std::sin(q[i]) + 0.08542;
            matrix.row(i) << sx[i] * radial * inv_sqrt2, sy[i] * radial * inv_sqrt2, 1.0;
            height[i] = 0.08195 + 0.029108 * std::sin(q[i]) - 0.13694 * std::cos(q[i]);
        }
        const Eigen::Vector3d plane = (matrix.transpose() * matrix).ldlt().solve(matrix.transpose() * height);
        const Eigen::Vector3d normal = Eigen::Vector3d{-plane.x(), -plane.y(), 1.0}.normalized();
        result.plane_error_m = (matrix * plane - height).cwiseAbs().maxCoeff();
        result.grade_deg = std::acos(std::clamp(normal.dot(up_), -1.0, 1.0)) * 180.0 / std::acos(-1.0);
        const bool planar = std::isfinite(result.grade_deg)
            && result.plane_error_m <= config_.plane_tolerance_m;
        sensor_valid_time_ += dt;
        result.sensor_ready = sensor_valid_time_ + 1e-7 >= config_.confirmation_s;
        result.flat_candidate = planar && result.grade_deg <= config_.enter_deg;
        const Eigen::Vector3d initial_up = attitude.normalized().conjugate() * Eigen::Vector3d::UnitZ();
        const double initial_grade = std::acos(std::clamp(normal.dot(initial_up), -1.0, 1.0))
            * 180.0 / std::acos(-1.0);
        result.encoder_flat_candidate = !result.sensor_ready
            && result.plane_error_m <= config_.plane_tolerance_m && initial_grade <= config_.enter_deg;
        if (flat_hold_) {
            flat_hold_ = planar && result.grade_deg <= config_.exit_deg;
        } else if (planar && result.grade_deg <= config_.enter_deg) {
            candidate_time_ += dt;
            flat_hold_ = candidate_time_ + 1e-12 >= config_.confirmation_s;
        } else {
            candidate_time_ = 0.0;
        }
        if (flat_hold_)
            candidate_time_ = 0.0;
        result.flat_hold = flat_hold_;
        return result;
    }

private:
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kGravity = 9.80665;
    static bool positive(double value) { return std::isfinite(value) && value > 0.0; }
    Config config_;
    bool initialized_ = false;
    bool flat_hold_ = false;
    double candidate_time_ = 0.0;
    double sensor_valid_time_ = 0.0;
    Eigen::Vector3d up_ = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d force_filter_ = Eigen::Vector3d::Zero();
};

} // namespace rmcs_core::deformable_rl
