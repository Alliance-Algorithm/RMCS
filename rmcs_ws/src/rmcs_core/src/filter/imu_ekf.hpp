#pragma once

#include <eigen3/Eigen/Dense>

#include <chrono>
#include <cmath>
#include <limits>
#include <optional>

namespace rmcs_core::filter {

class QuaternionEkf {
public:
    using Duration = std::chrono::nanoseconds;
    using Vec3 = Eigen::Vector3d;
    using Vec4 = Eigen::Vector4d;
    using Mat3 = Eigen::Matrix3d;
    using Mat4 = Eigen::Matrix4d;
    using Mat43 = Eigen::Matrix<double, 4, 3>;
    using Mat34 = Eigen::Matrix<double, 3, 4>;

    struct Config {
        Mat3 process_noise = [] {
            Mat3 matrix = Mat3::Zero();
            matrix.diagonal() << 0.865, 0.975, 1.077;
            return matrix;
        }();

        Mat3 measurement_noise = [] {
            Mat3 matrix = Mat3::Zero();
            matrix.diagonal() << 0.446, 0.476, 0.537;
            return matrix;
        }();

        Mat4 initial_covariance = Mat4::Identity();
        double gravity_acceleration = 9.7947;
        double accel_chi_square_threshold = 0.5;
        double dt_timeout_threshold = 0.1;
        double epsilon = 1e-12;
    };

    QuaternionEkf()
        : QuaternionEkf(Config{}) {}

    explicit QuaternionEkf(const Config& config)
        : config_(config) {
        reset();
    }

    void reset() {
        initialized_ = false;
        state_ = identity_quaternion();
        state_prior_ = state_;
        covariance_ = config_.initial_covariance;
        covariance_prior_ = covariance_;
        orthogonalization_tmp_ = Mat4::Identity();
        last_chi_square_loss_ = std::numeric_limits<double>::quiet_NaN();
        last_accel_update_accepted_ = false;
    }

    [[nodiscard]] const Config& config() const noexcept { return config_; }

    void set_config(const Config& config) {
        config_ = config;
        reset();
    }

    [[nodiscard]] bool initialized() const noexcept { return initialized_; }

    bool initialize_from_accel(const Vec3& accel_mps2, const double initial_yaw = 0.0) {
        if (!accel_mps2.allFinite()) {
            return false;
        }

        const std::optional<Vec3> normalized_accel = normalize_vector(accel_mps2);
        if (!normalized_accel.has_value()) {
            return false;
        }

        Vec3 init_euler;
        init_euler << initial_yaw, std::asin(-(*normalized_accel)(0)),
            std::atan2((*normalized_accel)(1), (*normalized_accel)(2));

        state_ = normalize_quaternion(quaternion_from_euler(init_euler));
        state_prior_ = state_;
        covariance_ = config_.initial_covariance;
        covariance_prior_ = covariance_;
        last_chi_square_loss_ = 0.0;
        last_accel_update_accepted_ = false;
        initialized_ = true;
        return true;
    }

    bool predict(const Vec3& gyro_rad_per_sec, const Duration dt) {
        if (!initialized_ || !gyro_rad_per_sec.allFinite()) {
            return false;
        }

        const double dt_seconds = duration_to_seconds(dt);
        if (!std::isfinite(dt_seconds) || dt_seconds < 0.0) {
            return false;
        }

        last_accel_update_accepted_ = false;
        if (dt_seconds <= config_.epsilon) {
            state_prior_ = state_;
            covariance_prior_ = covariance_;
            return true;
        }

        return do_predict(gyro_rad_per_sec, dt_seconds);
    }

    bool update_from_accel(const Vec3& accel_mps2) {
        if (!initialized_ || !accel_mps2.allFinite()) {
            return false;
        }

        const std::optional<Vec3> normalized_accel = normalize_vector(accel_mps2);
        if (!normalized_accel.has_value()) {
            return false;
        }

        last_accel_update_accepted_ = false;
        state_prior_ = state_;
        covariance_prior_ = covariance_;
        (void)do_update_from_normalized_accel(*normalized_accel);
        return true;
    }

    [[nodiscard]] auto quaternion() const noexcept -> const Vec4& { return state_; }

private:
    struct PropagationResult {
        Vec4 quaternion = identity_quaternion();
        Mat4 orthogonalization = Mat4::Identity();
    };

    [[nodiscard]] static Vec4 identity_quaternion() {
        Vec4 q;
        q << 1.0, 0.0, 0.0, 0.0;
        return q;
    }

    [[nodiscard]] std::optional<Vec3> normalize_vector(const Vec3& vector) const {
        const double squared_norm = vector.squaredNorm();
        if (!std::isfinite(squared_norm) || squared_norm <= config_.epsilon) {
            return std::nullopt;
        }

        return vector / std::sqrt(squared_norm);
    }

    [[nodiscard]] Vec4 normalize_quaternion(const Vec4& quaternion) const {
        const double squared_norm = quaternion.squaredNorm();
        if (!std::isfinite(squared_norm) || squared_norm <= config_.epsilon) {
            return identity_quaternion();
        }

        return quaternion / std::sqrt(squared_norm);
    }

    [[nodiscard]] static auto duration_to_seconds(const Duration dt) noexcept -> double {
        return std::chrono::duration<double>(dt).count();
    }

    [[nodiscard]] static Vec4 quaternion_from_euler(const Vec3& euler_yaw_pitch_roll) {
        const double half_yaw = euler_yaw_pitch_roll(0) * 0.5;
        const double half_pitch = euler_yaw_pitch_roll(1) * 0.5;
        const double half_roll = euler_yaw_pitch_roll(2) * 0.5;

        const double cy = std::cos(half_yaw);
        const double sy = std::sin(half_yaw);
        const double cp = std::cos(half_pitch);
        const double sp = std::sin(half_pitch);
        const double cr = std::cos(half_roll);
        const double sr = std::sin(half_roll);

        Vec4 result;
        result(0) = cy * cp * cr + sy * sp * sr;
        result(1) = cy * cp * sr - sy * sp * cr;
        result(2) = cy * sp * cr + sy * cp * sr;
        result(3) = sy * cp * cr - cy * sp * sr;
        return result;
    }

    [[nodiscard]] static Mat4 omega_matrix(const Vec3& gyro_rad_per_sec) {
        Mat4 matrix = Mat4::Zero();
        matrix(0, 1) = -gyro_rad_per_sec(0);
        matrix(0, 2) = -gyro_rad_per_sec(1);
        matrix(0, 3) = -gyro_rad_per_sec(2);
        matrix(1, 0) = gyro_rad_per_sec(0);
        matrix(1, 2) = gyro_rad_per_sec(2);
        matrix(1, 3) = -gyro_rad_per_sec(1);
        matrix(2, 0) = gyro_rad_per_sec(1);
        matrix(2, 1) = -gyro_rad_per_sec(2);
        matrix(2, 3) = gyro_rad_per_sec(0);
        matrix(3, 0) = gyro_rad_per_sec(2);
        matrix(3, 1) = gyro_rad_per_sec(1);
        matrix(3, 2) = -gyro_rad_per_sec(0);
        return matrix;
    }

    [[nodiscard]] auto propagate_quaternion(
        const Vec4& state, const Vec3& gyro_rad_per_sec, const double dt) const
        -> PropagationResult {
        const Mat4 omega = omega_matrix(gyro_rad_per_sec);
        const Vec4 quaternion_tmp = state + 0.5 * dt * omega * state;
        const double norm = quaternion_tmp.norm();
        const double inverse_norm =
            (std::isfinite(norm) && norm > config_.epsilon) ? 1.0 / norm : 1.0;

        PropagationResult result;
        result.orthogonalization =
            inverse_norm
            * (Mat4::Identity()
               - inverse_norm * inverse_norm * quaternion_tmp * quaternion_tmp.transpose());
        result.quaternion = normalize_quaternion(quaternion_tmp);
        return result;
    }

    [[nodiscard]] auto state_jacobian(const Vec3& gyro_rad_per_sec, const double dt) const -> Mat4 {
        return orthogonalization_tmp_
             * (Mat4::Identity() + 0.5 * dt * omega_matrix(gyro_rad_per_sec));
    }

    [[nodiscard]] auto process_noise_jacobian(const Vec4& state, const double dt) const -> Mat43 {
        Mat43 matrix_q;
        matrix_q(0, 0) = -state(1);
        matrix_q(0, 1) = -state(2);
        matrix_q(0, 2) = -state(3);
        matrix_q(1, 0) = state(0);
        matrix_q(1, 1) = -state(3);
        matrix_q(1, 2) = state(2);
        matrix_q(2, 0) = state(3);
        matrix_q(2, 1) = state(0);
        matrix_q(2, 2) = -state(1);
        matrix_q(3, 0) = -state(2);
        matrix_q(3, 1) = state(1);
        matrix_q(3, 2) = state(0);
        return orthogonalization_tmp_ * (0.5 * dt * matrix_q);
    }

    [[nodiscard]] static auto measurement_model(const Vec4& state) -> Vec3 {
        Vec3 result;
        result(0) = 2.0 * (state(1) * state(3) - state(0) * state(2));
        result(1) = 2.0 * (state(2) * state(3) + state(0) * state(1));
        result(2) =
            state(0) * state(0) - state(1) * state(1) - state(2) * state(2) + state(3) * state(3);
        return result;
    }

    [[nodiscard]] static auto measurement_jacobian(const Vec4& state) -> Mat34 {
        Mat34 result;
        result(0, 0) = -2.0 * state(2);
        result(0, 1) = 2.0 * state(3);
        result(0, 2) = -2.0 * state(0);
        result(0, 3) = 2.0 * state(1);

        result(1, 0) = 2.0 * state(1);
        result(1, 1) = 2.0 * state(0);
        result(1, 2) = 2.0 * state(3);
        result(1, 3) = 2.0 * state(2);

        result(2, 0) = 2.0 * state(0);
        result(2, 1) = -2.0 * state(1);
        result(2, 2) = -2.0 * state(2);
        result(2, 3) = 2.0 * state(3);
        return result;
    }

    bool do_predict(const Vec3& gyro_rad_per_sec, const double dt) {
        if (!initialized_ || !gyro_rad_per_sec.allFinite() || !std::isfinite(dt) || dt <= 0.0) {
            return false;
        }

        if (dt > config_.dt_timeout_threshold) {
            reset();
            return false;
        }

        const Vec4 current_state = normalize_quaternion(state_);
        const PropagationResult propagation =
            propagate_quaternion(current_state, gyro_rad_per_sec, dt);
        state_prior_ = propagation.quaternion;
        orthogonalization_tmp_ = propagation.orthogonalization;

        const Mat4 jacobian_f_x = state_jacobian(gyro_rad_per_sec, dt);
        const Mat43 jacobian_f_w = process_noise_jacobian(state_prior_, dt);
        covariance_prior_ = jacobian_f_x * covariance_ * jacobian_f_x.transpose()
                          + jacobian_f_w * config_.process_noise * jacobian_f_w.transpose();

        state_ = state_prior_;
        covariance_ = covariance_prior_;
        return true;
    }

    bool do_update_from_normalized_accel(const Vec3& normalized_accel) {
        if (!initialized_ || !normalized_accel.allFinite()) {
            return false;
        }

        const Vec3 innovation = normalized_accel - measurement_model(state_prior_);
        const Mat34 jacobian_h_x = measurement_jacobian(state_prior_);
        const Mat3 innovation_covariance =
            jacobian_h_x * covariance_prior_ * jacobian_h_x.transpose() + config_.measurement_noise;
        const Mat3 innovation_covariance_inverse = innovation_covariance.inverse();

        last_chi_square_loss_ =
            (innovation.transpose() * innovation_covariance_inverse * innovation)(0, 0);
        if (!std::isfinite(last_chi_square_loss_)
            || last_chi_square_loss_ > config_.accel_chi_square_threshold) {
            last_accel_update_accepted_ = false;
            return false;
        }

        const Eigen::Matrix<double, 4, 3> kalman_gain =
            covariance_prior_ * jacobian_h_x.transpose() * innovation_covariance_inverse;
        state_ = normalize_quaternion(state_prior_ + kalman_gain * innovation);

        const Mat4 matrix_tmp = Mat4::Identity() - kalman_gain * jacobian_h_x;
        covariance_ = matrix_tmp * covariance_prior_ * matrix_tmp.transpose()
                    + kalman_gain * config_.measurement_noise * kalman_gain.transpose();
        last_accel_update_accepted_ = true;
        return true;
    }

    Config config_;
    Vec4 state_ = identity_quaternion();
    Vec4 state_prior_ = identity_quaternion();
    Mat4 covariance_ = Mat4::Identity();
    Mat4 covariance_prior_ = Mat4::Identity();
    Mat4 orthogonalization_tmp_ = Mat4::Identity();
    double last_chi_square_loss_ = std::numeric_limits<double>::quiet_NaN();
    bool initialized_ = false;
    bool last_accel_update_accepted_ = false;
};

} // namespace rmcs_core::filter
