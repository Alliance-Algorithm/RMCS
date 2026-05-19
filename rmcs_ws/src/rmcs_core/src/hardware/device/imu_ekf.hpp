#pragma once

#include <eigen3/Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>

namespace imu_ekf {

class QuaternionEkf {
public:
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

        Mat3 accel_affine = [] {
            Mat3 matrix;
            matrix << 0.9813826498493404, 0.17232440504057203, 0.027984325323801115,
                -0.1690535919907899, 0.9747302115792275, -0.10336863799715153,
                -0.046825636945091266, 0.09953521655990044, 0.986897809387138;
            return matrix;
        }();

        Vec3 accel_bias = Vec3(0.0038458286072392397, 0.00647039594993548, 0.014968990490337293);
        Vec3 gyro_zero_offset =
            Vec3(-0.005450708363333335, -0.0008284202383333334, -0.0006914497383333334);

        Mat4 initial_covariance = Mat4::Identity();
        double gravity_acceleration = 9.7947;
        double accel_chi_square_threshold = 0.5;
        double dt_timeout_threshold = 0.1;
        double epsilon = 1e-12;
        bool apply_sensor_calibration = true;
    };

    struct TimedSample {
        Vec3 value = Vec3::Zero();
        double ready_timestamp = 0.0;
        bool valid = true;
    };

    QuaternionEkf()
        : QuaternionEkf(Config{}) {}

    explicit QuaternionEkf(const Config& config)
        : config_(config) {
        reset();
    }

    void reset() {
        initialized_ = false;
        state_ = identityQuaternion();
        state_prior_ = state_;
        covariance_ = config_.initial_covariance;
        covariance_prior_ = covariance_;
        orthogonalization_tmp_ = Mat4::Identity();
        latest_accel_original_.setZero();
        latest_gyro_original_.setZero();
        latest_valid_gyro_.setZero();
        latest_normalized_accel_ = Vec3::UnitZ();
        published_quaternion_ = identityQuaternion();
        published_euler_.setZero();
        published_rotation_ = Mat3::Identity();
        published_axis_angle_ = axisAngleFromQuaternion(published_quaternion_, config_.epsilon);
        published_accel_body_.setZero();
        published_gyro_body_.setZero();
        published_accel_odom_.setZero();
        published_gyro_odom_.setZero();
        last_chi_square_loss_ = std::numeric_limits<double>::quiet_NaN();
        last_accel_update_accepted_ = false;
        ekf_pre_valid_timestamp_ = 0.0;
    }

    [[nodiscard]] const Config& config() const noexcept { return config_; }

    void setConfig(const Config& config) {
        config_ = config;
        reset();
    }

    [[nodiscard]] bool initialized() const noexcept { return initialized_; }

    bool initializeFromAccel(const Vec3& accel_mps2, const double initial_yaw = 0.0) {
        if (!accel_mps2.allFinite()) {
            return false;
        }

        latest_accel_original_ = accel_mps2;
        const std::optional<Vec3> normalized_accel = normalizeVector(accel_mps2);
        if (!normalized_accel.has_value()) {
            return false;
        }

        latest_normalized_accel_ = *normalized_accel;

        Vec3 init_euler;
        init_euler << initial_yaw, std::asin(-latest_normalized_accel_(0)),
            std::atan2(latest_normalized_accel_(1), latest_normalized_accel_(2));

        state_ = normalizeQuaternion(quaternionFromEuler(init_euler));
        state_prior_ = state_;
        covariance_ = config_.initial_covariance;
        covariance_prior_ = covariance_;
        last_chi_square_loss_ = 0.0;
        last_accel_update_accepted_ = false;
        initialized_ = true;
        publishFromQuaternion(state_);
        updateKinematicOutputs();
        return true;
    }

    bool predict(const Vec3& gyro_rad_per_sec, const double dt) {
        if (!doPredict(gyro_rad_per_sec, dt)) {
            return false;
        }

        ekf_pre_valid_timestamp_ += dt;
        publishAt(ekf_pre_valid_timestamp_);
        return true;
    }

    bool updateFromAccel(const Vec3& accel_mps2) {
        if (!initialized_ || !accel_mps2.allFinite()) {
            return false;
        }

        latest_accel_original_ = accel_mps2;
        const std::optional<Vec3> normalized_accel = normalizeVector(accel_mps2);
        if (!normalized_accel.has_value()) {
            return false;
        }

        latest_normalized_accel_ = *normalized_accel;
        const bool accepted = doUpdateFromNormalizedAccel(latest_normalized_accel_);
        publishAt(ekf_pre_valid_timestamp_);
        return accepted;
    }

    bool
        step(const Vec3& gyro_rad_per_sec, const std::optional<Vec3>& accel_mps2, const double dt) {
        if (!initialized_) {
            if (!accel_mps2.has_value()) {
                return false;
            }
            const bool initialized = initializeFromAccel(*accel_mps2);
            publishAt(ekf_pre_valid_timestamp_);
            return initialized;
        }

        latest_gyro_original_ = gyro_rad_per_sec;
        latest_valid_gyro_ = gyro_rad_per_sec;

        if (!predict(gyro_rad_per_sec, dt)) {
            return false;
        }

        if (accel_mps2.has_value()) {
            latest_accel_original_ = *accel_mps2;
            (void)updateFromAccel(*accel_mps2);
        }

        publishAt(ekf_pre_valid_timestamp_);
        return true;
    }

    bool process(
        const std::optional<TimedSample>& gyro_sample,
        const std::optional<TimedSample>& accel_sample, const double now_timestamp,
        const double initial_yaw = 0.0) {
        if (!std::isfinite(now_timestamp)) {
            return false;
        }

        last_accel_update_accepted_ = false;

        bool gyro_update_available = false;
        bool accel_update_available = false;

        if (gyro_sample.has_value()) {
            if (!gyro_sample->value.allFinite() || !std::isfinite(gyro_sample->ready_timestamp)) {
                return false;
            }

            gyro_update_available = true;
            latest_gyro_original_ =
                gyro_sample->valid ? calibrateGyro(gyro_sample->value) : latest_valid_gyro_;
            if (gyro_sample->valid) {
                latest_valid_gyro_ = latest_gyro_original_;
            }
        }

        if (accel_sample.has_value()) {
            if (!accel_sample->value.allFinite() || !std::isfinite(accel_sample->ready_timestamp)) {
                return false;
            }

            latest_accel_original_ = accel_sample->value;
            if (accel_sample->valid) {
                latest_accel_original_ = calibrateAccel(accel_sample->value);
                const std::optional<Vec3> normalized_accel =
                    normalizeVector(latest_accel_original_);
                if (normalized_accel.has_value()) {
                    latest_normalized_accel_ = *normalized_accel;
                    accel_update_available = true;
                }
            }
        }

        if (!initialized_) {
            if (!accel_update_available) {
                return false;
            }

            if (!initializeFromAccel(latest_accel_original_, initial_yaw)) {
                return false;
            }

            ekf_pre_valid_timestamp_ = now_timestamp;
            publishAt(now_timestamp);
            return true;
        }

        if (!gyro_update_available && !accel_update_available) {
            publishAt(now_timestamp);
            return true;
        }

        if (gyro_update_available && !accel_update_available) {
            if (!advancePredictionTo(gyro_sample->ready_timestamp)) {
                return false;
            }
        } else if (!gyro_update_available && accel_update_available) {
            // Match the firmware path: accel-only update is ignored until a gyro timestamp is
            // available.
        } else if (gyro_update_available && accel_update_available) {
            const double gyro_ready = gyro_sample->ready_timestamp;
            const double accel_ready = accel_sample->ready_timestamp;

            if (std::abs(gyro_ready - accel_ready) <= config_.epsilon) {
                if (!advancePredictionTo(gyro_ready)) {
                    return false;
                }
                (void)doUpdateFromNormalizedAccel(latest_normalized_accel_);
            } else if (gyro_ready < accel_ready) {
                if (!advancePredictionTo(gyro_ready)) {
                    return false;
                }
            } else {
                if (!advancePredictionTo(accel_ready)) {
                    return false;
                }
                (void)doUpdateFromNormalizedAccel(latest_normalized_accel_);

                if (!advancePredictionTo(gyro_ready)) {
                    return false;
                }
            }
        }

        publishAt(now_timestamp);
        return true;
    }

    [[nodiscard]] double lastChiSquareLoss() const noexcept { return last_chi_square_loss_; }

    [[nodiscard]] bool accelUpdateAccepted() const noexcept { return last_accel_update_accepted_; }

    [[nodiscard]] Vec4 quaternion() const noexcept { return published_quaternion_; }

    [[nodiscard]] Vec3 eulerYawPitchRoll() const noexcept { return published_euler_; }

    [[nodiscard]] Mat3 rotationMatrix() const noexcept { return published_rotation_; }

    [[nodiscard]] Vec4 axisAngle() const noexcept { return published_axis_angle_; }

    [[nodiscard]] Vec3 accelBody() const noexcept { return published_accel_body_; }

    [[nodiscard]] Vec3 gyroBody() const noexcept { return published_gyro_body_; }

    [[nodiscard]] Vec3 accelOdom() const noexcept { return published_accel_odom_; }

    [[nodiscard]] Vec3 gyroOdom() const noexcept { return published_gyro_odom_; }

    [[nodiscard]] Vec3 originalAccel() const noexcept { return latest_accel_original_; }

    [[nodiscard]] Vec3 originalGyro() const noexcept { return latest_gyro_original_; }

    [[nodiscard]] const Mat4& covariance() const noexcept { return covariance_; }

    [[nodiscard]] const Mat4& priorCovariance() const noexcept { return covariance_prior_; }

private:
    struct PropagationResult {
        Vec4 quaternion = identityQuaternion();
        Mat4 orthogonalization = Mat4::Identity();
    };

    [[nodiscard]] static Vec4 identityQuaternion() {
        Vec4 q;
        q << 1.0, 0.0, 0.0, 0.0;
        return q;
    }

    [[nodiscard]] std::optional<Vec3> normalizeVector(const Vec3& vector) const {
        const double squared_norm = vector.squaredNorm();
        if (!std::isfinite(squared_norm) || squared_norm <= config_.epsilon) {
            return std::nullopt;
        }

        return vector / std::sqrt(squared_norm);
    }

    [[nodiscard]] Vec4 normalizeQuaternion(const Vec4& quaternion) const {
        const double squared_norm = quaternion.squaredNorm();
        if (!std::isfinite(squared_norm) || squared_norm <= config_.epsilon) {
            return identityQuaternion();
        }

        return quaternion / std::sqrt(squared_norm);
    }

    [[nodiscard]] Vec3 calibrateAccel(const Vec3& raw_accel) const {
        if (!config_.apply_sensor_calibration) {
            return raw_accel;
        }

        return config_.accel_affine * raw_accel + config_.accel_bias * config_.gravity_acceleration;
    }

    [[nodiscard]] Vec3 calibrateGyro(const Vec3& raw_gyro) const {
        if (!config_.apply_sensor_calibration) {
            return raw_gyro;
        }

        return raw_gyro + config_.gyro_zero_offset;
    }

    [[nodiscard]] static Vec4 quaternionFromEuler(const Vec3& euler_yaw_pitch_roll) {
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

    [[nodiscard]] static Mat4 omegaMatrix(const Vec3& gyro_rad_per_sec) {
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

    [[nodiscard]] PropagationResult propagateQuaternion(
        const Vec4& state, const Vec3& gyro_rad_per_sec, const double dt) const {
        const Mat4 omega = omegaMatrix(gyro_rad_per_sec);
        const Vec4 quaternion_tmp = state + 0.5 * dt * omega * state;
        const double norm = quaternion_tmp.norm();
        const double inverse_norm =
            (std::isfinite(norm) && norm > config_.epsilon) ? 1.0 / norm : 1.0;

        PropagationResult result;
        result.orthogonalization =
            inverse_norm
            * (Mat4::Identity()
               - inverse_norm * inverse_norm * quaternion_tmp * quaternion_tmp.transpose());
        result.quaternion = normalizeQuaternion(quaternion_tmp);
        return result;
    }

    [[nodiscard]] Mat4 stateJacobian(const Vec3& gyro_rad_per_sec, const double dt) const {
        return orthogonalization_tmp_
             * (Mat4::Identity() + 0.5 * dt * omegaMatrix(gyro_rad_per_sec));
    }

    [[nodiscard]] Mat43 processNoiseJacobian(const Vec4& state, const double dt) const {
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

    [[nodiscard]] static Vec3 measurementModel(const Vec4& state) {
        Vec3 result;
        result(0) = 2.0 * (state(1) * state(3) - state(0) * state(2));
        result(1) = 2.0 * (state(2) * state(3) + state(0) * state(1));
        result(2) =
            state(0) * state(0) - state(1) * state(1) - state(2) * state(2) + state(3) * state(3);
        return result;
    }

    [[nodiscard]] static Mat34 measurementJacobian(const Vec4& state) {
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

    [[nodiscard]] static Vec3 eulerFromQuaternion(const Vec4& quaternion) {
        Vec3 result = Vec3::Zero();
        result(0) = std::atan2(
            2.0 * (quaternion(0) * quaternion(3) + quaternion(1) * quaternion(2)),
            1.0 - 2.0 * (quaternion(2) * quaternion(2) + quaternion(3) * quaternion(3)));

        const double sin_pitch =
            2.0 * (quaternion(0) * quaternion(2) - quaternion(3) * quaternion(1));
        if (std::fabs(sin_pitch) >= 1.0) {
            result(1) = sin_pitch > 0.0 ? std::numbers::pi_v<double> / 2.0
                                        : -std::numbers::pi_v<double> / 2.0;
        } else {
            result(1) = std::asin(sin_pitch);
        }

        result(2) = std::atan2(
            2.0 * (quaternion(0) * quaternion(1) + quaternion(2) * quaternion(3)),
            1.0 - 2.0 * (quaternion(1) * quaternion(1) + quaternion(2) * quaternion(2)));
        return result;
    }

    [[nodiscard]] static Mat3 rotationFromQuaternion(const Vec4& quaternion) {
        Mat3 result = Mat3::Identity();

        const double q0q1 = quaternion(0) * quaternion(1);
        const double q0q2 = quaternion(0) * quaternion(2);
        const double q0q3 = quaternion(0) * quaternion(3);
        const double q1q1 = quaternion(1) * quaternion(1);
        const double q1q2 = quaternion(1) * quaternion(2);
        const double q1q3 = quaternion(1) * quaternion(3);
        const double q2q2 = quaternion(2) * quaternion(2);
        const double q2q3 = quaternion(2) * quaternion(3);
        const double q3q3 = quaternion(3) * quaternion(3);

        result(0, 0) = 1.0 - 2.0 * (q2q2 + q3q3);
        result(0, 1) = 2.0 * (q1q2 - q0q3);
        result(0, 2) = 2.0 * (q1q3 + q0q2);
        result(1, 0) = 2.0 * (q1q2 + q0q3);
        result(1, 1) = 1.0 - 2.0 * (q1q1 + q3q3);
        result(1, 2) = 2.0 * (q2q3 - q0q1);
        result(2, 0) = 2.0 * (q1q3 - q0q2);
        result(2, 1) = 2.0 * (q2q3 + q0q1);
        result(2, 2) = 1.0 - 2.0 * (q1q1 + q2q2);
        return result;
    }

    [[nodiscard]] static Vec4
        axisAngleFromQuaternion(const Vec4& quaternion, const double epsilon) {
        Vec4 result = Vec4::Zero();
        result(3) = 2.0 * std::acos(quaternion(0));

        const double sin_half_angle = std::sqrt(std::max(0.0, 1.0 - quaternion(0) * quaternion(0)));
        if (sin_half_angle <= epsilon) {
            result(0) = 1.0;
            return result;
        }

        double inverse_sin_half_angle = 1.0 / sin_half_angle;
        if (quaternion(1) < 0.0) {
            inverse_sin_half_angle = -inverse_sin_half_angle;
        }

        result(0) = quaternion(1) * inverse_sin_half_angle;
        result(1) = quaternion(2) * inverse_sin_half_angle;
        result(2) = quaternion(3) * inverse_sin_half_angle;
        return result;
    }

    bool doPredict(const Vec3& gyro_rad_per_sec, const double dt) {
        if (!initialized_ || !gyro_rad_per_sec.allFinite() || !std::isfinite(dt) || dt <= 0.0) {
            return false;
        }

        if (dt > config_.dt_timeout_threshold) {
            reset();
            return false;
        }

        const Vec4 current_state = normalizeQuaternion(state_);
        const PropagationResult propagation =
            propagateQuaternion(current_state, gyro_rad_per_sec, dt);
        state_prior_ = propagation.quaternion;
        orthogonalization_tmp_ = propagation.orthogonalization;

        const Mat4 jacobian_f_x = stateJacobian(gyro_rad_per_sec, dt);
        const Mat43 jacobian_f_w = processNoiseJacobian(state_prior_, dt);
        covariance_prior_ = jacobian_f_x * covariance_ * jacobian_f_x.transpose()
                          + jacobian_f_w * config_.process_noise * jacobian_f_w.transpose();

        state_ = state_prior_;
        covariance_ = covariance_prior_;
        return true;
    }

    bool doUpdateFromNormalizedAccel(const Vec3& normalized_accel) {
        if (!initialized_ || !normalized_accel.allFinite()) {
            return false;
        }

        const Vec3 innovation = normalized_accel - measurementModel(state_prior_);
        const Mat34 jacobian_h_x = measurementJacobian(state_prior_);
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
        state_ = normalizeQuaternion(state_prior_ + kalman_gain * innovation);

        const Mat4 matrix_tmp = Mat4::Identity() - kalman_gain * jacobian_h_x;
        covariance_ = matrix_tmp * covariance_prior_ * matrix_tmp.transpose()
                    + kalman_gain * config_.measurement_noise * kalman_gain.transpose();

        last_accel_update_accepted_ = true;
        return true;
    }

    bool advancePredictionTo(const double target_timestamp) {
        if (!std::isfinite(target_timestamp)) {
            return false;
        }

        const double dt = target_timestamp - ekf_pre_valid_timestamp_;
        if (dt < -config_.epsilon) {
            return false;
        }

        if (dt <= config_.epsilon) {
            state_prior_ = state_;
            covariance_prior_ = covariance_;
            ekf_pre_valid_timestamp_ = target_timestamp;
            return true;
        }

        if (!doPredict(latest_gyro_original_, dt)) {
            return false;
        }

        ekf_pre_valid_timestamp_ = target_timestamp;
        return true;
    }

    void publishAt(const double now_timestamp) {
        double dt = now_timestamp - ekf_pre_valid_timestamp_;
        if (!std::isfinite(dt) || dt < 0.0) {
            dt = 0.0;
        }

        const Vec4 extrapolated = propagateQuaternion(state_, latest_gyro_original_, dt).quaternion;
        publishFromQuaternion(extrapolated);
        updateKinematicOutputs();
    }

    void publishFromQuaternion(const Vec4& quaternion) {
        published_quaternion_ = normalizeQuaternion(quaternion);
        published_euler_ = eulerFromQuaternion(published_quaternion_);
        published_rotation_ = rotationFromQuaternion(published_quaternion_);
        published_axis_angle_ = axisAngleFromQuaternion(published_quaternion_, config_.epsilon);
    }

    void updateKinematicOutputs() {
        const Vec3 gravity_body =
            published_rotation_.transpose() * Vec3(0.0, 0.0, -config_.gravity_acceleration);
        published_accel_body_ = latest_accel_original_ + gravity_body;
        published_accel_odom_ = published_rotation_ * published_accel_body_;
        published_gyro_body_ = latest_gyro_original_;
        published_gyro_odom_ = published_rotation_ * published_gyro_body_;
    }

    Config config_;
    Vec4 state_ = identityQuaternion();
    Vec4 state_prior_ = identityQuaternion();
    Mat4 covariance_ = Mat4::Identity();
    Mat4 covariance_prior_ = Mat4::Identity();
    Mat4 orthogonalization_tmp_ = Mat4::Identity();

    Vec3 latest_accel_original_ = Vec3::Zero();
    Vec3 latest_gyro_original_ = Vec3::Zero();
    Vec3 latest_valid_gyro_ = Vec3::Zero();
    Vec3 latest_normalized_accel_ = Vec3::UnitZ();

    Vec4 published_quaternion_ = identityQuaternion();
    Vec3 published_euler_ = Vec3::Zero();
    Mat3 published_rotation_ = Mat3::Identity();
    Vec4 published_axis_angle_ = axisAngleFromQuaternion(identityQuaternion(), 1e-12);
    Vec3 published_accel_body_ = Vec3::Zero();
    Vec3 published_gyro_body_ = Vec3::Zero();
    Vec3 published_accel_odom_ = Vec3::Zero();
    Vec3 published_gyro_odom_ = Vec3::Zero();

    double last_chi_square_loss_ = std::numeric_limits<double>::quiet_NaN();
    double ekf_pre_valid_timestamp_ = 0.0;
    bool initialized_ = false;
    bool last_accel_update_accepted_ = false;
};

} // namespace imu_ekf
