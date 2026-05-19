#pragma once

#include <eigen3/Eigen/Dense>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <numbers>
#include <optional>
#include <utility>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_msgs/board_clock.hpp>

#include "filter/imu_ekf.hpp"

namespace rmcs_core::hardware::device {

class Bmi088Ekf {
public:
    using time_point = rmcs_msgs::BoardClock::time_point;
    using duration = time_point::duration;
    using Filter = rmcs_core::filter::QuaternionEkf;
    using Mat3 = Filter::Mat3;
    using Vec3 = Filter::Vec3;
    using Vec4 = Filter::Vec4;
    using nanoseconds = std::chrono::nanoseconds;
    using seconds_duration = std::chrono::duration<double>;

    struct Config {
        Filter::Config ekf;
        Mat3 sensor_to_filter = Mat3::Identity();
        double accel_full_scale_g = 6.0;
        double gyro_full_scale_deg_per_sec = 2000.0;
        std::size_t accel_queue_size = 8;
    };

    struct Snapshot {
        bool initialized = false;
        time_point timestamp{};
        Vec4 quaternion = Vec4::Zero();
        Vec3 euler_yaw_pitch_roll = Vec3::Zero();
        Vec3 accel_body = Vec3::Zero();
        Vec3 gyro_body = Vec3::Zero();
        Vec3 accel_odom = Vec3::Zero();
        Vec3 gyro_odom = Vec3::Zero();
        bool accel_update_accepted = false;
        double chi_square_loss = std::numeric_limits<double>::quiet_NaN();
    };

    Bmi088Ekf()
        : Bmi088Ekf(Config{}) {}

    explicit Bmi088Ekf(Config config)
        : config_(std::move(config))
        , queue_size_(config_.accel_queue_size == 0 ? 1 : config_.accel_queue_size)
        , accel_scale_mps2_(
              config_.accel_full_scale_g * config_.ekf.gravity_acceleration / kRawSensorMaxValue)
        , gyro_scale_rad_per_sec_(
              config_.gyro_full_scale_deg_per_sec * std::numbers::pi_v<double> / 180.0
              / kRawSensorMaxValue)
        , filter_(config_.ekf) {
        refresh_snapshot(std::nullopt);
    }

    void store_accelerometer_status(
        std::int16_t x, std::int16_t y, std::int16_t z, time_point sample_time) {
        auto guard = std::scoped_lock{mutex_};

        if (!accept_sample_time(last_accelerometer_time_, sample_time, "accelerometer")) {
            return;
        }

        const Vec3 accel_mps2 = convert_accelerometer(x, y, z);
        if (!filter_.initialized()) {
            if (filter_.initializeFromAccel(accel_mps2)) {
                fused_time_ = sample_time;
                accelerometer_watermark_ = sample_time;
                last_fused_accel_ = accel_mps2;
                last_fused_gyro_.setZero();
                clear_queues();
                refresh_snapshot(sample_time);
                return;
            }

            RCLCPP_WARN(
                logger(), "Accelerometer initialization failed at %.9f s",
                to_seconds(sample_time).count());
            return;
        }

        accelerometer_watermark_ = sample_time;
        if (!enqueue_accelerometer_sample(accel_mps2, sample_time)) {
            handle_queue_overflow("accelerometer", sample_time);
            return;
        }
        drain_ready_gyroscopes();
    }

    void store_gyroscope_status(
        std::int16_t x, std::int16_t y, std::int16_t z, time_point sample_time) {
        auto guard = std::scoped_lock{mutex_};

        if (!accept_sample_time(last_gyroscope_time_, sample_time, "gyroscope")) {
            return;
        }

        if (!filter_.initialized()) {
            return;
        }

        if (!enqueue_gyroscope_sample(convert_gyroscope(x, y, z), sample_time)) {
            handle_queue_overflow("gyroscope", sample_time);
            return;
        }
        drain_ready_gyroscopes();
    }

    [[nodiscard]] bool initialized() const noexcept {
        auto guard = std::scoped_lock{mutex_};
        return filter_.initialized();
    }

    [[nodiscard]] auto snapshot() const noexcept -> Snapshot {
        auto guard = std::scoped_lock{mutex_};
        return snapshot_;
    }

private:
    struct AccelerometerSample {
        double x_mps2 = 0.0;
        double y_mps2 = 0.0;
        double z_mps2 = 0.0;
        time_point timestamp{};

        [[nodiscard]] auto vector() const noexcept -> Vec3 {
            Vec3 result;
            result << x_mps2, y_mps2, z_mps2;
            return result;
        }
    };

    struct GyroscopeSample {
        double x_rad_per_sec = 0.0;
        double y_rad_per_sec = 0.0;
        double z_rad_per_sec = 0.0;
        time_point timestamp{};

        [[nodiscard]] auto vector() const noexcept -> Vec3 {
            Vec3 result;
            result << x_rad_per_sec, y_rad_per_sec, z_rad_per_sec;
            return result;
        }
    };

    [[nodiscard]] static auto logger() -> const rclcpp::Logger& {
        static const rclcpp::Logger logger = rclcpp::get_logger("bmi088_ekf");
        return logger;
    }

    [[nodiscard]] static auto to_seconds(time_point sample_time) noexcept -> seconds_duration {
        return std::chrono::duration_cast<seconds_duration>(sample_time.time_since_epoch());
    }

    [[nodiscard]] static auto to_nanoseconds(duration delta) noexcept -> nanoseconds {
        return std::chrono::duration_cast<nanoseconds>(delta);
    }

    [[nodiscard]] auto accept_sample_time(
        std::optional<time_point>& last_time, time_point sample_time, const char* stream_name) const
        -> bool {
        if (last_time.has_value() && sample_time <= *last_time) {
            RCLCPP_WARN(
                logger(),
                "Dropping non-monotonic %s sample at %.9f s (last=%.9f s)", stream_name,
                to_seconds(sample_time).count(), to_seconds(*last_time).count());
            return false;
        }

        last_time = sample_time;
        if (!fused_time_.has_value() || sample_time >= *fused_time_) {
            return true;
        }

        RCLCPP_WARN(
            logger(), "Dropping late %s sample at %.9f s (filter=%.9f s)", stream_name,
            to_seconds(sample_time).count(), to_seconds(*fused_time_).count());
        return false;
    }

    [[nodiscard]] auto convert_accelerometer(
        std::int16_t x, std::int16_t y, std::int16_t z) const noexcept -> Vec3 {
        Vec3 raw_accel;
        raw_accel << static_cast<double>(x), static_cast<double>(y), static_cast<double>(z);
        return config_.sensor_to_filter * (raw_accel * accel_scale_mps2_);
    }

    [[nodiscard]] auto convert_gyroscope(
        std::int16_t x, std::int16_t y, std::int16_t z) const noexcept -> Vec3 {
        Vec3 raw_gyro;
        raw_gyro << static_cast<double>(x), static_cast<double>(y), static_cast<double>(z);
        return config_.sensor_to_filter * (raw_gyro * gyro_scale_rad_per_sec_);
    }

    [[nodiscard]] auto enqueue_accelerometer_sample(const Vec3& accel_mps2, time_point sample_time)
        -> bool {
        AccelerometerSample sample;
        sample.x_mps2 = accel_mps2.x();
        sample.y_mps2 = accel_mps2.y();
        sample.z_mps2 = accel_mps2.z();
        sample.timestamp = sample_time;
        return enqueue_sample(accelerometer_queue_, std::move(sample));
    }

    [[nodiscard]] auto enqueue_gyroscope_sample(const Vec3& gyro_rad_per_sec, time_point sample_time)
        -> bool {
        GyroscopeSample sample;
        sample.x_rad_per_sec = gyro_rad_per_sec.x();
        sample.y_rad_per_sec = gyro_rad_per_sec.y();
        sample.z_rad_per_sec = gyro_rad_per_sec.z();
        sample.timestamp = sample_time;
        return enqueue_sample(gyroscope_queue_, std::move(sample));
    }

    template <typename Sample>
    [[nodiscard]] auto enqueue_sample(std::deque<Sample>& queue, Sample sample) -> bool {
        if (queue.size() < queue_size_) {
            queue.push_back(std::move(sample));
            return true;
        }
        return false;
    }

    void handle_queue_overflow(const char* stream_name, time_point sample_time) {
        RCLCPP_WARN(
            logger(), "%s queue overflow at %.9f s, resetting filter state", stream_name,
            to_seconds(sample_time).count());
        reset_filter_state();
    }

    void drain_ready_gyroscopes() {
        while (!gyroscope_queue_.empty()) {
            if (!accelerometer_watermark_.has_value()) {
                return;
            }

            const auto& gyro = gyroscope_queue_.front();
            if (gyro.timestamp > *accelerometer_watermark_) {
                return;
            }

            const auto gyro_time = gyro.timestamp;
            const auto gyro_rad_per_sec = gyro.vector();
            if (!process_gyroscope_sample(gyro_rad_per_sec, gyro_time)) {
                RCLCPP_WARN(
                    logger(), "Gyroscope processing failed at %.9f s, clearing pending queues",
                    to_seconds(gyro_time).count());
                reset_after_failed_process(gyro_time);
                return;
            }

            gyroscope_queue_.pop_front();
            fused_time_ = gyro_time;
            refresh_snapshot(gyro_time);
        }
    }

    [[nodiscard]] auto process_gyroscope_sample(const Vec3& gyro_rad_per_sec, time_point gyro_time)
        -> bool {
        if (!fused_time_.has_value()) {
            return false;
        }

        auto cursor = *fused_time_;
        last_fused_gyro_ = gyro_rad_per_sec;
        while (!accelerometer_queue_.empty() && accelerometer_queue_.front().timestamp <= gyro_time) {
            const auto accel = accelerometer_queue_.front();
            if (!filter_.predict(gyro_rad_per_sec, to_nanoseconds(accel.timestamp - cursor))) {
                return false;
            }
            last_fused_accel_ = accel.vector();
            if (!filter_.updateFromAccel(last_fused_accel_)) {
                return false;
            }

            cursor = accel.timestamp;
            accelerometer_queue_.pop_front();
        }

        return filter_.predict(gyro_rad_per_sec, to_nanoseconds(gyro_time - cursor));
    }

    void reset_after_failed_process(time_point) { reset_filter_state(); }

    void reset_filter_state() {
        filter_.reset();
        fused_time_.reset();
        accelerometer_watermark_.reset();
        last_fused_accel_.setZero();
        last_fused_gyro_.setZero();
        clear_queues();
        refresh_snapshot(std::nullopt);
    }

    void clear_queues() {
        accelerometer_queue_.clear();
        gyroscope_queue_.clear();
    }

    void refresh_snapshot(std::optional<time_point> sample_time) {
        snapshot_.initialized = filter_.initialized();
        snapshot_.timestamp =
            snapshot_.initialized && sample_time.has_value() ? *sample_time : time_point{};
        snapshot_.accel_update_accepted = filter_.accelUpdateAccepted();
        snapshot_.chi_square_loss = filter_.lastChiSquareLoss();

        if (!snapshot_.initialized) {
            snapshot_.quaternion = Vec4::Zero();
            snapshot_.euler_yaw_pitch_roll = Vec3::Zero();
            snapshot_.accel_body = Vec3::Zero();
            snapshot_.gyro_body = Vec3::Zero();
            snapshot_.accel_odom = Vec3::Zero();
            snapshot_.gyro_odom = Vec3::Zero();
            return;
        }

        snapshot_.quaternion = filter_.quaternion();
        snapshot_.euler_yaw_pitch_roll = eulerFromQuaternion(snapshot_.quaternion);
        const Mat3 rotation = rotationFromQuaternion(snapshot_.quaternion);
        const Vec3 gravity_body =
            rotation.transpose() * Vec3(0.0, 0.0, -config_.ekf.gravity_acceleration);
        snapshot_.accel_body = last_fused_accel_ + gravity_body;
        snapshot_.gyro_body = last_fused_gyro_;
        snapshot_.accel_odom = rotation * snapshot_.accel_body;
        snapshot_.gyro_odom = rotation * snapshot_.gyro_body;
    }

    [[nodiscard]] static auto eulerFromQuaternion(const Vec4& quaternion) -> Vec3 {
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

    [[nodiscard]] static auto rotationFromQuaternion(const Vec4& quaternion) -> Mat3 {
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

    static constexpr double kRawSensorMaxValue = 32767.0;

    Config config_;
    std::size_t queue_size_ = 1;
    double accel_scale_mps2_ = 0.0;
    double gyro_scale_rad_per_sec_ = 0.0;
    mutable std::mutex mutex_;
    Filter filter_;
    std::deque<AccelerometerSample> accelerometer_queue_;
    std::deque<GyroscopeSample> gyroscope_queue_;
    std::optional<time_point> fused_time_;
    std::optional<time_point> accelerometer_watermark_;
    std::optional<time_point> last_accelerometer_time_;
    std::optional<time_point> last_gyroscope_time_;
    Vec3 last_fused_accel_ = Vec3::Zero();
    Vec3 last_fused_gyro_ = Vec3::Zero();
    Snapshot snapshot_;
};

} // namespace rmcs_core::hardware::device
