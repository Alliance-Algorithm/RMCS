#pragma once

#include <eigen3/Eigen/Dense>

#include <cstddef>
#include <limits>
#include <mutex>
#include <numbers>
#include <optional>
#include <utility>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_utility/ring_buffer.hpp>

#include "board_clock.hpp"
#include "filter/imu_ekf.hpp"

namespace rmcs_core::hardware::device {

class Bmi088Ekf {
public:
    using time_point = BoardClock::time_point;
    using Filter = rmcs_core::filter::QuaternionEkf;
    using Mat3 = Filter::Mat3;
    using Vec3 = Filter::Vec3;
    using Vec4 = Filter::Vec4;
    using seconds_duration = std::chrono::duration<double>;

    struct Config {
        Config() { ekf.apply_sensor_calibration = false; }

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
        , accel_scale_mps2_(
              config_.accel_full_scale_g * config_.ekf.gravity_acceleration / kRawSensorMaxValue)
        , gyro_scale_rad_per_sec_(
              config_.gyro_full_scale_deg_per_sec * std::numbers::pi_v<double> / 180.0
              / kRawSensorMaxValue)
        , filter_(config_.ekf)
        , accel_queue_(config_.accel_queue_size) {
        refresh_snapshot(std::nullopt);
    }

    void store_accelerometer_status(
        std::int16_t x, std::int16_t y, std::int16_t z, time_point sample_time) {
        auto guard = std::scoped_lock{mutex_};

        if (!accept_sample_time(last_accelerometer_time_, sample_time, "accelerometer"))
            return;

        const Vec3 accel_mps2 = convert_accelerometer(x, y, z);
        if (!filter_.initialized()) {
            if (try_initialize(accel_mps2, sample_time))
                return;

            RCLCPP_WARN(
                logger(), "Accelerometer initialization failed at %.9f s",
                to_seconds(sample_time).count());
            return;
        }

        enqueue_accel_sample(accel_mps2, sample_time);
    }

    void store_gyroscope_status(
        std::int16_t x, std::int16_t y, std::int16_t z, time_point sample_time) {
        auto guard = std::scoped_lock{mutex_};

        if (!accept_sample_time(last_gyroscope_time_, sample_time, "gyroscope"))
            return;

        if (!filter_.initialized())
            return;

        const auto drained_accel_samples = drain_accel_samples_until(sample_time);
        if (!drained_accel_samples.has_value())
            return;

        const auto gyro_sample = make_timed_sample(convert_gyroscope(x, y, z), sample_time);
        if (!filter_.process(
                gyro_sample, *drained_accel_samples,
                to_seconds(sample_time).count())) {
            RCLCPP_WARN(
                logger(), "Gyroscope processing failed at %.9f s, clearing pending accel queue",
                to_seconds(sample_time).count());
            reset_after_failed_process(sample_time);
            return;
        }

        filter_time_ = sample_time;
        refresh_snapshot(sample_time);
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
    struct BufferedAccelerometerSample {
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

    [[nodiscard]] static auto logger() -> const rclcpp::Logger& {
        static const rclcpp::Logger logger = rclcpp::get_logger("bmi088_ekf");
        return logger;
    }

    [[nodiscard]] static auto to_seconds(time_point sample_time) noexcept -> seconds_duration {
        return std::chrono::duration_cast<seconds_duration>(sample_time.time_since_epoch());
    }

    [[nodiscard]] auto accept_sample_time(
        std::optional<time_point>& last_time, time_point sample_time, const char* stream_name) const
        -> bool {
        if (!check_monotonic_timestamp(last_time, sample_time, stream_name))
            return false;

        if (!filter_time_.has_value() || sample_time >= *filter_time_)
            return true;

        RCLCPP_WARN(
            logger(), "Dropping late %s sample at %.9f s (filter=%.9f s)", stream_name,
            to_seconds(sample_time).count(), to_seconds(*filter_time_).count());
        return false;
    }

    [[nodiscard]] static auto check_monotonic_timestamp(
        std::optional<time_point>& last_time, time_point sample_time, const char* stream_name)
        -> bool {
        if (last_time.has_value() && sample_time <= *last_time) {
            RCLCPP_WARN(
                logger(),
                "Dropping non-monotonic %s sample at %.9f s (last=%.9f s)", stream_name,
                to_seconds(sample_time).count(), to_seconds(*last_time).count());
            return false;
        }

        last_time = sample_time;
        return true;
    }

    [[nodiscard]] auto convert_accelerometer(
        const std::int16_t x, const std::int16_t y, const std::int16_t z) const noexcept -> Vec3 {
        Vec3 raw_accel;
        raw_accel << static_cast<double>(x), static_cast<double>(y), static_cast<double>(z);

        return config_.sensor_to_filter * (raw_accel * accel_scale_mps2_);
    }

    [[nodiscard]] auto convert_gyroscope(
        const std::int16_t x, const std::int16_t y, const std::int16_t z) const noexcept -> Vec3 {
        Vec3 raw_gyro;
        raw_gyro << static_cast<double>(x), static_cast<double>(y), static_cast<double>(z);

        return config_.sensor_to_filter * (raw_gyro * gyro_scale_rad_per_sec_);
    }

    [[nodiscard]] static auto make_timed_sample(const Vec3& value, time_point sample_time)
        -> Filter::TimedSample {
        return Filter::TimedSample{
            .value = value,
            .ready_timestamp = to_seconds(sample_time).count(),
            .valid = true,
        };
    }

    [[nodiscard]] auto try_initialize(const Vec3& accel_mps2, time_point sample_time) -> bool {
        const auto accel_sample = make_timed_sample(accel_mps2, sample_time);
        if (!filter_.process(std::nullopt, accel_sample, to_seconds(sample_time).count()))
            return false;

        filter_time_ = sample_time;
        accel_queue_.clear();
        refresh_snapshot(sample_time);
        return true;
    }

    [[nodiscard]] auto drain_accel_samples_until(time_point sample_time)
        -> std::optional<std::optional<Filter::TimedSample>> {
        std::optional<Filter::TimedSample> latest_sample;

        while (auto* queued_sample = accel_queue_.peek_front()) {
            if (queued_sample->timestamp > sample_time)
                break;

            latest_sample = make_timed_sample(queued_sample->vector(), queued_sample->timestamp);
            if (!accel_queue_.pop_front([](BufferedAccelerometerSample&&) noexcept {})) {
                RCLCPP_ERROR(logger(), "Failed to pop accelerometer queue front");
                return std::nullopt;
            }
        }

        return latest_sample;
    }

    void enqueue_accel_sample(const Vec3& accel_mps2, time_point sample_time) {
        BufferedAccelerometerSample sample;
        sample.x_mps2 = accel_mps2.x();
        sample.y_mps2 = accel_mps2.y();
        sample.z_mps2 = accel_mps2.z();
        sample.timestamp = sample_time;

        if (accel_queue_.emplace_back(sample))
            return;

        RCLCPP_WARN(
            logger(), "Accelerometer queue full at %.9f s, dropping oldest sample",
            to_seconds(sample_time).count());
        if (!accel_queue_.pop_front([](BufferedAccelerometerSample&&) noexcept {})) {
            RCLCPP_ERROR(logger(), "Failed to drop oldest accelerometer sample");
            return;
        }

        if (!accel_queue_.emplace_back(sample))
            RCLCPP_ERROR(logger(), "Failed to enqueue accelerometer sample after drop");
    }

    void reset_after_failed_process(time_point sample_time) {
        filter_.reset();
        filter_time_ = sample_time;
        accel_queue_.clear();
        refresh_snapshot(std::nullopt);
    }

    void refresh_snapshot(std::optional<time_point> sample_time) {
        snapshot_.initialized = filter_.initialized();
        snapshot_.timestamp =
            snapshot_.initialized && sample_time.has_value() ? *sample_time : time_point{};
        snapshot_.quaternion = filter_.quaternion();
        snapshot_.euler_yaw_pitch_roll = filter_.eulerYawPitchRoll();
        snapshot_.accel_body = filter_.accelBody();
        snapshot_.gyro_body = filter_.gyroBody();
        snapshot_.accel_odom = filter_.accelOdom();
        snapshot_.gyro_odom = filter_.gyroOdom();
        snapshot_.accel_update_accepted = filter_.accelUpdateAccepted();
        snapshot_.chi_square_loss = filter_.lastChiSquareLoss();
    }

    static constexpr double kRawSensorMaxValue = 32767.0;

    Config config_;
    double accel_scale_mps2_ = 0.0;
    double gyro_scale_rad_per_sec_ = 0.0;
    mutable std::mutex mutex_;
    Filter filter_;
    rmcs_utility::RingBuffer<BufferedAccelerometerSample> accel_queue_;
    std::optional<time_point> filter_time_;
    std::optional<time_point> last_accelerometer_time_;
    std::optional<time_point> last_gyroscope_time_;
    Snapshot snapshot_;
};

} // namespace rmcs_core::hardware::device
