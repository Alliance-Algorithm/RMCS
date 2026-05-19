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
#include "imu_ekf.hpp"

namespace rmcs_core::hardware::device {

class Bmi088Ekf {
public:
    using time_point = BoardClock::time_point;
    using Filter = imu_ekf::QuaternionEkf;
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
        , filter_(config_.ekf)
        , accel_queue_(config_.accel_queue_size) {
        refresh_snapshot(std::nullopt);
    }

    void store_accelerometer_status(
        std::int16_t x, std::int16_t y, std::int16_t z, time_point sample_time) {
        auto guard = std::scoped_lock{mutex_};

        if (!check_monotonic_timestamp(last_accelerometer_time_, sample_time, "accelerometer"))
            return;

        if (filter_time_.has_value() && sample_time < *filter_time_) {
            RCLCPP_WARN(
                logger(),
                "Dropping late accelerometer sample at %.9f s (filter=%.9f s)",
                to_seconds(sample_time).count(),
                to_seconds(*filter_time_).count());
            return;
        }

        const Vec3 accel_mps2 = convert_accelerometer(x, y, z);
        if (!filter_.initialized()) {
            if (try_initialize(accel_mps2, sample_time))
                return;

            RCLCPP_WARN(
                logger(), "Accelerometer initialization failed at %.9f s",
                to_seconds(sample_time).count());
            return;
        }

        push_accel_sample(accel_mps2, sample_time);
    }

    void store_gyroscope_status(
        std::int16_t x, std::int16_t y, std::int16_t z, time_point sample_time) {
        auto guard = std::scoped_lock{mutex_};

        if (!check_monotonic_timestamp(last_gyroscope_time_, sample_time, "gyroscope"))
            return;

        if (filter_time_.has_value() && sample_time < *filter_time_) {
            RCLCPP_WARN(
                logger(), "Dropping late gyroscope sample at %.9f s (filter=%.9f s)",
                to_seconds(sample_time).count(),
                to_seconds(*filter_time_).count());
            return;
        }

        if (!filter_.initialized())
            return;

        std::optional<Filter::TimedSample> latest_accel_sample;
        while (auto* queued_sample = accel_queue_.peek_front()) {
            if (queued_sample->timestamp > sample_time)
                break;

            latest_accel_sample = to_timed_sample(*queued_sample);
            if (!accel_queue_.pop_front([](BufferedAccelerometerSample&&) noexcept {})) {
                RCLCPP_ERROR(logger(), "Failed to pop accelerometer queue front");
                return;
            }
        }

        const auto gyro_sample = Filter::TimedSample{
            .value = convert_gyroscope(x, y, z),
            .ready_timestamp = to_seconds(sample_time).count(),
            .valid = true,
        };

        if (!filter_.process(
                gyro_sample, latest_accel_sample, to_seconds(sample_time).count())) {
            RCLCPP_WARN(
                logger(), "Gyroscope processing failed at %.9f s, clearing pending accel queue",
                to_seconds(sample_time).count());
            filter_.reset();
            filter_time_ = sample_time;
            accel_queue_.clear();
            refresh_snapshot(std::nullopt);
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
    struct TimeTracker {
        bool initialized = false;
        time_point last_time{};
    };

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

        const double scale =
            config_.accel_full_scale_g * filter_.config().gravity_acceleration / 32767.0;
        return config_.sensor_to_filter * (raw_accel * scale);
    }

    [[nodiscard]] auto convert_gyroscope(
        const std::int16_t x, const std::int16_t y, const std::int16_t z) const noexcept -> Vec3 {
        Vec3 raw_gyro;
        raw_gyro << static_cast<double>(x), static_cast<double>(y), static_cast<double>(z);

        const double scale =
            config_.gyro_full_scale_deg_per_sec / 32767.0 * std::numbers::pi_v<double> / 180.0;
        return config_.sensor_to_filter * (raw_gyro * scale);
    }

    [[nodiscard]] static auto to_timed_sample(const BufferedAccelerometerSample& sample)
        -> Filter::TimedSample {
        return Filter::TimedSample{
            .value = sample.vector(),
            .ready_timestamp = to_seconds(sample.timestamp).count(),
            .valid = true,
        };
    }

    [[nodiscard]] auto try_initialize(const Vec3& accel_mps2, time_point sample_time) -> bool {
        const auto accel_sample = Filter::TimedSample{
            .value = accel_mps2,
            .ready_timestamp = to_seconds(sample_time).count(),
            .valid = true,
        };

        if (!filter_.process(std::nullopt, accel_sample, to_seconds(sample_time).count()))
            return false;

        filter_time_ = sample_time;
        accel_queue_.clear();
        refresh_snapshot(sample_time);
        return true;
    }

    void push_accel_sample(const Vec3& accel_mps2, time_point sample_time) {
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

    Config config_;
    mutable std::mutex mutex_;
    Filter filter_;
    rmcs_utility::RingBuffer<BufferedAccelerometerSample> accel_queue_;
    std::optional<time_point> filter_time_;
    std::optional<time_point> last_accelerometer_time_;
    std::optional<time_point> last_gyroscope_time_;
    Snapshot snapshot_;
};

} // namespace rmcs_core::hardware::device
