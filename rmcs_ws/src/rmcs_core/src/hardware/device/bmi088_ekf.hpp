#pragma once

#include <eigen3/Eigen/Dense>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
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
        Vec4 quaternion = Vec4::Zero();
        Vec3 gyro_body = Vec3::Zero();
    };

    Bmi088Ekf()
        : Bmi088Ekf(Config{}) {}

    explicit Bmi088Ekf(Config config)
        : config_(std::move(config))
        , queue_size_(config_.accel_queue_size == 0 ? 1 : config_.accel_queue_size)
        , accel_scale_mps2_(
              config_.accel_full_scale_g * config_.ekf.gravity_acceleration / kRawSensorMaxValue)
        , gyro_scale_rad_per_sec_(
              config_.gyro_full_scale_deg_per_sec * std::numbers::pi_v<double>
              / 180.0 / kRawSensorMaxValue)
        , filter_(config_.ekf) {
        refresh_snapshot();
    }

    void store_accelerometer_status(
        std::int16_t x, std::int16_t y, std::int16_t z, time_point sample_time) {
        auto guard = std::scoped_lock{mutex_};

        if (!accept_sample_time(last_accelerometer_time_, sample_time, "accelerometer")) {
            return;
        }

        const Vec3 accel_mps2 = convert_accelerometer(x, y, z);
        if (!filter_.initialized()) {
            if (filter_.initialize_from_accel(accel_mps2)) {
                fused_time_ = sample_time;
                accelerometer_watermark_ = sample_time;
                last_fused_gyro_.setZero();
                clear_queues();
                refresh_snapshot();
                return;
            }

            RCLCPP_WARN(
                logger(), "Accelerometer initialization failed at %.9f s",
                to_seconds(sample_time).count());
            return;
        }

        accelerometer_watermark_ = sample_time;
        if (!enqueue_sample(accelerometer_queue_, accel_mps2, sample_time)) {
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

        if (!enqueue_sample(gyroscope_queue_, convert_gyroscope(x, y, z), sample_time)) {
            handle_queue_overflow("gyroscope", sample_time);
            return;
        }
        drain_ready_gyroscopes();
    }

    [[nodiscard]] bool initialized() const noexcept {
        auto guard = std::scoped_lock{mutex_};
        return filter_.initialized();
    }

    [[nodiscard]] Snapshot snapshot() const noexcept {
        auto guard = std::scoped_lock{mutex_};
        return snapshot_;
    }

private:
    struct TimedSample {
        Vec3 value_si = Vec3::Zero();
        time_point timestamp{};
    };

    [[nodiscard]] static const rclcpp::Logger& logger() {
        static const rclcpp::Logger logger = rclcpp::get_logger("bmi088_ekf");
        return logger;
    }

    [[nodiscard]] static seconds_duration to_seconds(time_point sample_time) noexcept {
        return std::chrono::duration_cast<seconds_duration>(sample_time.time_since_epoch());
    }

    [[nodiscard]] static nanoseconds to_nanoseconds(duration delta) noexcept {
        return std::chrono::duration_cast<nanoseconds>(delta);
    }

    [[nodiscard]] bool accept_sample_time(
        std::optional<time_point>& last_time, time_point sample_time,
        const char* stream_name) const {
        if (last_time.has_value() && sample_time <= *last_time) {
            RCLCPP_WARN(
                logger(), "Dropping non-monotonic %s sample at %.9f s (last=%.9f s)", stream_name,
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

    [[nodiscard]] Vec3
        convert_accelerometer(std::int16_t x, std::int16_t y, std::int16_t z) const noexcept {
        Vec3 raw_accel;
        raw_accel << static_cast<double>(x), static_cast<double>(y), static_cast<double>(z);
        return config_.sensor_to_filter * (raw_accel * accel_scale_mps2_);
    }

    [[nodiscard]] Vec3
        convert_gyroscope(std::int16_t x, std::int16_t y, std::int16_t z) const noexcept {
        Vec3 raw_gyro;
        raw_gyro << static_cast<double>(x), static_cast<double>(y), static_cast<double>(z);
        return config_.sensor_to_filter * (raw_gyro * gyro_scale_rad_per_sec_);
    }

    [[nodiscard]] bool enqueue_sample(
        std::deque<TimedSample>& queue, const Vec3& value_si, time_point sample_time) const {
        if (queue.size() >= queue_size_) {
            return false;
        }

        queue.push_back(TimedSample{.value_si = value_si, .timestamp = sample_time});
        return true;
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
            const auto gyro_rad_per_sec = gyro.value_si;
            if (!process_gyroscope_sample(gyro_rad_per_sec, gyro_time)) {
                RCLCPP_WARN(
                    logger(), "Gyroscope processing failed at %.9f s, clearing pending queues",
                    to_seconds(gyro_time).count());
                reset_after_failed_process();
                return;
            }

            gyroscope_queue_.pop_front();
            fused_time_ = gyro_time;
            refresh_snapshot();
        }
    }

    [[nodiscard]] bool
        process_gyroscope_sample(const Vec3& gyro_rad_per_sec, time_point gyro_time) {
        if (!fused_time_.has_value()) {
            return false;
        }

        auto cursor = *fused_time_;
        last_fused_gyro_ = gyro_rad_per_sec;
        while (!accelerometer_queue_.empty()
               && accelerometer_queue_.front().timestamp <= gyro_time) {
            const auto accel = accelerometer_queue_.front();
            if (!filter_.predict(gyro_rad_per_sec, to_nanoseconds(accel.timestamp - cursor))) {
                return false;
            }
            if (!filter_.update_from_accel(accel.value_si)) {
                return false;
            }

            cursor = accel.timestamp;
            accelerometer_queue_.pop_front();
        }

        return filter_.predict(gyro_rad_per_sec, to_nanoseconds(gyro_time - cursor));
    }

    void reset_after_failed_process() { reset_filter_state(); }

    void reset_filter_state() {
        filter_.reset();
        fused_time_.reset();
        accelerometer_watermark_.reset();
        last_fused_gyro_.setZero();
        clear_queues();
        refresh_snapshot();
    }

    void clear_queues() {
        accelerometer_queue_.clear();
        gyroscope_queue_.clear();
    }

    void refresh_snapshot() {
        snapshot_.initialized = filter_.initialized();
        if (!snapshot_.initialized) {
            snapshot_.quaternion = Vec4::Zero();
            snapshot_.gyro_body = Vec3::Zero();
            return;
        }

        snapshot_.quaternion = filter_.quaternion();
        snapshot_.gyro_body = last_fused_gyro_;
    }

    static constexpr double kRawSensorMaxValue = 32767.0;

    Config config_;
    std::size_t queue_size_ = 1;
    double accel_scale_mps2_ = 0.0;
    double gyro_scale_rad_per_sec_ = 0.0;
    mutable std::mutex mutex_;
    Filter filter_;
    std::deque<TimedSample> accelerometer_queue_;
    std::deque<TimedSample> gyroscope_queue_;
    std::optional<time_point> fused_time_;
    std::optional<time_point> accelerometer_watermark_;
    std::optional<time_point> last_accelerometer_time_;
    std::optional<time_point> last_gyroscope_time_;
    Vec3 last_fused_gyro_ = Vec3::Zero();
    Snapshot snapshot_;
};

} // namespace rmcs_core::hardware::device
