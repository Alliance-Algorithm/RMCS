#pragma once

#include <eigen3/Eigen/Dense>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <mutex>
#include <numbers>
#include <optional>
#include <utility>

#include <librmcs/data/datas.hpp>
#include <rmcs_utility/ring_buffer.hpp>

#include "imu_ekf/imu_ekf.hpp"

class Bmi088Ekf {
public:
    using Filter = imu_ekf::QuaternionEkf;
    using Mat3 = Filter::Mat3;
    using Vec3 = Filter::Vec3;
    using Vec4 = Filter::Vec4;

    struct Config {
        Config() {
            ekf.apply_sensor_calibration = false;
        }

        Filter::Config ekf;
        Mat3 sensor_to_filter = Mat3::Identity();
        double accel_full_scale_g = 6.0;
        double gyro_full_scale_deg_per_sec = 2000.0;
        std::size_t accel_queue_size = 8;
    };

    struct Snapshot {
        bool initialized = false;
        std::int64_t timestamp_quarter_us = 0;
        double timestamp_sec = 0.0;
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
        refresh_snapshot(0);
    }

    void on_accelerometer(const librmcs::data::AccelerometerDataView& data) {
        auto guard = std::scoped_lock{mutex_};

        const auto timestamp_quarter_us =
            lift_timestamp(accel_clock_, data.timestamp_quarter_us, "accelerometer");
        if (!timestamp_quarter_us.has_value())
            return;

        if (filter_time_quarter_us_.has_value() && *timestamp_quarter_us < *filter_time_quarter_us_) {
            std::fprintf(
                stderr,
                "Bmi088Ekf: dropping late accelerometer sample at %lld qus (filter=%lld qus)\n",
                static_cast<long long>(*timestamp_quarter_us),
                static_cast<long long>(*filter_time_quarter_us_));
            return;
        }

        const Vec3 accel_mps2 = convert_accelerometer(data.x, data.y, data.z);
        if (!filter_.initialized()) {
            if (try_initialize(accel_mps2, *timestamp_quarter_us))
                return;

            std::fprintf(
                stderr,
                "Bmi088Ekf: accelerometer initialization failed at %lld qus\n",
                static_cast<long long>(*timestamp_quarter_us));
            return;
        }

        push_accel_sample(accel_mps2, *timestamp_quarter_us);
    }

    void on_gyroscope(const librmcs::data::GyroscopeDataView& data) {
        auto guard = std::scoped_lock{mutex_};

        const auto timestamp_quarter_us =
            lift_timestamp(gyro_clock_, data.timestamp_quarter_us, "gyroscope");
        if (!timestamp_quarter_us.has_value())
            return;

        if (filter_time_quarter_us_.has_value() && *timestamp_quarter_us < *filter_time_quarter_us_) {
            std::fprintf(
                stderr,
                "Bmi088Ekf: dropping late gyroscope sample at %lld qus (filter=%lld qus)\n",
                static_cast<long long>(*timestamp_quarter_us),
                static_cast<long long>(*filter_time_quarter_us_));
            return;
        }

        if (!filter_.initialized())
            return;

        std::optional<Filter::TimedSample> latest_accel_sample;
        while (auto* queued_sample = accel_queue_.peek_front()) {
            if (queued_sample->timestamp_quarter_us > *timestamp_quarter_us)
                break;

            latest_accel_sample = to_timed_sample(*queued_sample);
            if (!accel_queue_.pop_front([](AccelSample&&) noexcept {})) {
                std::fprintf(stderr, "Bmi088Ekf: failed to pop accelerometer queue front\n");
                return;
            }
        }

        const auto gyro_sample = Filter::TimedSample{
            .value = convert_gyroscope(data.x, data.y, data.z),
            .ready_timestamp = to_seconds(*timestamp_quarter_us),
            .valid = true,
        };

        if (!filter_.process(gyro_sample, latest_accel_sample, to_seconds(*timestamp_quarter_us))) {
            std::fprintf(
                stderr,
                "Bmi088Ekf: gyroscope processing failed at %lld qus, clearing pending accel queue\n",
                static_cast<long long>(*timestamp_quarter_us));
            filter_.reset();
            filter_time_quarter_us_ = *timestamp_quarter_us;
            accel_queue_.clear();
            refresh_snapshot(0);
            return;
        }

        filter_time_quarter_us_ = *timestamp_quarter_us;
        refresh_snapshot(*timestamp_quarter_us);
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
    struct StreamClock {
        bool initialized = false;
        std::uint32_t last_raw_timestamp_quarter_us = 0;
        std::int64_t last_lifted_timestamp_quarter_us = 0;
    };

    struct AccelSample {
        double x_mps2 = 0.0;
        double y_mps2 = 0.0;
        double z_mps2 = 0.0;
        std::int64_t timestamp_quarter_us = 0;

        [[nodiscard]] auto vector() const noexcept -> Vec3 {
            Vec3 result;
            result << x_mps2, y_mps2, z_mps2;
            return result;
        }
    };

    static constexpr double kQuarterUsToSeconds = 1.0 / 4'000'000.0;

    [[nodiscard]] static auto to_seconds(const std::int64_t timestamp_quarter_us) noexcept -> double {
        return static_cast<double>(timestamp_quarter_us) * kQuarterUsToSeconds;
    }

    [[nodiscard]] auto lift_timestamp(
        StreamClock& stream_clock,
        const std::uint32_t raw_timestamp_quarter_us,
        const char* stream_name) -> std::optional<std::int64_t> {
        if (!stream_clock.initialized) {
            stream_clock.initialized = true;
            stream_clock.last_raw_timestamp_quarter_us = raw_timestamp_quarter_us;

            if (latest_unwrapped_timestamp_quarter_us_.has_value()) {
                const auto anchor_low32 =
                    static_cast<std::uint32_t>(*latest_unwrapped_timestamp_quarter_us_);
                const auto signed_offset =
                    static_cast<std::int32_t>(raw_timestamp_quarter_us - anchor_low32);
                stream_clock.last_lifted_timestamp_quarter_us =
                    *latest_unwrapped_timestamp_quarter_us_ + static_cast<std::int64_t>(signed_offset);
            } else {
                stream_clock.last_lifted_timestamp_quarter_us = raw_timestamp_quarter_us;
            }

            update_latest_unwrapped_timestamp(stream_clock.last_lifted_timestamp_quarter_us);
            return stream_clock.last_lifted_timestamp_quarter_us;
        }

        const auto delta = static_cast<std::int32_t>(
            raw_timestamp_quarter_us - stream_clock.last_raw_timestamp_quarter_us);
        if (delta <= 0) {
            std::fprintf(
                stderr,
                "Bmi088Ekf: dropping non-monotonic %s sample: raw=%u last=%u\n",
                stream_name,
                raw_timestamp_quarter_us,
                stream_clock.last_raw_timestamp_quarter_us);
            return std::nullopt;
        }

        stream_clock.last_raw_timestamp_quarter_us = raw_timestamp_quarter_us;
        stream_clock.last_lifted_timestamp_quarter_us += static_cast<std::int64_t>(delta);
        update_latest_unwrapped_timestamp(stream_clock.last_lifted_timestamp_quarter_us);
        return stream_clock.last_lifted_timestamp_quarter_us;
    }

    void update_latest_unwrapped_timestamp(const std::int64_t timestamp_quarter_us) {
        if (!latest_unwrapped_timestamp_quarter_us_.has_value()
            || timestamp_quarter_us > *latest_unwrapped_timestamp_quarter_us_) {
            latest_unwrapped_timestamp_quarter_us_ = timestamp_quarter_us;
        }
    }

    [[nodiscard]] auto convert_accelerometer(
        const std::int16_t x,
        const std::int16_t y,
        const std::int16_t z) const noexcept -> Vec3 {
        Vec3 raw_accel;
        raw_accel << static_cast<double>(x), static_cast<double>(y), static_cast<double>(z);

        const double scale =
            config_.accel_full_scale_g * filter_.config().gravity_acceleration / 32767.0;
        return config_.sensor_to_filter * (raw_accel * scale);
    }

    [[nodiscard]] auto convert_gyroscope(
        const std::int16_t x,
        const std::int16_t y,
        const std::int16_t z) const noexcept -> Vec3 {
        Vec3 raw_gyro;
        raw_gyro << static_cast<double>(x), static_cast<double>(y), static_cast<double>(z);

        const double scale = config_.gyro_full_scale_deg_per_sec / 32767.0
                           * std::numbers::pi_v<double> / 180.0;
        return config_.sensor_to_filter * (raw_gyro * scale);
    }

    [[nodiscard]] static auto to_timed_sample(const AccelSample& sample) -> Filter::TimedSample {
        return Filter::TimedSample{
            .value = sample.vector(),
            .ready_timestamp = to_seconds(sample.timestamp_quarter_us),
            .valid = true,
        };
    }

    [[nodiscard]] auto try_initialize(
        const Vec3& accel_mps2,
        const std::int64_t timestamp_quarter_us) -> bool {
        const auto accel_sample = Filter::TimedSample{
            .value = accel_mps2,
            .ready_timestamp = to_seconds(timestamp_quarter_us),
            .valid = true,
        };

        if (!filter_.process(std::nullopt, accel_sample, to_seconds(timestamp_quarter_us)))
            return false;

        filter_time_quarter_us_ = timestamp_quarter_us;
        accel_queue_.clear();
        refresh_snapshot(timestamp_quarter_us);
        return true;
    }

    void push_accel_sample(const Vec3& accel_mps2, const std::int64_t timestamp_quarter_us) {
        AccelSample sample;
        sample.x_mps2 = accel_mps2.x();
        sample.y_mps2 = accel_mps2.y();
        sample.z_mps2 = accel_mps2.z();
        sample.timestamp_quarter_us = timestamp_quarter_us;

        if (accel_queue_.emplace_back(sample))
            return;

        std::fprintf(
            stderr,
            "Bmi088Ekf: accelerometer queue full at %lld qus, dropping oldest sample\n",
            static_cast<long long>(timestamp_quarter_us));
        if (!accel_queue_.pop_front([](AccelSample&&) noexcept {})) {
            std::fprintf(stderr, "Bmi088Ekf: failed to drop oldest accelerometer sample\n");
            return;
        }

        if (!accel_queue_.emplace_back(sample)) {
            std::fprintf(stderr, "Bmi088Ekf: failed to enqueue accelerometer sample after drop\n");
        }
    }

    void refresh_snapshot(const std::int64_t timestamp_quarter_us) {
        snapshot_.initialized = filter_.initialized();
        snapshot_.timestamp_quarter_us = snapshot_.initialized ? timestamp_quarter_us : 0;
        snapshot_.timestamp_sec = to_seconds(snapshot_.timestamp_quarter_us);
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
    rmcs_utility::RingBuffer<AccelSample> accel_queue_;
    std::optional<std::int64_t> filter_time_quarter_us_;
    std::optional<std::int64_t> latest_unwrapped_timestamp_quarter_us_;
    StreamClock accel_clock_;
    StreamClock gyro_clock_;
    Snapshot snapshot_;
};
