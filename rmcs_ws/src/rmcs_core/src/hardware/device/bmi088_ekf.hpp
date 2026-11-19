#pragma once

#include <eigen3/Eigen/Geometry>

#include <chrono>
#include <cstdint>
#include <limits>
#include <mutex>
#include <numbers>
#include <optional>
#include <utility>

#include <rmcs_msgs/board_clock.hpp>
#include <rmcs_msgs/imu_snapshot.hpp>

#include "filter/imu_ekf.hpp"

namespace rmcs_core::hardware::device {

class Bmi088Ekf {
public:
    struct Config {
        Eigen::Matrix3d body_to_sensor = Eigen::Matrix3d::Identity();
        Eigen::Vector3d sensor_position_body = Eigen::Vector3d::Zero();
    };

    using TimePoint = rmcs_msgs::BoardClock::time_point;
    using Snapshot = rmcs_msgs::ImuSnapshot;

    struct Diagnostics {
        double chi_square = std::numeric_limits<double>::quiet_NaN();
        std::optional<TimePoint> correction_time;
        bool correction_attempted = false;
        bool correction_applied = false;
        bool gyro_saturated = false;
        bool time_gap_rejected = false;
    };

    explicit Bmi088Ekf(Config config)
        : config_(std::move(config)) {}

    Bmi088Ekf()
        : Bmi088Ekf(Config{}) {}

    void push_accelerometer_sample(
        std::int16_t x, std::int16_t y, std::int16_t z, TimePoint sample_time) {
        const Eigen::Vector3d accel_g =
            config_.body_to_sensor.transpose() * convert_accelerometer(x, y, z);

        if (!initialized_) {
            if (ekf_.reset_from_accel(accel_g)) {
                ekf_state_time_ = sample_time;
                latest_snapshot_ = {
                    ekf_.quaternion(),
                    Eigen::Vector3d::Zero(),
                    ekf_state_time_,
                };
                const auto guard = std::scoped_lock{mutex_};
                initialized_ = true;
            }
            return;
        }

        if (sample_time < ekf_state_time_)
            return;

        if (pending_accel_sample_ && sample_time < pending_accel_sample_->sample_time)
            return;

        pending_accel_sample_ = {accel_g, sample_time};
    }

    std::optional<Snapshot> try_update_with_gyroscope_sample(
        std::int16_t x, std::int16_t y, std::int16_t z, TimePoint sample_time,
        Diagnostics* diagnostics = nullptr) {
        if (diagnostics)
            *diagnostics = {};
        const Eigen::Vector3d gyro_rad_per_sec =
            config_.body_to_sensor.transpose() * convert_gyroscope(x, y, z);

        if (!initialized_)
            return std::nullopt;

        if (sample_time < ekf_state_time_)
            return std::nullopt;

        const GyroSample gyro_sample{gyro_rad_per_sec, sample_time};
        std::optional<GyroSample> previous_gyro;
        if (is_gyro_saturated(gyro_rad_per_sec)) {
            if (diagnostics)
                diagnostics->gyro_saturated = true;
            previous_gyro_sample_.reset();
            ekf_.inflate_attitude_uncertainty_to_initial();
        } else if (!config_.sensor_position_body.isZero(0.0)) {
            previous_gyro = std::exchange(previous_gyro_sample_, gyro_sample);
        }

        while (pending_accel_sample_) {
            const auto& accel_sample_time = pending_accel_sample_->sample_time;
            if (accel_sample_time < ekf_state_time_ || accel_sample_time > sample_time)
                break;

            if (!ekf_.predict(
                    gyro_rad_per_sec,
                    std::chrono::duration<double>{accel_sample_time - ekf_state_time_}.count()))
                break;
            ekf_state_time_ = accel_sample_time;

            const auto accel_g =
                acceleration_at_body_origin(*pending_accel_sample_, previous_gyro, gyro_sample);
            const auto correction = ekf_.prepare_correction(accel_g);
            if (diagnostics) {
                diagnostics->correction_attempted = true;
                diagnostics->correction_time = accel_sample_time;
                if (correction)
                    diagnostics->chi_square = correction->chi_square();
            }
            if (!correction || correction->chi_square() >= 3.0)
                break;
            if (!ekf_.correct(*correction))
                break;
            if (diagnostics)
                diagnostics->correction_applied = true;

            pending_accel_sample_ = std::nullopt;
            break;
        }

        // Guard against stale IMU frames that librmcs may deliver right after reconnect before the
        // device-side buffer is drained. Integrating a frame with a large timestamp jump can inject
        // a huge bogus gyro delta, so drop it instead of advancing the filter.
        // TODO: Remove this once librmcs guarantees buffered historical IMU frames are flushed on
        // connection.
        if (sample_time - ekf_state_time_ > kMaxGyroGap) {
            if (diagnostics)
                diagnostics->time_gap_rejected = true;
            ekf_state_time_ = sample_time;
            return std::nullopt;
        }

        if (!ekf_.predict(
                gyro_rad_per_sec,
                std::chrono::duration<double>{sample_time - ekf_state_time_}.count()))
            return std::nullopt;
        ekf_state_time_ = sample_time;

        auto snapshot = Snapshot{
            ekf_.quaternion(),
            gyro_rad_per_sec,
            ekf_state_time_,
        };
        {
            const auto guard = std::scoped_lock{mutex_};
            latest_snapshot_ = snapshot;
        }
        return snapshot;
    }

    [[nodiscard]] bool initialized() const noexcept {
        const auto guard = std::scoped_lock{mutex_};
        return initialized_;
    }

    [[nodiscard]] std::optional<Snapshot> snapshot() const noexcept {
        const auto guard = std::scoped_lock{mutex_};
        if (!initialized_)
            return std::nullopt;
        return latest_snapshot_;
    }

private:
    struct AccelSample {
        Eigen::Vector3d accel_g;
        TimePoint sample_time;
    };

    struct GyroSample {
        Eigen::Vector3d angular_velocity;
        TimePoint sample_time;
    };

    static constexpr auto kMaxGyroGap = std::chrono::milliseconds{1};

    [[nodiscard]] Eigen::Vector3d acceleration_at_body_origin(
        const AccelSample& accel, const std::optional<GyroSample>& previous,
        const GyroSample& current) const {
        if (!previous || accel.sample_time < previous->sample_time
            || accel.sample_time > current.sample_time
            || current.sample_time <= previous->sample_time)
            return accel.accel_g;

        const auto interval = current.sample_time - previous->sample_time;
        if (interval > kMaxGyroGap)
            return accel.accel_g;

        const Eigen::Vector3d alpha = (current.angular_velocity - previous->angular_velocity)
                                    / std::chrono::duration<double>{interval}.count();
        const double elapsed =
            std::chrono::duration<double>{accel.sample_time - previous->sample_time}.count();
        const Eigen::Vector3d omega = previous->angular_velocity + alpha * elapsed;
        // The lever arm points from the body origin to the accelerometer.
        const auto& r = config_.sensor_position_body;
        return accel.accel_g - (alpha.cross(r) + omega.cross(omega.cross(r))) / 9.81;
    }

    [[nodiscard]] static Eigen::Vector3d
        convert_accelerometer(std::int16_t x, std::int16_t y, std::int16_t z) noexcept {
        return Eigen::Vector3d{
                   static_cast<double>(x), static_cast<double>(y), static_cast<double>(z)}
             / 32767.0 * 6.0;
    }

    [[nodiscard]] static Eigen::Vector3d
        convert_gyroscope(std::int16_t x, std::int16_t y, std::int16_t z) noexcept {
        return Eigen::Vector3d{
                   static_cast<double>(x), static_cast<double>(y), static_cast<double>(z)}
             / 32767.0 * 2000.0 / 180.0 * std::numbers::pi;
    }

    [[nodiscard]] static bool is_gyro_saturated(const Eigen::Vector3d& gyro_rad_per_sec) noexcept {
        return gyro_rad_per_sec.cwiseAbs().maxCoeff() >= 0.98 * 2000.0 / 180.0 * std::numbers::pi;
    }

    const Config config_;

    mutable std::mutex mutex_;
    bool initialized_ = false;

    filter::ImuEkf ekf_;
    TimePoint ekf_state_time_;

    std::optional<AccelSample> pending_accel_sample_;
    std::optional<GyroSample> previous_gyro_sample_;

    Snapshot latest_snapshot_;
};

} // namespace rmcs_core::hardware::device
