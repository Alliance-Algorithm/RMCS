#pragma once

#include <eigen3/Eigen/Geometry>

#include <chrono>
#include <cstdint>
#include <mutex>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <utility>

#include <rmcs_msgs/board_clock.hpp>
#include <rmcs_msgs/imu_snapshot.hpp>

#include "filter/imu_ekf.hpp"
#include "hardware/device/imu_lever_arm.hpp"

namespace rmcs_core::hardware::device {

class Bmi088Ekf {
public:
    struct Config {
        Eigen::Matrix3d body_to_sensor = Eigen::Matrix3d::Identity();
        // Body origin -> accelerometer, in body axes, metres.
        Eigen::Vector3d sensor_position_body_m = Eigen::Vector3d::Zero();
        double angular_acceleration_filter_tau_s = 0.01;
    };

    using TimePoint = rmcs_msgs::BoardClock::time_point;
    using Snapshot = rmcs_msgs::ImuSnapshot;

    struct AccelerationSnapshot {
        Eigen::Vector3d specific_force_body_mps2;
        TimePoint timestamp;
    };

    explicit Bmi088Ekf(Config config)
        : config_(std::move(config))
        , lever_arm_(config_.sensor_position_body_m, config_.angular_acceleration_filter_tau_s) {
        if (!config_.body_to_sensor.allFinite()
            || std::abs(config_.body_to_sensor.determinant() - 1.0) > 1e-3
            || (config_.body_to_sensor.transpose() * config_.body_to_sensor
                - Eigen::Matrix3d::Identity())
                       .norm()
                   > 1e-3)
            throw std::invalid_argument("IMU installation must be a proper rotation matrix");
    }

    Bmi088Ekf()
        : Bmi088Ekf(Config{}) {}

    bool push_accelerometer_sample(
        std::int16_t x, std::int16_t y, std::int16_t z, TimePoint sample_time) {
        const Eigen::Vector3d accel_g =
            config_.body_to_sensor.transpose() * convert_accelerometer(x, y, z);

        if (lever_arm_.enabled()) {
            if ((initialized_ && sample_time <= ekf_state_time_)
                || (pending_accel_sample_ && sample_time <= pending_accel_sample_->sample_time))
                return false;
            // Wait for bracketing gyro samples before publishing or correcting the EKF.
            pending_accel_sample_ = {accel_g, sample_time};
            return true;
        }

        if (!initialized_) {
            if (ekf_.reset_from_accel(accel_g)) {
                ekf_state_time_ = sample_time;
                latest_snapshot_ = {
                    ekf_.quaternion(),
                    Eigen::Vector3d::Zero(),
                    ekf_state_time_,
                };
                const auto guard = std::scoped_lock{mutex_};
                latest_acceleration_ = AccelerationSnapshot{accel_g * 9.80665, sample_time};
                initialized_ = true;
                return true;
            }
            return false;
        }

        if (sample_time <= ekf_state_time_)
            return false;

        if (pending_accel_sample_ && sample_time <= pending_accel_sample_->sample_time)
            return false;

        {
            const auto guard = std::scoped_lock{mutex_};
            latest_acceleration_ = AccelerationSnapshot{accel_g * 9.80665, sample_time};
        }
        pending_accel_sample_ = {accel_g, sample_time};
        return true;
    }

    std::optional<Snapshot> try_update_with_gyroscope_sample(
        std::int16_t x, std::int16_t y, std::int16_t z, TimePoint sample_time) {
        const Eigen::Vector3d sensor_gyro = convert_gyroscope(x, y, z);
        const Eigen::Vector3d gyro_rad_per_sec = config_.body_to_sensor.transpose() * sensor_gyro;
        // Clipping occurs on the physical sensor axes, before the mounting rotation.
        const bool gyro_saturated = is_gyro_saturated(sensor_gyro);

        if (lever_arm_.enabled()) {
            if (gyro_saturated)
                lever_arm_.reset();
            else if (!lever_arm_.observe(gyro_rad_per_sec, sample_time))
                return std::nullopt;
        }

        if (!initialized_) {
            if (!lever_arm_.enabled() || !pending_accel_sample_)
                return std::nullopt;
            const auto& sample = *pending_accel_sample_;
            const auto corrected = lever_arm_.correct(sample.accel_g * 9.80665, sample.sample_time);
            if (!corrected || !ekf_.reset_from_accel(*corrected / 9.80665))
                return std::nullopt;
            ekf_state_time_ = sample.sample_time;
            const auto guard = std::scoped_lock{mutex_};
            latest_acceleration_ = AccelerationSnapshot{*corrected, sample.sample_time};
            latest_snapshot_ = {ekf_.quaternion(), gyro_rad_per_sec, ekf_state_time_};
            initialized_ = true;
            pending_accel_sample_.reset();
        }

        if (sample_time < ekf_state_time_)
            return std::nullopt;

        // Buffered frames after reconnect or a receive gap must not integrate the current gyro
        // over an unobserved interval. Check before the pending accelerometer prediction, which
        // otherwise advances the state time and hides the gap from this guard.
        // TODO: Remove this once librmcs guarantees historical IMU frames are flushed on
        // connection.
        if (std::chrono::duration<double>{sample_time - ekf_state_time_}.count()
            > ImuLeverArm::kMaximumSampleInterval) {
            ekf_state_time_ = sample_time;
            if (pending_accel_sample_ && pending_accel_sample_->sample_time <= sample_time)
                pending_accel_sample_ = std::nullopt;
            return std::nullopt;
        }

        if (gyro_saturated)
            ekf_.inflate_attitude_uncertainty_to_initial();

        while (pending_accel_sample_) {
            const auto& accel_sample_time = pending_accel_sample_->sample_time;
            if (accel_sample_time < ekf_state_time_ || accel_sample_time > sample_time)
                break;

            if (!ekf_.predict(
                    gyro_rad_per_sec,
                    std::chrono::duration<double>{accel_sample_time - ekf_state_time_}.count()))
                break;
            ekf_state_time_ = accel_sample_time;

            Eigen::Vector3d accel_g = pending_accel_sample_->accel_g;
            if (lever_arm_.enabled()) {
                const auto corrected = lever_arm_.correct(accel_g * 9.80665, accel_sample_time);
                if (!corrected) {
                    pending_accel_sample_.reset();
                    break;
                }
                accel_g = *corrected / 9.80665;
                const auto guard = std::scoped_lock{mutex_};
                latest_acceleration_ = AccelerationSnapshot{*corrected, accel_sample_time};
            }
            const auto correction = ekf_.prepare_correction(accel_g);
            if (!correction || correction->chi_square() >= 16.0)
                break;
            if (!ekf_.correct(*correction))
                break;

            pending_accel_sample_ = std::nullopt;
            break;
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

    // Called on the receive thread after a control reset. Preserve physical attitude and mount.
    void reset_motion_history() {
        lever_arm_.reset();
        if (lever_arm_.enabled()) {
            pending_accel_sample_.reset();
            const auto guard = std::scoped_lock{mutex_};
            latest_acceleration_.reset();
        }
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

    [[nodiscard]] std::optional<AccelerationSnapshot> acceleration_snapshot() const noexcept {
        const auto guard = std::scoped_lock{mutex_};
        return latest_acceleration_;
    }

private:
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
    ImuLeverArm lever_arm_;

    mutable std::mutex mutex_;
    bool initialized_ = false;

    filter::ImuEkf ekf_;
    TimePoint ekf_state_time_;

    struct AccelSample {
        Eigen::Vector3d accel_g;
        TimePoint sample_time;
    };
    std::optional<AccelSample> pending_accel_sample_;

    Snapshot latest_snapshot_;
    std::optional<AccelerationSnapshot> latest_acceleration_;
};

} // namespace rmcs_core::hardware::device
