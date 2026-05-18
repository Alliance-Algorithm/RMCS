#pragma once

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <utility>

#include "bmi088.hpp"

#include <librmcs/agent/rmcs_board_lite.hpp>
#include <librmcs/data/datas.hpp>
#include <librmcs/spec/rmcs_board_lite/gpio.hpp>

class TriggerBoard : public librmcs::agent::RmcsBoardLite {
public:
    struct Clock {
        // 250ns / tick
        using duration = std::chrono::duration<std::int64_t, std::ratio<1, 4'000'000>>;
        using rep = duration::rep;
        using period = duration::period;
        using time_point = std::chrono::time_point<Clock>;

        [[maybe_unused]] static constexpr bool is_steady = true;
    };

    using SignalCallback = std::function<void(Clock::time_point)>;
    using ImuConfig = Bmi088Ekf::Config;
    using ImuSnapshot = Bmi088Ekf::Snapshot;

    explicit TriggerBoard(SignalCallback&& callback)
        : TriggerBoard(std::move(callback), ImuConfig{}) {}

    explicit TriggerBoard(
        SignalCallback&& callback, ImuConfig imu_config)
        : imu_ekf_(std::move(imu_config))
        , callback_(std::move(callback)) {
        start_transmit().gpio_digital_read(
            librmcs::spec::rmcs_board_lite::kGpioDescriptors.kUart1Tx,
            {
                .period_ms = 0,
                .asap = false,
                .rising_edge = false,
                .falling_edge = true,
                .capture_timestamp = true,
                .pull = librmcs::data::GpioPull::kUp,
            });
    }

    [[nodiscard]] auto imu_initialized() const noexcept -> bool {
        return imu_ekf_.initialized();
    }

    [[nodiscard]] auto imu_snapshot() const noexcept -> ImuSnapshot {
        return imu_ekf_.snapshot();
    }

private:
    void accelerometer_receive_callback(const librmcs::data::AccelerometerDataView& data) override {
        if (!has_latest_accelerometer_timestamp_) {
            imu_ekf_.process_accelerometer(to_accelerometer_sample(data));
            has_latest_accelerometer_timestamp_ = true;
            last_accelerometer_timestamp_raw_ = data.timestamp_quarter_us;
            latest_accelerometer_timestamp_ = data.timestamp_quarter_us;
            return;
        }

        const auto delta = static_cast<std::int32_t>(
            data.timestamp_quarter_us - last_accelerometer_timestamp_raw_);
        if (delta <= 0) {
            std::fprintf(
                stderr,
                "TriggerBoard: dropping non-monotonic accelerometer sample: raw=%u last=%u\n",
                data.timestamp_quarter_us,
                last_accelerometer_timestamp_raw_);
            return;
        }

        imu_ekf_.process_accelerometer(to_accelerometer_sample(data));
        latest_accelerometer_timestamp_ += static_cast<std::int64_t>(delta);
        last_accelerometer_timestamp_raw_ = data.timestamp_quarter_us;
    }

    void gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data) override {
        if (!has_latest_gyroscope_timestamp_) {
            has_latest_gyroscope_timestamp_ = true;
            last_gyroscope_timestamp_raw_ = data.timestamp_quarter_us;
            imu_ekf_.process_gyroscope(to_gyroscope_sample(data));
            return;
        }

        const auto delta = static_cast<std::int32_t>(
            data.timestamp_quarter_us - last_gyroscope_timestamp_raw_);
        if (delta <= 0) {
            std::fprintf(
                stderr,
                "TriggerBoard: dropping non-monotonic gyroscope sample: raw=%u last=%u\n",
                data.timestamp_quarter_us,
                last_gyroscope_timestamp_raw_);
            return;
        }

        last_gyroscope_timestamp_raw_ = data.timestamp_quarter_us;
        imu_ekf_.process_gyroscope(to_gyroscope_sample(data));
    }

    void gpio_digital_read_result_callback(
        const librmcs::spec::rmcs_board_lite::GpioDescriptor& gpio,
        const librmcs::data::GpioDigitalDataView& data) override {
        if (gpio != librmcs::spec::rmcs_board_lite::kGpioDescriptors.kUart1Tx)
            return;
        if (!data.timestamp_quarter_us)
            return;
        if (!callback_)
            return;

        if (!has_latest_accelerometer_timestamp_)
            return;
        Clock::time_point timestamp;

        const auto latest_timestamp_low32 =
            static_cast<std::uint32_t>(latest_accelerometer_timestamp_);
        const auto signed_offset =
            static_cast<std::int32_t>(*data.timestamp_quarter_us - latest_timestamp_low32);
        const auto lifted_timestamp =
            latest_accelerometer_timestamp_ + static_cast<std::int64_t>(signed_offset);
        timestamp = Clock::time_point{Clock::duration{lifted_timestamp}};

        callback_(timestamp);
    }

    [[nodiscard]] static auto to_accelerometer_sample(
        const librmcs::data::AccelerometerDataView& data) noexcept -> Bmi088Ekf::AccelerometerSample {
        return Bmi088Ekf::AccelerometerSample{
            .x = data.x,
            .y = data.y,
            .z = data.z,
            .timestamp_quarter_us = data.timestamp_quarter_us,
        };
    }

    [[nodiscard]] static auto to_gyroscope_sample(
        const librmcs::data::GyroscopeDataView& data) noexcept -> Bmi088Ekf::GyroscopeSample {
        return Bmi088Ekf::GyroscopeSample{
            .x = data.x,
            .y = data.y,
            .z = data.z,
            .timestamp_quarter_us = data.timestamp_quarter_us,
        };
    }

    bool has_latest_accelerometer_timestamp_ = false;
    bool has_latest_gyroscope_timestamp_ = false;
    std::uint32_t last_accelerometer_timestamp_raw_ = 0;
    std::uint32_t last_gyroscope_timestamp_raw_ = 0;
    std::int64_t latest_accelerometer_timestamp_ = 0;

    Bmi088Ekf imu_ekf_;
    SignalCallback callback_;
};
