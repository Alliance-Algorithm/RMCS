#pragma once

#include <atomic>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {

/*
半双工通信设备，一发一收获取读数
并不支持高频，10Hz下精确度较高且通信稳定
*/

class ForceSensor {
public:
    explicit ForceSensor(rmcs_executor::Component& status_component) {
        status_component.register_output("/force_sensor/weight/channel1", weight_channel1_, nan_);
        status_component.register_output("/force_sensor/weight/channel2", weight_channel2_, nan_);
    }

    enum class Mode { ZERO_CALIBRATE, MEASUREMENT };

    void store_status(const uint64_t can_data) {
        force_sensor_status_.store(
            std::bit_cast<ForceSensorStatus>(can_data), std::memory_order::relaxed);
    }

    void update_status() {
        auto status = force_sensor_status_.load(std::memory_order::relaxed);

        uint32_t ch1_weight = static_cast<uint32_t>(status.ch1_weight_0) << 24
                            | static_cast<uint32_t>(status.ch1_weight_1) << 16
                            | static_cast<uint32_t>(status.ch1_weight_2) << 8
                            | static_cast<uint32_t>(status.ch1_weight_3);
        uint32_t ch2_weight = static_cast<uint32_t>(status.ch2_weight_0) << 24
                            | static_cast<uint32_t>(status.ch2_weight_1) << 16
                            | static_cast<uint32_t>(status.ch2_weight_2) << 8
                            | static_cast<uint32_t>(status.ch2_weight_3);

        *weight_channel1_ = std::bit_cast<int>(ch1_weight);
        *weight_channel2_ = std::bit_cast<int>(ch2_weight);
    }

    static uint32_t get_can_id(Mode mode, uint32_t devive_id = 1) {
        if (mode == Mode::MEASUREMENT) {
            return 0x03 << 8 | devive_id;
        } else if (mode == Mode::ZERO_CALIBRATE) {
            return 0x02 << 8 | devive_id;
        } else {
            return 0;
        }
    }
    static uint64_t generate_read_weight_command() { return 0x00; }

    static uint64_t generate_zero_calibration_command(int channel = 0) {
        if (channel == 1) {
            return 0x01;
        }
        if (channel == 2) {
            return 0x02;
        } else {
            return 0x0F;
        }
    }

private:
    static constexpr double nan_ = std::numeric_limits<int>::quiet_NaN();

    struct __attribute__((packed, aligned(8))) ForceSensorStatus {
        uint8_t ch1_weight_0;
        uint8_t ch1_weight_1;
        uint8_t ch1_weight_2;
        uint8_t ch1_weight_3;
        uint8_t ch2_weight_0;
        uint8_t ch2_weight_1;
        uint8_t ch2_weight_2;
        uint8_t ch2_weight_3;
    };
    std::atomic<ForceSensorStatus> force_sensor_status_{};
    static_assert(decltype(force_sensor_status_)::is_always_lock_free);

    rmcs_executor::Component::OutputInterface<int> weight_channel1_;
    rmcs_executor::Component::OutputInterface<int> weight_channel2_;
};

} // namespace rmcs_core::hardware::device