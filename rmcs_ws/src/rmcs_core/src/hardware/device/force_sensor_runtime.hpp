#pragma once

#include <atomic>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/force_sensor_mode.hpp>

namespace rmcs_core::hardware::device {

class ForceSensorRuntime {
public:
    explicit ForceSensorRuntime(rmcs_executor::Component& status_component) {
        status_component.register_output("/force_sensor/channel_1/weight", channel_1_weight_, nan_);
        status_component.register_output("/force_sensor/channel_2/weight", channel_2_weight_, nan_);
    }

    void store_status(const uint64_t can_data) {
        force_sensor_status_.store(std::bit_cast<ForceSensorStatus>(can_data), std::memory_order::relaxed);
    }

    void update_status() {
        auto status = force_sensor_status_.load(std::memory_order::relaxed);

        uint32_t ch1_weight =
            static_cast<uint32_t>(status.ch1_weight_0) << 24 | static_cast<uint32_t>(status.ch1_weight_1) << 16
            | static_cast<uint32_t>(status.ch1_weight_2) << 8 | static_cast<uint32_t>(status.ch1_weight_3);
        uint32_t ch2_weight =
            static_cast<uint32_t>(status.ch2_weight_0) << 24 | static_cast<uint32_t>(status.ch2_weight_1) << 16
            | static_cast<uint32_t>(status.ch2_weight_2) << 8 | static_cast<uint32_t>(status.ch2_weight_3);

        *channel_1_weight_ = std::bit_cast<int>(ch1_weight);
        *channel_2_weight_ = std::bit_cast<int>(ch2_weight);
    }

    static uint64_t generate_command() { return 0x00; }
    static uint64_t generate_zero_calibration_command() { return 0x0F; }

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

    rmcs_executor::Component::OutputInterface<int> channel_1_weight_;
    rmcs_executor::Component::OutputInterface<int> channel_2_weight_;
};

} // namespace rmcs_core::hardware::device