#pragma once

#include <atomic>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/force_sensor_mode.hpp>

namespace rmcs_core::hardware::device {

class ForceSensor {
public:
    explicit ForceSensor(rmcs_executor::Component& status_component) {

        status_component.register_output("/force_sensor/channel_1/weight", weight_ch1_, nan_);
        status_component.register_output("/force_sensor/channel_2/weight", weight_ch2_, nan_);
    }

    void store_status(const uint64_t can_data) {
        force_sensor_status_.store(std::bit_cast<ForceSensorStatus>(can_data), std::memory_order::relaxed);
    }

    void update_status() {
        auto status = force_sensor_status_.load(std::memory_order::relaxed);

        uint32_t ch1_weight = static_cast<uint32_t>(status.ch1_0) << 24 | static_cast<uint32_t>(status.ch1_1) << 16
                            | static_cast<uint32_t>(status.ch1_2) << 8 | static_cast<uint32_t>(status.ch1_3);
        uint32_t ch2_weight = static_cast<uint32_t>(status.ch2_0) << 24 | static_cast<uint32_t>(status.ch2_1) << 16
                            | static_cast<uint32_t>(status.ch2_2) << 8 | static_cast<uint32_t>(status.ch2_3);

        *weight_ch1_ = std::bit_cast<int>(ch1_weight);
        *weight_ch2_ = std::bit_cast<int>(ch2_weight);
    }

    static uint64_t generate_command() { return 0x00; }

private:
    static constexpr double nan_ = std::numeric_limits<int>::quiet_NaN();

    struct __attribute__((packed, aligned(8))) ForceSensorStatus {
        uint8_t ch1_0;
        uint8_t ch1_1;
        uint8_t ch1_2;
        uint8_t ch1_3;
        uint8_t ch2_0;
        uint8_t ch2_1;
        uint8_t ch2_2;
        uint8_t ch2_3;
    };
    std::atomic<ForceSensorStatus> force_sensor_status_{};
    static_assert(decltype(force_sensor_status_)::is_always_lock_free);

    rmcs_executor::Component::OutputInterface<int> weight_ch1_;
    rmcs_executor::Component::OutputInterface<int> weight_ch2_;
};

} // namespace rmcs_core::hardware::device