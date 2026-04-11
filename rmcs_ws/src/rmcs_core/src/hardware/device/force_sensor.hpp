#pragma once

#include "filter/mean_low_pass_filter.hpp"
#include "hardware/device/can_packet.hpp"
#include <atomic>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {

class ForceSensor {
public:
    explicit ForceSensor(rmcs_executor::Component& status_component)
        : filter_ch1_(10, 0.2)
        , filter_ch2_(10, 0.2) {
        status_component.register_output("/force_sensor/channel_1/weight", weight_ch1_, nan_);
        status_component.register_output("/force_sensor/channel_2/weight", weight_ch2_, nan_);
    }

    void store_status(std::span<const std::byte> can_data) {
        ForceSensorStatus status{};
        std::memcpy(&status, can_data.data(),
                    std::min(can_data.size(), sizeof(ForceSensorStatus)));
        force_sensor_status_.store(status, std::memory_order_relaxed);
    }

    void update_status() {
        auto status = force_sensor_status_.load(std::memory_order::relaxed);

        uint32_t ch1_weight =
            static_cast<uint32_t>(status.ch1_0) << 24 | static_cast<uint32_t>(status.ch1_1) << 16
            | static_cast<uint32_t>(status.ch1_2) << 8  | static_cast<uint32_t>(status.ch1_3);
        uint32_t ch2_weight =
            static_cast<uint32_t>(status.ch2_0) << 24 | static_cast<uint32_t>(status.ch2_1) << 16
            | static_cast<uint32_t>(status.ch2_2) << 8  | static_cast<uint32_t>(status.ch2_3);

        double raw_ch1 = static_cast<double>(std::bit_cast<int>(ch1_weight));
        double raw_ch2 = static_cast<double>(std::bit_cast<int>(ch2_weight));

        *weight_ch1_ = static_cast<int>(std::round(filter_ch1_.update(raw_ch1)));
        *weight_ch2_ = static_cast<int>(std::round(filter_ch2_.update(raw_ch2)));
    }

    // static uint64_t generate_command() { return 0x00; }
    static uint64_t generate_zero_calibration_command() { return 0x0F; }


    static CanPacket8::Quarter generate_command(){return CanPacket8::Quarter{0};}

private:
    static constexpr double nan_ = std::numeric_limits<int>::quiet_NaN();

    struct __attribute__((packed, aligned(8))) ForceSensorStatus {
        uint8_t ch1_0, ch1_1, ch1_2, ch1_3;
        uint8_t ch2_0, ch2_1, ch2_2, ch2_3;
    };
    std::atomic<ForceSensorStatus> force_sensor_status_{};
    static_assert(decltype(force_sensor_status_)::is_always_lock_free);

    rmcs_executor::Component::OutputInterface<int> weight_ch1_;
    rmcs_executor::Component::OutputInterface<int> weight_ch2_;

    filter::MeanLowPassFilter<> filter_ch1_;
    filter::MeanLowPassFilter<> filter_ch2_;
};

} // namespace rmcs_core::hardware::device