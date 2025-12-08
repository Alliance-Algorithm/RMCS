#pragma once
#include "hardware/endian_promise.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {
using rmcs_executor::Component;
struct PowerMeterConfig {
    explicit PowerMeterConfig() = default;
};
class PowerMeter {
public:
    PowerMeter(Component& status_component, const std::string& name_prefix) {
        status_component.register_output(name_prefix + "/voltage", voltage_, 0.0f);
        status_component.register_output(name_prefix + "/current", current_, 0.0f);
        status_component.register_output(name_prefix + "/power", power_, 0.0f);
    }

    PowerMeter(const PowerMeter&)            = delete;
    PowerMeter& operator=(const PowerMeter&) = delete;

    void store_status(uint64_t can_result) {
        can_data_.store(can_result, std::memory_order::relaxed);
    }

    void update() {
        auto feedback =
            std::bit_cast<PowerMeterFeedback>(can_data_.load(std::memory_order::relaxed));

        *voltage_ = static_cast<double>(feedback.voltage)
                  * (range_conversion_voltage_ / range_conversion_factor);
        *current_ = static_cast<double>(feedback.current)
                  * (range_conversion_current_ / range_conversion_factor);
        *power_ = static_cast<double>(feedback.power)
                * (range_conversion_power_ / range_conversion_factor);
        if (*power_ >= 120.0f)[[unlikely]] {
            RCLCPP_WARN(
                rclcpp::get_logger("PowerMeter"), "Power reading is abnormally high: %.2f W",
                *power_);
        }
    }

    double get_voltage() const { return *voltage_; }
    double get_current() const { return *current_; }
    double get_power() const { return *power_; }

private:
    struct alignas(uint64_t) PowerMeterFeedback {
        uint16_t voltage;
        uint16_t current;
        uint16_t power;
        uint16_t unused;
    };
    static constexpr int range_conversion_factor      = 65535;
    static constexpr double range_conversion_voltage_ = 65.535;
    static constexpr double range_conversion_current_ = 65.535;
    static constexpr double range_conversion_power_   = 650.535;
    std::atomic<uint64_t> can_data_                   = 0;
    Component::OutputInterface<double> voltage_;
    Component::OutputInterface<double> current_;
    Component::OutputInterface<double> power_;
};

} // namespace rmcs_core::hardware::device