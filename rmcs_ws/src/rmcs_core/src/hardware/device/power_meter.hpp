#pragma once

#include <atomic>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"

namespace rmcs_core::hardware::device {

class PowerMeter {
public:
    struct Config {
        explicit Config() = default;
    };

    PowerMeter(rmcs_executor::Component& status_component, const std::string& name_prefix) {
        status_component.register_output(name_prefix + "/voltage", voltage_output_, 0.0);
        status_component.register_output(name_prefix + "/current", current_output_, 0.0);
        status_component.register_output(name_prefix + "/power", power_output_, 0.0);
    }

    PowerMeter(
        rmcs_executor::Component& status_component, const std::string& name_prefix,
        const Config&)
        : PowerMeter(status_component, name_prefix) {}

    PowerMeter(const PowerMeter&) = delete;
    PowerMeter& operator=(const PowerMeter&) = delete;
    PowerMeter(PowerMeter&&) = delete;
    PowerMeter& operator=(PowerMeter&&) = delete;

    ~PowerMeter() = default;

    void store_status(std::span<const std::byte> can_data) {
        can_packet_.store(CanPacket8{can_data}, std::memory_order::relaxed);
    }

    void update_status() {
        const auto feedback =
            std::bit_cast<Feedback>(can_packet_.load(std::memory_order::relaxed));

        voltage_ = static_cast<double>(feedback.voltage)
                 * (kRangeConversionVoltage / static_cast<double>(kRangeConversionFactor));
        current_ = static_cast<double>(feedback.current)
                 * (kRangeConversionCurrent / static_cast<double>(kRangeConversionFactor));
        power_ = static_cast<double>(feedback.power)
               * (kRangeConversionPower / static_cast<double>(kRangeConversionFactor));

        *voltage_output_ = voltage();
        *current_output_ = current();
        *power_output_ = power();
    }

    double voltage() const { return voltage_; }
    double current() const { return current_; }
    double power() const { return power_; }

private:
    struct alignas(uint64_t) Feedback {
        uint16_t voltage;
        uint16_t current;
        uint16_t power;
        uint16_t unused;
    };

    static constexpr int kRangeConversionFactor = 65535;
    static constexpr double kRangeConversionVoltage = 65.535;
    static constexpr double kRangeConversionCurrent = 65.535;
    static constexpr double kRangeConversionPower = 650.535;

    std::atomic<CanPacket8> can_packet_{CanPacket8{uint64_t{0}}};

    double voltage_ = 0.0;
    double current_ = 0.0;
    double power_ = 0.0;

    rmcs_executor::Component::OutputInterface<double> voltage_output_;
    rmcs_executor::Component::OutputInterface<double> current_output_;
    rmcs_executor::Component::OutputInterface<double> power_output_;
};

} // namespace rmcs_core::hardware::device
