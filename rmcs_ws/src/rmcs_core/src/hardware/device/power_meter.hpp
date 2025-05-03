#pragma once

#include <algorithm>
#include <cmath>

#include <atomic>
#include <cstdint>

#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {
using rmcs_executor::Component;

class Powermeter {
public:
    explicit Powermeter(Component& status_component, Component& command_component) {
        status_component.register_output("/chassis/steering/power", chassis_power_, 0.0);
        status_component.register_output("/chassis/steering/voltage", chassis_voltage_, 0.0);
        status_component.register_output("/chassis/powermeter/voltage", powermeter_voltage_, 0.0);
        status_component.register_output("/chassis/powermeter/enabled", powermeter_enabled_, false);

        command_component.register_input(
            "/chassis/powermeter/control_enable", powermeter_control_enabled_);
        command_component.register_input(
            "/chassis/powermeter/charge_power_limit", powermeter_charge_power_limit_);
    }

    void store_status(uint64_t can_data) {
        can_data_.store(std::bit_cast<PowermeterStatus>(can_data), std::memory_order::relaxed);
    }

    void update_status() {
        auto status = can_data_.load(std::memory_order::relaxed);

        *chassis_power_      = uint_to_double(status.chassis_power, 0.0, 500.0);
        *chassis_voltage_    = uint_to_double(status.chassis_voltage, 0.0, 50.0);
        *powermeter_voltage_ = uint_to_double(status.powermeter_voltage, 0.0, 50.0);
        *powermeter_enabled_ = status.enabled;
    }

    uint16_t generate_command() const {
        PowermeterCommand command;

        command.enabled = *powermeter_control_enabled_;

        double power_limit = *powermeter_charge_power_limit_;
        if (std::isnan(power_limit))
            command.power_limit = 0;
        else
            command.power_limit = static_cast<uint8_t>(std::clamp(power_limit, 0.0, 255.0));

        return std::bit_cast<uint16_t>(command);
    }

    uint16_t generate_disable_command() const {
        PowermeterCommand command;

        command.enabled = false;

        double power_limit = *powermeter_charge_power_limit_;

        if (std::isnan(power_limit))
            command.power_limit = 0;
        else
            command.power_limit = static_cast<uint8_t>(std::clamp(power_limit, 0.0, 255.0));

        return std::bit_cast<uint16_t>(command);
    }

    double chassis_power() { return *chassis_power_; }
    double chassis_voltage() { return *chassis_voltage_; }
    double powermeter_voltage() { return *powermeter_voltage_; }
    double powermeter_enabled() { return *powermeter_enabled_; }

private:
    static constexpr double
        uint_to_double(std::unsigned_integral auto value, double min, double max) {
        double span   = max - min;
        double offset = min;
        return (double)value / (double)decltype(value)(-1) * span + offset;
    }

    struct __attribute__((packed, aligned(8))) PowermeterStatus {
        uint16_t chassis_power;
        uint16_t powermeter_voltage;
        uint16_t chassis_voltage;
        uint8_t enabled;
        uint8_t unused;
    };
    std::atomic<PowermeterStatus> can_data_{};
    static_assert(decltype(can_data_)::is_always_lock_free);

    struct __attribute__((packed, aligned(2))) PowermeterCommand {
        uint8_t power_limit;
        bool enabled;
    };

    Component::OutputInterface<double> chassis_power_;
    Component::OutputInterface<double> chassis_voltage_;
    Component::OutputInterface<double> powermeter_voltage_;
    Component::OutputInterface<bool> powermeter_enabled_;

    Component::InputInterface<bool> powermeter_control_enabled_;
    Component::InputInterface<double> powermeter_charge_power_limit_;
};

} // namespace rmcs_core::hardware::device