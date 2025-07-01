#pragma once

#include <cmath>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {
using rmcs_executor::Component;

class Supercap {
public:
    explicit Supercap(Component& status_component, Component& command_component) {
        status_component.register_output("/chassis/power", chassis_power_, 0.0);
        status_component.register_output("/chassis/voltage", chassis_voltage_, 0.0);
        status_component.register_output("/chassis/supercap/voltage", supercap_voltage_, 0.0);
        status_component.register_output("/chassis/supercap/enabled", supercap_enabled_, false);

        command_component.register_input("/referee/chassis/output_status", chassis_output_status_);
        command_component.register_input(
            "/chassis/supercap/charge_power_limit", supercap_charge_power_limit_);
    }

    void store_status(uint64_t can_data) {
        can_data_.store(std::bit_cast<SupercapStatus>(can_data), std::memory_order::relaxed);
    }

    void update_status() {
        auto status = can_data_.load(std::memory_order::relaxed);

        *chassis_power_    = uint_to_double(status.chassis_power, -100.0, 400.0);
        *chassis_voltage_  = uint_to_double(status.chassis_voltage, 0.0, 50.0);
        *supercap_voltage_ = uint_to_double(status.supercap_voltage, 0.0, 50.0);
        *supercap_enabled_ = status.enabled;
    }

    uint16_t generate_command() const {
        SupercapCommand command;

        command.enabled = *chassis_output_status_;

        double power_limit = *supercap_charge_power_limit_;
        if (std::isnan(power_limit))
            command.power_limit = 0;
        else
            command.power_limit = static_cast<uint8_t>(std::clamp(power_limit, 0.0, 255.0));

        return std::bit_cast<uint16_t>(command);
    }

    uint16_t generate_disable_command() const {
        SupercapCommand command;

        command.enabled    = false;
        double power_limit = *supercap_charge_power_limit_;
        if (std::isnan(power_limit))
            command.power_limit = 0;
        else
            command.power_limit = static_cast<uint8_t>(std::clamp(power_limit, 0.0, 255.0));

        return std::bit_cast<uint16_t>(command);
    }

    double chassis_power() { return *chassis_power_; }
    double chassis_voltage() { return *chassis_voltage_; }
    double supercap_voltage() { return *supercap_voltage_; }
    double supercap_enabled() { return *supercap_enabled_; }

private:
    static constexpr double
        uint_to_double(std::unsigned_integral auto value, double min, double max) {
        double span   = max - min;
        double offset = min;
        return (double)value / (double)decltype(value)(-1) * span + offset;
    }

    struct __attribute__((packed, aligned(8))) SupercapStatus {
        uint16_t chassis_power;
        uint16_t supercap_voltage;
        uint16_t chassis_voltage;
        uint8_t enabled;
        uint8_t unused;
    };
    std::atomic<SupercapStatus> can_data_{};
    static_assert(decltype(can_data_)::is_always_lock_free);

    struct __attribute__((packed, aligned(2))) SupercapCommand {
        uint8_t power_limit;
        bool enabled;
    };

    Component::OutputInterface<double> chassis_power_;
    Component::OutputInterface<double> chassis_voltage_;
    Component::OutputInterface<double> supercap_voltage_;
    Component::OutputInterface<bool> supercap_enabled_;

    Component::InputInterface<bool> chassis_output_status_;
    Component::InputInterface<double> supercap_charge_power_limit_;
};

} // namespace rmcs_core::hardware::device