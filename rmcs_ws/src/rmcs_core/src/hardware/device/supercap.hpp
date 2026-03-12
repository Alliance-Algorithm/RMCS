#pragma once

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"

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

    void store_status(std::span<const std::byte> can_data) {
        if (can_data.size() != 8) [[unlikely]]
            return;

        can_data_.store(CanPacket8{can_data}, std::memory_order_relaxed);
    }

    void update_status() {
        auto status = std::bit_cast<SupercapStatus>(can_data_.load(std::memory_order::relaxed));

        *chassis_power_ = uint_to_double(status.chassis_power, -100.0, 400.0);
        *chassis_voltage_ = uint_to_double(status.chassis_voltage, 0.0, 50.0);
        *supercap_voltage_ = uint_to_double(status.supercap_voltage, 0.0, 50.0);
        *supercap_enabled_ = status.enabled;
    }

    CanPacket8::Quarter generate_command() const {
        SupercapCommand command;

        command.enabled = *chassis_output_status_;

        const double power_limit = *supercap_charge_power_limit_;
        if (std::isnan(power_limit))
            command.power_limit = 0;
        else
            command.power_limit = static_cast<uint8_t>(std::clamp(power_limit, 0.0, 255.0));

        return std::bit_cast<CanPacket8::Quarter>(command);
    }

    CanPacket8::Quarter generate_disable_command() const {
        SupercapCommand command;

        command.enabled = false;
        const double power_limit = *supercap_charge_power_limit_;
        if (std::isnan(power_limit))
            command.power_limit = 0;
        else
            command.power_limit = static_cast<uint8_t>(std::clamp(power_limit, 0.0, 255.0));

        return std::bit_cast<CanPacket8::Quarter>(command);
    }

    double chassis_power() { return *chassis_power_; }
    double chassis_voltage() { return *chassis_voltage_; }
    double supercap_voltage() { return *supercap_voltage_; }
    double supercap_enabled() { return *supercap_enabled_; }

private:
    static constexpr double
        uint_to_double(std::unsigned_integral auto value, double min, double max) {
        const double span = max - min;
        const double offset = min;
        return (static_cast<double>(value)
                / static_cast<double>(std::numeric_limits<decltype(value)>::max()) * span)
             + offset;
    }

    struct __attribute__((packed, aligned(8))) SupercapStatus {
        uint16_t chassis_power;
        uint16_t supercap_voltage;
        uint16_t chassis_voltage;
        uint8_t enabled;
        uint8_t unused;
    };
    std::atomic<CanPacket8> can_data_;
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
