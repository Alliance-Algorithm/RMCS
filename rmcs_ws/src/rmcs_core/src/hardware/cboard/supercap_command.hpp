#pragma once

#include <cmath>

#include <bit>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/cboard/package.hpp"

namespace rmcs_core::hardware::cboard {
using namespace rmcs_executor;

class SupercapCommand {
public:
    explicit SupercapCommand(Component* component) {
        component->register_input("/chassis/supercap/control_enable", supercap_control_enabled_);
        component->register_input(
            "/chassis/supercap/charge_power_limit", supercap_charge_power_limit_);
    }

    void write_command_to_package(Package& package, size_t index) const {
        auto& dynamic_part = package.dynamic_part<PackageDjiMotorControlPart>();

        struct __attribute__((packed)) {
            bool enabled;
            uint8_t power_limit;
        } command;

        command.enabled = *supercap_control_enabled_;

        double power_limit = *supercap_charge_power_limit_;
        if (std::isnan(power_limit))
            command.power_limit = 0;
        else
            command.power_limit = static_cast<uint8_t>(std::clamp(power_limit, 0.0, 255.0));

        dynamic_part.current[index] = std::bit_cast<int16_t>(command);
    }

private:
    Component::InputInterface<bool> supercap_control_enabled_;
    Component::InputInterface<double> supercap_charge_power_limit_;
};

} // namespace rmcs_core::hardware::cboard