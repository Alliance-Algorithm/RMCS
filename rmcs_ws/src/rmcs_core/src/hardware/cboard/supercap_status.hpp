#pragma once

#include <memory>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/cboard/package.hpp"

namespace rmcs_core::hardware::cboard {
using namespace rmcs_executor;

class SupercapStatus {
public:
    explicit SupercapStatus(Component* component) {
        component->register_output("/chassis/power", chassis_power_, 0.0);
        component->register_output("/chassis/voltage", chassis_voltage_, 0.0);
        component->register_output("/chassis/supercap/voltage", supercap_voltage_, 0.0);
        component->register_output("/chassis/supercap/enabled", supercap_enabled_, false);
    }

    void update(std::unique_ptr<Package> package, rclcpp::Logger& logger) {
        auto& static_part = package->static_part();

        if (package->dynamic_part_size() != sizeof(SupercapFeedbackPart)) {
            RCLCPP_ERROR(
                logger, "Package size does not match (supercap): [0x%02X 0x%02X] (size = %d)",
                static_part.type, static_part.index, static_part.data_size);
            return;
        }

        auto& dynamic_part = package->dynamic_part<SupercapFeedbackPart>();
        *chassis_power_    = uint_to_double(dynamic_part.chassis_power, 0.0, 500.0);
        *chassis_voltage_  = uint_to_double(dynamic_part.chassis_voltage, 0.0, 50.0);
        *supercap_voltage_ = uint_to_double(dynamic_part.supercap_voltage, 0.0, 50.0);
        *supercap_enabled_ = dynamic_part.enabled;
    }

private:
    struct __attribute__((packed)) SupercapFeedbackPart {
        can_id_t can_id;
        uint16_t chassis_power;
        uint16_t supercap_voltage;
        uint16_t chassis_voltage;
        uint8_t enabled;
        uint8_t unused;
    };

    static constexpr double
        uint_to_double(std::unsigned_integral auto value, double min, double max) {
        double span   = max - min;
        double offset = min;
        return (double)value / (double)decltype(value)(-1) * span + offset;
    }

    Component::OutputInterface<double> chassis_power_;
    Component::OutputInterface<double> chassis_voltage_;
    Component::OutputInterface<double> supercap_voltage_;
    Component::OutputInterface<bool> supercap_enabled_;
};

} // namespace rmcs_core::hardware::cboard