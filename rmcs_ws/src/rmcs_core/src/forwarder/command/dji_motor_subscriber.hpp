#pragma once

#include <cmath>

#include <algorithm>

#include <rmcs_executor/component.hpp>

#include "forwarder/package.hpp"

namespace rmcs_core::forwarder {

class DjiMotorSubscriber {
public:
    DjiMotorSubscriber(rmcs_executor::Component* component, const std::string& name_prefix) {
        component->register_input(name_prefix + "/scale", scale_);
        component->register_input(name_prefix + "/offset", offset_);
        component->register_input(name_prefix + "/max_current", max_current_);
        component->register_input(name_prefix + "/control_current", control_current_);
    }
    DjiMotorSubscriber(const DjiMotorSubscriber&)            = delete;
    DjiMotorSubscriber& operator=(const DjiMotorSubscriber&) = delete;

    void write_command_to_package(Package& package, size_t index) {
        auto& dynamic_part = package.dynamic_part<PackageDjiMotorControlPart>();

        double current = *control_current_;
        if (!std::isfinite(current)) {
            dynamic_part.current[index] = 0;
            return;
        }

        current = current / *scale_;
        current = std::clamp(current / *max_current_, -1.0, 1.0) * 16384.0;

        dynamic_part.current[index] = static_cast<short>(std::round(current));
    }

private:
    rmcs_executor::Component::InputInterface<double> scale_;

    rmcs_executor::Component::InputInterface<double> offset_;
    rmcs_executor::Component::InputInterface<double> max_current_;

    rmcs_executor::Component::InputInterface<double> control_current_;
};

} // namespace rmcs_core::forwarder