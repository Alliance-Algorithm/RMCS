#pragma once

#include <cmath>

#include <algorithm>

#include <rmcs_executor/component.hpp>

#include "hardware/cboard/dji_motor_status.hpp"
#include "hardware/cboard/package.hpp"

namespace rmcs_core::hardware::cboard {

class DjiMotorCommand {
public:
    DjiMotorCommand(rmcs_executor::Component* component, const std::string& name_prefix) {
        component->register_input(name_prefix + "/motor", motor_);
        component->register_input(name_prefix + "/control_torque", control_torque_);
    }
    DjiMotorCommand(const DjiMotorCommand&)            = delete;
    DjiMotorCommand& operator=(const DjiMotorCommand&) = delete;

    void write_command_to_package(Package& package, size_t index) {
        auto& dynamic_part = package.dynamic_part<PackageDjiMotorControlPart>();

        double torque = *control_torque_;
        if (std::isnan(torque)) {
            dynamic_part.current[index] = 0;
            return;
        }
        double max_torque = (*motor_)->get_max_torque();
        torque            = std::clamp(torque, -max_torque, max_torque);

        double current = std::round((*motor_)->torque_to_raw_current_coefficient_ * torque);
        dynamic_part.current[index] = static_cast<short>(current);
    }

private:
    rmcs_executor::Component::InputInterface<DjiMotorStatus*> motor_;

    rmcs_executor::Component::InputInterface<double> control_torque_;
};

} // namespace rmcs_core::hardware::cboard