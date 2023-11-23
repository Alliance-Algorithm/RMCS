// Copyright (c) 2023, Alliance
// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <limits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmcs_hardware_interface/rmcs_hardware_interface.hpp"

namespace rmcs_hardware_interface {
hardware_interface::CallbackReturn
    RMCS_System::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // TODO(anyone): read parameters and initialize the hardware
    // serialport_ = info_.hardware_parameters["serialport"];

    hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
    RMCS_System::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces
    // serial_.open(serialport_);

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RMCS_System::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            // TODO(anyone): insert correct interfaces
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RMCS_System::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            // TODO(anyone): insert correct interfaces
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn
    RMCS_System::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    // TODO(anyone): prepare the robot to receive commands

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
    RMCS_System::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    // TODO(anyone): prepare the robot to stop receiving commands

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
    RMCS_System::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    // TODO(anyone): read robot states

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
    RMCS_System::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    // TODO(anyone): write robot's commands'

    return hardware_interface::return_type::OK;
}

} // namespace rmcs_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rmcs_hardware_interface::RMCS_System, hardware_interface::SystemInterface)
