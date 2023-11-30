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

#ifndef OMNI_INFANTRY_HARDWARE_INTERFACE__OMNI_INFANTRY_HARDWARE_INTERFACE_HPP_
#define OMNI_INFANTRY_HARDWARE_INTERFACE__OMNI_INFANTRY_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "omni_infantry_hardware_interface/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hw_states_packages/can_motor.hpp"
#include "hw_states_packages/uart_dbus.hpp"
#include "serial_deliver.hpp"

namespace omni_infantry_hardware_interface {
class OmniInfantrySystem : public hardware_interface::SystemInterface {
public:
    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareInfo& info) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& previous_state) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& previous_state) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::return_type
        read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::return_type
        write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    std::string serialport_;
    serial::SerialDeliver serial_;

    hw_states_packages::CanMotorTxData can_motor_commands_;
    std::vector<hw_states_packages::CanMotorRxData> can_motor_states_;
};

} // namespace omni_infantry_hardware_interface

#endif // OMNI_INFANTRY_HARDWARE_INTERFACE__OMNI_INFANTRY_HARDWARE_INTERFACE_HPP_
