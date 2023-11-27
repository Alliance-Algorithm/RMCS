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

#ifndef RMCS_HARDWARE_INTERFACE__RMCS_HARDWARE_INTERFACE_HPP_
#define RMCS_HARDWARE_INTERFACE__RMCS_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rmcs_hardware_interface/visibility_control.h"

#include "serial/serial_deliver.hpp"

namespace rmcs_hardware_interface {
class RMCS_System : public hardware_interface::SystemInterface {
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

    std::vector<double> hw_effort_commands_;
    std::vector<double> hw_position_states_;
    std::vector<double> hw_velocity_states_;
};

} // namespace rmcs_hardware_interface

#endif // RMCS_HARDWARE_INTERFACE__RMCS_HARDWARE_INTERFACE_HPP_
