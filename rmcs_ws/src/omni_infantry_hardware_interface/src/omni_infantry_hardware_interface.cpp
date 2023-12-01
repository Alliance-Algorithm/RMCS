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
#include "omni_infantry_hardware_interface/omni_infantry_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace omni_infantry_hardware_interface {
hardware_interface::CallbackReturn
    OmniInfantrySystem::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    serialport_ = info_.hardware_parameters["serialport"];
    can_motor_commands_.setCanId(stol(info_.hardware_parameters["can_motor_tx_id"]));
    hw_states_packages::CanMotorRxData::setPositionMax(
        stod(info_.hardware_parameters["can_motor_position_max"]));

    can_motor_states_.resize(info_.joints.size());

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
    OmniInfantrySystem::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    if (!serial_.open(serialport_))
        return CallbackReturn::ERROR;

    serial_.subscribe(hw_states_packages::can1_type_code);
    serial_.subscribe(hw_states_packages::dbus_type_code);

    // try { // Dbus state publisher
    //     auto s_publisher_ =
    //         rclcpp::create_publisher<rmcs_msgs::msg::Dbus>("~/dbus",
    //         rclcpp::SystemDefaultsQoS());
    //     dbus_state_publisher_ = std::make_unique<DbusStatePublisher>(s_publisher_);
    // } catch (const std::exception& e) {
    //     fprintf(
    //         stderr,
    //         "Exception thrown during publisher creation at configure stage with message : %s \n",
    //         e.what());
    //     return CallbackReturn::ERROR;
    // }

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> OmniInfantrySystem::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); ++i) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &can_motor_states_[i].position));
    }

    for (size_t i = 0; i < info_.joints.size(); ++i) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
            &can_motor_states_[i].velocity));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> OmniInfantrySystem::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < 4; ++i) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
            &can_motor_commands_.current[i]));
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn
    OmniInfantrySystem::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    // To clear the system buffer and call all the callback functions
    serial_.update();

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
    OmniInfantrySystem::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    // Nothing TODO

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
    OmniInfantrySystem::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    static uint8_t data_buf[1024];
    static hw_states_packages::UartDbusRxData uart_dbus_states;

    serial_.recv(hw_states_packages::dbus_type_code, data_buf);
    uart_dbus_states.set(data_buf);
    // TODO

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
    OmniInfantrySystem::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    static uint8_t data_buf[1024];

    can_motor_commands_.get(data_buf);
    serial_.send(hw_states_packages::can1_type_code, data_buf, can_motor_commands_.tx_data_size);

    return hardware_interface::return_type::OK;
}

} // namespace omni_infantry_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    omni_infantry_hardware_interface::OmniInfantrySystem, hardware_interface::SystemInterface)
