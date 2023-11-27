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

    ///// TODO(anyone): read parameters and initialize the hardware
    serialport_ = info_.hardware_parameters["serialport"];

    hw_position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_effort_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
    RMCS_System::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    ///// TODO(anyone): prepare the robot to be ready for read calls and write calls of some
    /// interfaces
    serial_.open(serialport_);

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RMCS_System::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]));
    }
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RMCS_System::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_effort_commands_[i]));
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn
    RMCS_System::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    ///// TODO(anyone): prepare the robot to receive commands

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
    RMCS_System::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    ///// TODO(anyone): prepare the robot to stop receiving commands

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
    RMCS_System::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    // * read robot states
    static uint8_t rx_buf[1024];
    size_t recv_buf_size = serial_.recv(
        serial::SerialPackage::TypeEncode(serial::SerialPackage::PackageType::USB_PKG_CAN, 0x01),
        rx_buf);

    if (recv_buf_size == 12) {
        // * The single motor
        hw_position_states_[0] =
            static_cast<int16_t>((static_cast<int16_t>(rx_buf[9]) << 8) | rx_buf[10]);
        hw_velocity_states_[0] =
            static_cast<int16_t>((static_cast<int16_t>(rx_buf[11]) << 8) | rx_buf[12]);

        RCLCPP_DEBUG(
            rclcpp::get_logger("RMCS_System"), "Recv position: %lf | velocity: %lf",
            hw_position_states_[0], hw_velocity_states_[0]);
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
    RMCS_System::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    // * write robot's commands'
    constexpr size_t trans_buf_size = 12;
    static uint8_t tx_buf[trans_buf_size];

    // '0xAF' 'Type' 'Destnation' 'Index' 'Size' 'Data[0]' ... 'Data[Size-1]' 'CRC'
    memcpy(reinterpret_cast<char*>(tx_buf), "\x00\x02\x00\x00", 4);
    int16_t effort = static_cast<int16_t>(hw_effort_commands_[0]);
    memcpy(reinterpret_cast<char*>(tx_buf + 4), reinterpret_cast<const char*>(&effort), 2);
    memcpy(reinterpret_cast<char*>(tx_buf + 6), "\x00\x00\x00\x00\x00\x00", 6);

    serial_.send(
        serial::SerialPackage::TypeEncode(serial::SerialPackage::PackageType::USB_PKG_CAN, 0x01),
        tx_buf, trans_buf_size);
    RCLCPP_DEBUG(rclcpp::get_logger("RMCS_System"), "Send effort: %d", effort);

    return hardware_interface::return_type::OK;
}

} // namespace rmcs_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rmcs_hardware_interface::RMCS_System, hardware_interface::SystemInterface)
