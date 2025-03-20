#pragma once

#include <bit>
#include <bitset>
#include <cstdint>
#include <librmcs/device/lk_motor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {

class LkMotor
    : public librmcs::device::LkMotor
    , rclcpp::Node {
public:
    LkMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix)
        : librmcs::device::LkMotor()
        , Node("LkMotor") {
        status_component.register_output(name_prefix + "/angle", angle_, 0.0);
        status_component.register_output(name_prefix + "/velocity", velocity_, 0.0);
        status_component.register_output(name_prefix + "/torque", torque_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_, 0.0);

        // command_component.register_input(name_prefix + "/control_command", is_enable_, false);
        command_component.register_input(
            name_prefix + "/control_velocity", control_velocity_, false);
    }

    LkMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix, const Config& config)
        : LkMotor(status_component, command_component, name_prefix) {
        configure(config);
    }

    void configure(const Config& config) {
        librmcs::device::LkMotor::configure(config);

        *max_torque_ = max_torque();
    }

    void update_status() {
        librmcs::device::LkMotor::update_status();
        *angle_    = angle();
        *velocity_ = velocity();
        *torque_   = torque();

        // command_id_ = command_id();

        // can_data_ = can_data();
    }

    double control_velocity() const {
        if (control_velocity_.ready()) [[likely]]
            return *control_velocity_;
        else
            return std::numeric_limits<double>::quiet_NaN();
    }

    uint64_t generate_command() {
        uint64_t control_command;
        // bool is_enable = *is_enable_;

        control_command = librmcs::device::LkMotor::generate_velocity_command(control_velocity());

        // if (!is_enable && last_is_enable_) {
        //     control_command = librmcs::device::LkMotor::generate_pause_command();
        //     last_is_enable_ = is_enable;
        // } else if (is_enable && !last_is_enable_) {
        //     control_command = librmcs::device::LkMotor::generate_startup_command();
        //     last_is_enable_ = is_enable;
        // }

        // control_command = librmcs::device::LkMotor::generate_status_request();

        // struct alignas(8) feedback {
        //     uint8_t id;
        //     uint8_t placeholder0[5]{};
        //     uint8_t motor_state;
        //     uint8_t error_state;
        // } command{};

        // command = std::bit_cast<feedback>(can_data_);
        // RCLCPP_INFO(
        //     get_logger(), "control command:%x,feedback command:%x,error_state: %s",
        //     std::bit_cast<feedback>(control_command).id, command_id_,
        //     std::bitset<8>(command.error_state).to_string().c_str());

        return control_command;
    }

private:
    rmcs_executor::Component::OutputInterface<double> angle_;
    rmcs_executor::Component::OutputInterface<double> velocity_;
    rmcs_executor::Component::OutputInterface<double> torque_;
    rmcs_executor::Component::OutputInterface<double> max_torque_;

    rmcs_executor::Component::InputInterface<double> control_velocity_;
    rmcs_executor::Component::InputInterface<bool> is_enable_;

    // uint8_t command_id_;
    // bool last_is_enable_ = false;

    // uint64_t can_data_;
};

} // namespace rmcs_core::hardware::device