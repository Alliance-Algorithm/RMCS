#pragma once

#include "librmcs/utility/cross_os.hpp"
#include <bit>
#include <cstdint>
#include <librmcs/device/lk_motor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/lkmotor_status.hpp>

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

        command_component.register_input(
            name_prefix + "/control_velocity", control_velocity_, false);

        command_component.register_input(name_prefix + "/motor_status", motor_status_, false);
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
    }

    double control_velocity() const {
        if (control_velocity_.ready()) [[likely]]
            return *control_velocity_;
        else
            return std::numeric_limits<double>::quiet_NaN();
    }

    uint64_t generate_command() {
        auto motor_status = *motor_status_;

        switch (motor_status) {
        case rmcs_msgs::LkmotorStatus::REQUEST: return generate_status_request(); break;
        case rmcs_msgs::LkmotorStatus::START_UP: return generate_startup_command(); break;
        case rmcs_msgs::LkmotorStatus::DISABLE: return generate_pause_command(); break;
        case rmcs_msgs::LkmotorStatus::ENABLE:
            return generate_velocity_command(control_velocity());
            break;
        case rmcs_msgs::LkmotorStatus::UNKNOWN:
        default: return generate_pause_command(); break;
        }
    }

private:
    constexpr static uint64_t generate_pause_command() {
        PACKED_STRUCT({
            uint8_t id = 0x81;
            uint8_t placeholder[7]{};
        } command alignas(8){};)

        return std::bit_cast<uint64_t>(command);
    }

private:
    rmcs_executor::Component::OutputInterface<double> angle_;
    rmcs_executor::Component::OutputInterface<double> velocity_;
    rmcs_executor::Component::OutputInterface<double> torque_;
    rmcs_executor::Component::OutputInterface<double> max_torque_;

    rmcs_executor::Component::InputInterface<rmcs_msgs::LkmotorStatus> motor_status_;
    rmcs_executor::Component::InputInterface<double> control_velocity_;
};

} // namespace rmcs_core::hardware::device