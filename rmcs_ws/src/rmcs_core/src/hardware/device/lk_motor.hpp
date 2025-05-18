#pragma once

#include <librmcs/device/lk_motor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {

class LkMotor : public librmcs::device::LkMotor {
public:
    LkMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix)
        : librmcs::device::LkMotor() {
        status_component.register_output(name_prefix + "/angle", angle_, 0.0);
        status_component.register_output(name_prefix + "/velocity", velocity_, 0.0);
        status_component.register_output(name_prefix + "/torque", torque_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_, 0.0);

        command_component.register_input(
            name_prefix + "/control_velocity", control_velocity_, false);
        command_component.register_input(name_prefix + "/control_torque", control_torque_, false);
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

    double control_torque() const {
        if (control_torque_.ready()) [[likely]]
            return *control_torque_;
        else
            return std::numeric_limits<double>::quiet_NaN();
    }

    uint64_t generate_command() {
        return librmcs::device::LkMotor::generate_velocity_command(control_velocity());
    }

private:
    rmcs_executor::Component::OutputInterface<double> angle_;
    rmcs_executor::Component::OutputInterface<double> velocity_;
    rmcs_executor::Component::OutputInterface<double> torque_;
    rmcs_executor::Component::OutputInterface<double> max_torque_;

    rmcs_executor::Component::InputInterface<double> control_velocity_;
    rmcs_executor::Component::InputInterface<double> control_torque_;
};

} // namespace rmcs_core::hardware::device