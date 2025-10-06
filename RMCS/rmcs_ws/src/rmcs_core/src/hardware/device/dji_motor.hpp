#pragma once

#include <librmcs/device/dji_motor.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {

class DjiMotor : public librmcs::device::DjiMotor {
public:
    DjiMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix)
        : librmcs::device::DjiMotor() {
        status_component.register_output(name_prefix + "/angle", angle_, 0.0);
        status_component.register_output(name_prefix + "/velocity", velocity_, 0.0);
        status_component.register_output(name_prefix + "/torque", torque_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_, 0.0);

        command_component.register_input(name_prefix + "/control_torque", control_torque_, false);
    }

    DjiMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix, const Config& config)
        : DjiMotor(status_component, command_component, name_prefix) {
        configure(config);
    }

    void configure(const Config& config) {
        librmcs::device::DjiMotor::configure(config);

        *max_torque_ = max_torque();
    }

    void update_status() {
        librmcs::device::DjiMotor::update_status();
        *angle_    = angle();
        *velocity_ = velocity();
        *torque_   = torque();
    }

    double control_torque() const {
        if (control_torque_.ready()) [[likely]]
            return *control_torque_;
        else
            return 0.0;
    }

    uint16_t generate_command() {
        return librmcs::device::DjiMotor::generate_command(control_torque());
    }

private:
    rmcs_executor::Component::OutputInterface<double> angle_;
    rmcs_executor::Component::OutputInterface<double> velocity_;
    rmcs_executor::Component::OutputInterface<double> torque_;
    rmcs_executor::Component::OutputInterface<double> max_torque_;

    rmcs_executor::Component::InputInterface<double> control_torque_;
};

} // namespace rmcs_core::hardware::device