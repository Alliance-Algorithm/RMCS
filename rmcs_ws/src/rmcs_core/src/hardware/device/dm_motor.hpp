#include <cstdint>
#include <rmcs_executor/component.hpp>

#include "librmcs/device/dm_motor.hpp"

namespace rmcs_core::hardware::device {

class DmMotor : public librmcs::device::DmMotor {
public:
    DmMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix) {
        status_component.register_output(name_prefix + "/angle", angle_, 0.0);
        status_component.register_output(name_prefix + "/velocity", velocity_, 0.0);
        status_component.register_output(name_prefix + "/torque", torque_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_, 0.0);

        command_component.register_input(name_prefix + "/control_torque", control_torque_, false);
    }

    DmMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix, const Config& config)
        : DmMotor(status_component, command_component, name_prefix) {
        configure(config);
    }

    void configure(const Config& config) {
        librmcs::device::DmMotor::configure(config);

        *max_torque_ = max_torque();
    }

    void update_status() {
        librmcs::device::DmMotor::update_status();
        *angle_ = angle();
        *velocity_ = velocity();
        *torque_ = torque();

        motor_state_ = error_state();
    }

    double control_torque() {
        if (control_torque_.ready()) [[likely]]
            return *control_torque_;
        else
            return std::numeric_limits<double>::quiet_NaN();
    }

    constexpr static uint64_t generate_enable_command() {
        return librmcs::device::DmMotor::generate_enable_command();
    }

    constexpr static uint64_t generate_clear_error_command() {
        return librmcs::device::DmMotor::generate_clear_error_command();
    }

    uint64_t generate_torque_command(double control_torque) {
        return librmcs::device::DmMotor::generate_torque_command(control_torque);
    }

    uint64_t generate_disable_command() {
        return librmcs::device::DmMotor::generate_disable_command();
    }

    uint64_t generate_command() {
        if (motor_state_ == 0) {
            return generate_enable_command();
        } else if (motor_state_ == 1) {
            return generate_torque_command(control_torque());
        } else {
            return generate_clear_error_command();
        }
    }

private:
    rmcs_executor::Component::OutputInterface<double> angle_;
    rmcs_executor::Component::OutputInterface<double> velocity_;
    rmcs_executor::Component::OutputInterface<double> torque_;
    rmcs_executor::Component::OutputInterface<double> max_torque_;

    rmcs_executor::Component::InputInterface<double> control_torque_;

    uint8_t motor_state_;
};
} // namespace rmcs_core::hardware::device