#pragma once

#include <limits>

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
        status_component.register_output(name_prefix + "/temperature", temperature_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_, 0.0);

        command_component.register_input( //
            name_prefix + "/control_torque", control_torque_, false);
        command_component.register_input( //
            name_prefix + "/control_velocity", control_velocity_, false);
        command_component.register_input( //
            name_prefix + "/control_angle", control_angle_, false);
        command_component.register_input( //
            name_prefix + "/control_angle_shift", control_angle_shift_, false);
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
        *angle_ = angle();
        *velocity_ = velocity();
        *torque_ = torque();
        *temperature_ = temperature();
    }

    double control_torque() const {
        if (control_torque_.ready()) [[likely]]
            return *control_torque_;
        else
            return std::numeric_limits<double>::quiet_NaN();
    }

    double control_velocity() const {
        if (control_velocity_.ready()) [[likely]]
            return *control_velocity_;
        else
            return std::numeric_limits<double>::quiet_NaN();
    }

    double control_angle() const {
        if (control_angle_.ready()) [[likely]]
            return *control_angle_;
        else
            return std::numeric_limits<double>::quiet_NaN();
    }

    double control_angle_shift() const {
        if (control_angle_shift_.ready()) [[likely]]
            return *control_angle_shift_;
        else
            return std::numeric_limits<double>::quiet_NaN();
    }

    using librmcs::device::LkMotor::generate_torque_command;
    uint64_t generate_torque_command() { return generate_torque_command(control_torque()); }

    using librmcs::device::LkMotor::generate_velocity_command;
    uint64_t generate_velocity_command() { return generate_velocity_command(control_velocity()); }

    using librmcs::device::LkMotor::generate_angle_command;
    uint64_t generate_angle_command() { return generate_angle_command(control_angle()); }

    using librmcs::device::LkMotor::generate_angle_shift_command;
    uint64_t generate_angle_shift_command() {
        return generate_angle_shift_command(control_angle_shift());
    }

    uint64_t generate_command() {
        if (first_generate_auto_command_) [[unlikely]] {
            first_generate_auto_command_ = false;
            if (!control_angle_shift_.ready() && !control_angle_.ready()
                && !control_velocity_.ready() && !control_torque_.ready())
                throw std::runtime_error{"[LkMotor] No manipulating available!"};
            else {
                if (!control_angle_shift_.ready())
                    control_angle_shift_.bind_directly(nan_);
                if (!control_angle_.ready())
                    control_angle_.bind_directly(nan_);
                if (!control_velocity_.ready())
                    control_velocity_.bind_directly(nan_);
                if (!control_torque_.ready())
                    control_torque_.bind_directly(nan_);
            }
        }

        if (!std::isnan(control_angle_shift()))
            return generate_angle_shift_command(control_angle_shift(), control_velocity());
        else if (!std::isnan(control_angle()))
            return generate_angle_command(control_angle(), control_velocity());
        else if (!std::isnan(control_velocity()))
            return generate_velocity_command(control_velocity(), control_torque());
        else
            return generate_torque_command(control_torque());
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    rmcs_executor::Component::OutputInterface<double> angle_;
    rmcs_executor::Component::OutputInterface<double> velocity_;
    rmcs_executor::Component::OutputInterface<double> torque_;
    rmcs_executor::Component::OutputInterface<double> temperature_;
    rmcs_executor::Component::OutputInterface<double> max_torque_;

    rmcs_executor::Component::InputInterface<double> control_torque_;
    rmcs_executor::Component::InputInterface<double> control_velocity_;
    rmcs_executor::Component::InputInterface<double> control_angle_;
    rmcs_executor::Component::InputInterface<double> control_angle_shift_;

    bool first_generate_auto_command_ = true;
};

} // namespace rmcs_core::hardware::device