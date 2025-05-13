#pragma once

#include <keyboard.hpp>
#include <librmcs/device/lk_motor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {

class LkMotor : public librmcs::device::LkMotor {
public:
    enum class Mode : uint8_t { Velocity, Angle, Unknown };

    LkMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix)
        : librmcs::device::LkMotor() {
        status_component.register_output(name_prefix + "/angle", angle_, 0.0);
        status_component.register_output(name_prefix + "/velocity", velocity_, 0.0);
        status_component.register_output(name_prefix + "/torque", torque_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_, 0.0);

        command_component.register_input(name_prefix + "/mode", mode_, false);
        command_component.register_input(
            name_prefix + "/control_velocity", control_velocity_, false);
        command_component.register_input(
            name_prefix + "/control_angle_error", control_angle_, false);
        command_component.register_input(name_prefix + "/velocity_limit", velocity_limit_, false);
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

    Mode mode() const {
        if (mode_.ready()) [[likely]]
            return *mode_;
        else
            return Mode::Unknown;
    }

    double velocity_limit() const {
        if (velocity_limit_.ready()) [[likely]]
            return *velocity_limit_;
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

    uint64_t generate_command() {
        if (mode_.ready()) [[likely]] {
            uint64_t command{std::numeric_limits<uint64_t>::quiet_NaN()};
            switch (*mode_) {
            case Mode::Velocity: {
                command = librmcs::device::LkMotor::generate_velocity_command(control_velocity());
                break;
            }
            case Mode::Angle: {
                if (velocity_limit_.ready())
                    command = librmcs::device::LkMotor::generate_angle_command(
                        control_angle(), *velocity_limit_);
                else
                    command = librmcs::device::LkMotor::generate_angle_command(control_angle());
                break;
            }
            case Mode::Unknown: {
                return command;
            }
                return command;
            }
        }
        return 0;
    }

private:
    rmcs_executor::Component::OutputInterface<double> angle_;
    rmcs_executor::Component::OutputInterface<double> velocity_;
    rmcs_executor::Component::OutputInterface<double> torque_;
    rmcs_executor::Component::OutputInterface<double> max_torque_;

    rmcs_executor::Component::InputInterface<Mode> mode_;
    rmcs_executor::Component::InputInterface<double> control_velocity_;
    rmcs_executor::Component::InputInterface<double> control_angle_;
    rmcs_executor::Component::InputInterface<double> velocity_limit_;
};

} // namespace rmcs_core::hardware::device