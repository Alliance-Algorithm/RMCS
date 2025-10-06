#pragma once

#include <fmt/format.h>

#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::pid {
class SmartInput {
public:
    template <typename T>
    requires std::is_base_of_v<rmcs_executor::Component, T> && std::is_base_of_v<rclcpp::Node, T>
    explicit SmartInput(T& component, const std::string& name) {
        if (!component.has_parameter(name))
            throw std::runtime_error(
                fmt::format(
                    "Parameter '{}' was not found in component '{}'", name,
                    component.get_component_name()));

        register_input(component, component.get_parameter(name));
    }

    template <typename T>
    requires std::is_base_of_v<rmcs_executor::Component, T> && std::is_base_of_v<rclcpp::Node, T>
    explicit SmartInput(T& component, const std::string& name, double default_value) {
        if (!component.has_parameter(name)) {
            input_.make_and_bind_directly(default_value);
            return;
        }

        register_input(component, component.get_parameter(name));
    }

    double operator*() const { return is_negative_ ? -*input_ : *input_; }

private:
    void register_input(rmcs_executor::Component& component, const rclcpp::Parameter& value) {
        if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            input_.make_and_bind_directly(value.as_double());
        } else {
            const auto& input_name = value.as_string();
            if (input_name[0] == '-') {
                is_negative_ = true;
                component.register_input(input_name.substr(1), input_);
            } else
                component.register_input(input_name, input_);
        }
    }

    rmcs_executor::Component::InputInterface<double> input_;
    bool is_negative_ = false;
};
} // namespace rmcs_core::controller::pid