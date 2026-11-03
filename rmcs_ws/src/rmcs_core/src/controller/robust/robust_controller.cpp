#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/robust/robust_calculator.hpp"

namespace rmcs_core::controller::robust {

class ConfigurableInput {
public:
    ConfigurableInput(
        rmcs_executor::Component& component, rclcpp::Node& node, const std::string& parameter_name,
        double default_value) {
        if (!node.has_parameter(parameter_name)) {
            input_.make_and_bind_directly(default_value);
            return;
        }

        const auto parameter = node.get_parameter(parameter_name);
        if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            input_.make_and_bind_directly(parameter.as_double());
            return;
        }
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
            throw std::invalid_argument(parameter_name + " must be a topic name or a number");

        auto topic = parameter.as_string();
        if (topic.empty())
            throw std::invalid_argument(parameter_name + " topic cannot be empty");
        if (topic.front() == '-') {
            negative_ = true;
            topic.erase(topic.begin());
            if (topic.empty())
                throw std::invalid_argument(parameter_name + " has an empty topic name");
        }
        component.register_input(topic, input_);
    }

    double value() const {
        const auto input_value = *input_;
        return negative_ ? -input_value : input_value;
    }

private:
    rmcs_executor::Component::InputInterface<double> input_;
    bool negative_{false};
};

class RobustController final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    RobustController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , enabled_(false)
        , measurement_(*this, *this, "measurement", 0.0)
        , setpoint_(*this, *this, "setpoint", 0.0)
        , feedforward_(*this, *this, "feedforward", 0.0)
        , calculator_(load_parameters()) {
        get_parameter_or("enabled", enabled_, false);
        if (!has_parameter("control"))
            throw std::invalid_argument("RobustController requires a 'control' output parameter");
        const auto control_name = get_parameter("control").as_string();
        if (control_name.empty())
            throw std::invalid_argument("RobustController output name cannot be empty");
        register_output(control_name, control_, 0.0);
        if (has_parameter("shadow_control")) {
            const auto shadow_name = get_parameter("shadow_control").as_string();
            if (shadow_name.empty() || shadow_name == control_name)
                throw std::invalid_argument("shadow_control must be a distinct, non-empty output name");
            register_output(shadow_name, shadow_control_, 0.0);
            has_shadow_output_ = true;
        }
    }

    void update() override {
        if (!enabled_) {
            calculator_.reset();
            previous_update_ = {};
            *control_ = 0.0;
            if (has_shadow_output_)
                *shadow_control_ = 0.0;
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        const double dt = previous_update_ == std::chrono::steady_clock::time_point{}
            ? 0.0 : std::chrono::duration<double>(now - previous_update_).count();
        previous_update_ = now;
        const double torque = calculator_.update(
            setpoint_.value(), measurement_.value(), feedforward_.value(), dt);
        *control_ = torque;
        if (has_shadow_output_)
            *shadow_control_ = torque;
    }

private:
    RobustVelocityParameters load_parameters() {
        RobustVelocityParameters parameters;
        parameters.inertia = get_parameter_or("inertia", parameters.inertia);
        parameters.viscous_friction = get_parameter_or("viscous_friction", parameters.viscous_friction);
        parameters.coulomb_friction = get_parameter_or("coulomb_friction", parameters.coulomb_friction);
        parameters.static_friction_compensation =
            get_parameter_or("static_friction_compensation", parameters.static_friction_compensation);
        parameters.torque_bias = get_parameter_or("torque_bias", parameters.torque_bias);
        parameters.tracking_gain = get_parameter_or("tracking_gain", parameters.tracking_gain);
        parameters.robust_gain = get_parameter_or("robust_gain", parameters.robust_gain);
        parameters.boundary_width = get_parameter_or("boundary_width", parameters.boundary_width);
        parameters.zero_reference_boundary_width = get_parameter_or(
            "zero_reference_boundary_width", parameters.zero_reference_boundary_width);
        parameters.integral_gain = get_parameter_or("integral_gain_nm_per_rad", parameters.integral_gain);
        parameters.integral_torque_limit =
            get_parameter_or("integral_torque_limit_nm", parameters.integral_torque_limit);
        parameters.friction_velocity_scale =
            get_parameter_or("friction_velocity_scale", parameters.friction_velocity_scale);
        parameters.friction_transition_velocity_scale = get_parameter_or(
            "friction_transition_velocity_scale", parameters.friction_transition_velocity_scale);
        parameters.friction_model = parse_friction_model(
            get_parameter_or("friction_model", std::string{"blend"}));
        parameters.standstill_velocity_threshold =
            get_parameter_or("standstill_velocity_threshold", parameters.standstill_velocity_threshold);
        parameters.output_min = get_parameter_or("output_min", parameters.output_min);
        parameters.output_max = get_parameter_or("output_max", parameters.output_max);
        std::string law{"smooth"};
        law = get_parameter_or("auxiliary_law", law);
        parameters.auxiliary_law = parse_auxiliary_law(law);
        return parameters;
    }

    bool enabled_{false};
    ConfigurableInput measurement_, setpoint_, feedforward_;
    RobustVelocityCalculator calculator_;
    std::chrono::steady_clock::time_point previous_update_{};
    OutputInterface<double> control_;
    OutputInterface<double> shadow_control_;
    bool has_shadow_output_{false};
};

} // namespace rmcs_core::controller::robust

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::robust::RobustController, rmcs_executor::Component)
