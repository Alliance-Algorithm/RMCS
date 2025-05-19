#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

#include "pid_calculator.hpp"

namespace rmcs_core::controller::pid {

class PidController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    PidController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , pid_calculator_(
              get_parameter("kp").as_double(), get_parameter("ki").as_double(),
              get_parameter("kd").as_double()) {

        register_input(get_parameter("measurement").as_string(), measurement_);

        filter_alpha_ = this->get_parameter_or<double>("filter_alpha", 1.0);

        filter_alpha_ = std::clamp(filter_alpha_, 0.0, 1.0);

        // Allows using immediate value instead of message name
        auto parameter_setpoint = get_parameter("setpoint");
        if (parameter_setpoint.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            setpoint_.make_and_bind_directly(parameter_setpoint.as_double());
        } else {
            register_input(parameter_setpoint.as_string(), setpoint_);
        }

        register_output(get_parameter("control").as_string(), control_);

        get_parameter("integral_min", pid_calculator_.integral_min);
        get_parameter("integral_max", pid_calculator_.integral_max);
        get_parameter("integral_split_min", pid_calculator_.integral_split_min);
        get_parameter("integral_split_max", pid_calculator_.integral_split_max);
        get_parameter("output_min", pid_calculator_.output_min);
        get_parameter("output_max", pid_calculator_.output_max);

                if (measurement_.ready()) {
            filtered_measurement_ = *measurement_;
        } else {
            filtered_measurement_ = 0.0;
        }
    }

    void update() override {
        double raw = *measurement_;
        filtered_measurement_ = filter_alpha_ * raw
                              + (1.0 - filter_alpha_) * filtered_measurement_;
        double err = *setpoint_ - filtered_measurement_;
        *control_ = pid_calculator_.update(err);
    }

private:
    double filter_alpha_;            // α ∈ [0,1]
    double filtered_measurement_; 
    PidCalculator pid_calculator_;

    InputInterface<double> measurement_;
    InputInterface<double> setpoint_;

    OutputInterface<double> control_;
};

} // namespace rmcs_core::controller::pid

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::pid::PidController, rmcs_executor::Component)
