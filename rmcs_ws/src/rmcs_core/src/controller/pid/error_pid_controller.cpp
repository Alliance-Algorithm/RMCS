#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

#include "pid_calculator.hpp"

namespace rmcs_core::controller::pid {

class ErrorPidController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ErrorPidController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , pid_calculator_(
              get_parameter("kp").as_double(), get_parameter("ki").as_double(),
              get_parameter("kd").as_double()) {

        register_input(get_parameter("measurement").as_string(), measurement_);
        register_output(get_parameter("control").as_string(), control_);

        get_parameter("integral_min", pid_calculator_.integral_min);
        get_parameter("integral_max", pid_calculator_.integral_max);
        get_parameter("output_min", pid_calculator_.output_min);
        get_parameter("output_max", pid_calculator_.output_max);
    }

    void update() override {
        auto err  = *measurement_;
        *control_ = pid_calculator_.update(err);
    }

private:
    PidCalculator pid_calculator_;

    InputInterface<double> measurement_;

    OutputInterface<double> control_;
};

} // namespace rmcs_core::controller::pid

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::pid::ErrorPidController, rmcs_executor::Component)