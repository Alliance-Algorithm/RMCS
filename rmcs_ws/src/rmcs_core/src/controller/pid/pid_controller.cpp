#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

#include "pid_calculator.hpp"

namespace rmcs_core::controller::pid {

class PidController : public rmcs_executor::Component, public rclcpp::Node {
public:
  PidController()
      : Node(get_component_name(),
             rclcpp::NodeOptions{}
                 .automatically_declare_parameters_from_overrides(true)),
        pid_calculator_(get_parameter("kp").as_double(),
                        get_parameter("ki").as_double(),
                        get_parameter("kd").as_double()),
        logger_(get_logger()) {
    RCLCPP_INFO(get_logger(), "chassis_controller init start");

    register_input(get_parameter("measurement").as_string(), measurement_);

    // Allows using immediate value instead of message name
    auto parameter_setpoint = get_parameter("setpoint");
    if (parameter_setpoint.get_type() ==
        rclcpp::ParameterType::PARAMETER_DOUBLE) {
      setpoint_immediate_value_ = parameter_setpoint.as_double();
      setpoint_.bind_directly(setpoint_immediate_value_);
    } else {
      register_input(parameter_setpoint.as_string(), setpoint_);
    }
    register_output(get_parameter("control").as_string(), control_);

    get_parameter("integral_min", pid_calculator_.integral_min);
    get_parameter("integral_max", pid_calculator_.integral_max);
    get_parameter("output_min", pid_calculator_.output_min);
    get_parameter("output_max", pid_calculator_.output_max);
  }

  void update() override {
    auto err = *setpoint_ - *measurement_;
    *control_ = pid_calculator_.update(err);
    // if (get_component_name() == "left_front_wheel_split_pid_controller" ||
    // get_component_name() == "left_back_wheel_split_pid_controller") {
    //     RCLCPP_INFO(logger_, "%f %f %f %f %f %f", pid_calculator_.kp,
    //     pid_calculator_.ki, pid_calculator_.kd, *setpoint_, *measurement_,
    //     *control_);
    // }
  }

private:
  PidCalculator pid_calculator_;

  InputInterface<double> measurement_;
  InputInterface<double> setpoint_;
  double setpoint_immediate_value_;

  rclcpp::Logger logger_;

  OutputInterface<double> control_;
};

} // namespace rmcs_core::controller::pid

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::pid::PidController,
                       rmcs_executor::Component)
