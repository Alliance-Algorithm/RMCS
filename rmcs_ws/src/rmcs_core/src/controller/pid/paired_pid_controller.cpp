
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::pid {

class PairedPidController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    PairedPidController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        auto parameter_setpoint = get_parameter("setpoint");
        if (parameter_setpoint.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            setpoint_immediate_value_ = parameter_setpoint.as_double();
            setpoint_.bind_directly(setpoint_immediate_value_);
        } else {
            register_input(parameter_setpoint.as_string(), setpoint_);
        }

        sync_compensation_           = get_parameter("sync_compensation").as_double();
        error_integral_compensation_ = get_parameter("error_integral_compensation").as_double();

        register_input(get_parameter("motor1_measurement").as_string(), motor1_measurement_);
        register_input(get_parameter("motor2_measurement").as_string(), motor2_measurement_);

        register_output(get_parameter("motor1_control_error").as_string(), motor1_control_error_);
        register_output(get_parameter("motor2_control_error").as_string(), motor2_control_error_);
    }

    void update() override {
        double motor1_error   = *setpoint_ - *motor1_measurement_;
        double motor2_error   = *setpoint_ - *motor2_measurement_;
        double relative_error = (*motor1_measurement_ - *motor2_measurement_) * sync_compensation_;
        error_integral_ += relative_error * error_integral_compensation_;

        *motor1_control_error_ = motor1_error - relative_error - error_integral_;
        *motor2_control_error_ = motor2_error + relative_error + error_integral_;
    }

private:
    double sync_compensation_;
    double error_integral_compensation_;
    double error_integral_ = 0;
    double setpoint_immediate_value_;

    InputInterface<double> motor1_measurement_;
    InputInterface<double> motor2_measurement_;
    InputInterface<double> setpoint_;

    OutputInterface<double> motor1_control_error_;
    OutputInterface<double> motor2_control_error_;
};
} // namespace rmcs_core::controller::pid

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::pid::PairedPidController, rmcs_executor::Component)