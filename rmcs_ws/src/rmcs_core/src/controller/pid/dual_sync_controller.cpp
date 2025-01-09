
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

#include "pid_calculator.hpp"

namespace rmcs_core::controller::pid {

class DualSyncController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DualSyncController()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , motor1_pid_calculator_(
              get_parameter("motor1_kp").as_double(), get_parameter("motor1_ki").as_double(),
              get_parameter("motor1_kd").as_double())
        , motor2_pid_calculator_(
              get_parameter("motor2_kp").as_double(), get_parameter("motor2_ki").as_double(),
              get_parameter("motor2_kd").as_double()) {

        register_input(get_parameter("motor1_measurement").as_string(), motor1_measurement_);
        register_input(get_parameter("motor2_measurement").as_string(), motor2_measurement_);
        register_output(get_parameter("motor1_control").as_string(), motor1_control_);
        register_output(get_parameter("motor2_control").as_string(), motor2_control_);

        auto parameter_setpoint = get_parameter("setpoint");
        if (parameter_setpoint.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            setpoint_immediate_value_ = parameter_setpoint.as_double();
            setpoint_.bind_directly(setpoint_immediate_value_);
        } else {
            register_input(parameter_setpoint.as_string(), setpoint_);
        }

        sync_compensation_          = get_parameter("sync_compensation").as_double();
        error_integral_compensation = get_parameter("error_integral_compensation").as_double();

        get_parameter("integral_min", motor1_pid_calculator_.integral_min);
        get_parameter("integral_max", motor1_pid_calculator_.integral_max);
        get_parameter("output_min", motor1_pid_calculator_.output_min);
        get_parameter("output_max", motor1_pid_calculator_.output_max);

        get_parameter("integral_min", motor2_pid_calculator_.integral_min);
        get_parameter("integral_max", motor2_pid_calculator_.integral_max);
        get_parameter("output_min", motor2_pid_calculator_.output_min);
        get_parameter("output_max", motor2_pid_calculator_.output_max);
    }

    void update() override {
        double motor1_error   = *setpoint_ - *motor1_measurement_;
        double motor2_error   = *setpoint_ - *motor2_measurement_;
        double relative_error = *motor1_measurement_ - *motor2_measurement_;

        *motor1_control_ = motor1_pid_calculator_.update(motor1_error - relative_error - error_integral_);
        *motor2_control_ = motor2_pid_calculator_.update(motor2_error + relative_error + error_integral_);
        error_integral_ += relative_error * error_integral_compensation;
        RCLCPP_INFO(
            get_logger(), "v1:%12lf,v2:%12lf,error:%12lf,integral:%12lf", *motor1_measurement_, *motor2_measurement_,
            relative_error, error_integral_);
    }

private:
    PidCalculator motor1_pid_calculator_, motor2_pid_calculator_;

    InputInterface<double> motor1_measurement_;
    InputInterface<double> motor2_measurement_;
    InputInterface<double> setpoint_;

    OutputInterface<double> motor1_control_;
    OutputInterface<double> motor2_control_;

    double setpoint_immediate_value_;
    double sync_compensation_;
    double error_integral_compensation;
    double error_integral_ = 0;
};

} // namespace rmcs_core::controller::pid

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::pid::DualSyncController, rmcs_executor::Component)