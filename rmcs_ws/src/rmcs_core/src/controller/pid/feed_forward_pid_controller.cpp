#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

#include "pid_calculator.hpp"

namespace rmcs_core::controller::pid {

class FeedForwardPidController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    FeedForwardPidController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , pid_calculator_(
              get_parameter("kp").as_double(), get_parameter("ki").as_double(),
              get_parameter("kd").as_double()) {

        register_input(get_parameter("measurement").as_string(), measurement_);

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
        get_parameter("output_min", pid_calculator_.output_min);
        get_parameter("output_max", pid_calculator_.output_max);
    }

    void update() override {
        setpoint = *setpoint_;
        auto err = setpoint - *measurement_;
        // auto feed_forward = -18. * get_differential_feed_forward(setpoint, setpoint_last);

        // RCLCPP_INFO(
        // this->get_logger(),
        // "measurment:%f,setpoint:%f,setpoint_last:%f,feedforward:%f,control:%f", *measurement_,
        // setpoint, setpoint_last, feed_forward, *control_);

        *control_ = pid_calculator_.update(err);
        // *control_     = pid_calculator_.update(err + feed_forward);
        setpoint_last = setpoint;
    }

private:
    static inline double get_differential_feed_forward(double setpoint, double setpoint_last) {
        return (setpoint - setpoint_last);
    }

    PidCalculator pid_calculator_;

    InputInterface<double> measurement_;
    InputInterface<double> setpoint_;

    OutputInterface<double> control_;

    double setpoint, setpoint_last;
};

} // namespace rmcs_core::controller::pid

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::pid::FeedForwardPidController, rmcs_executor::Component)
