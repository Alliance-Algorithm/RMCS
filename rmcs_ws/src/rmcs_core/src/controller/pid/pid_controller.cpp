#include <cmath>
#include <limits>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/pid_calculator.hpp"
#include "controller/pid/smart_input.hpp"

namespace rmcs_core::controller::pid {

class PidController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    PidController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , measurement_(*this, "measurement")
        , setpoint_(*this, "setpoint")
        , feedforward_(*this, "feedforward", 0.0)
        , output_min_override_(*this, "output_min_override", std::numeric_limits<double>::quiet_NaN())
        , output_max_override_(*this, "output_max_override", std::numeric_limits<double>::quiet_NaN())
        , pid_calculator_(
              get_parameter("kp").as_double(), get_parameter("ki").as_double(),
              get_parameter("kd").as_double()) {

        register_output(get_parameter("control").as_string(), control_);

        get_parameter("integral_min", pid_calculator_.integral_min);
        get_parameter("integral_max", pid_calculator_.integral_max);

        get_parameter("integral_split_min", pid_calculator_.integral_split_min);
        get_parameter("integral_split_max", pid_calculator_.integral_split_max);

        get_parameter("output_min", pid_calculator_.output_min);
        get_parameter("output_max", pid_calculator_.output_max);
        default_output_min_ = pid_calculator_.output_min;
        default_output_max_ = pid_calculator_.output_max;
    }

    void update() override {
        const double output_min_override = *output_min_override_;
        const double output_max_override = *output_max_override_;
        if (std::isfinite(output_min_override) && std::isfinite(output_max_override)
            && output_min_override <= output_max_override) {
            pid_calculator_.output_min = output_min_override;
            pid_calculator_.output_max = output_max_override;
        } else {
            pid_calculator_.output_min = default_output_min_;
            pid_calculator_.output_max = default_output_max_;
        }

        auto err = *setpoint_ - *measurement_;
        *control_ = *feedforward_ + pid_calculator_.update(err);
    }

private:
    SmartInput measurement_, setpoint_, feedforward_, output_min_override_, output_max_override_;

    PidCalculator pid_calculator_;
    double default_output_min_{0.0};
    double default_output_max_{0.0};

    OutputInterface<double> control_;
};

} // namespace rmcs_core::controller::pid

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::pid::PidController, rmcs_executor::Component)
