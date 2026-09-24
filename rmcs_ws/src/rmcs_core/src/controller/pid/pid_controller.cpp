#include <cstddef>

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
        , pid_calculator_(
              get_parameter("kp").as_double(), get_parameter("ki").as_double(),
              get_parameter("kd").as_double()) {

        register_output(get_parameter("control").as_string(), control_);

        if (has_parameter("reset_interface")) {
            const auto reset_interface = get_parameter("reset_interface").as_string();
            if (!reset_interface.empty()) {
                register_input(reset_interface, reset_count_);
                reset_enabled_ = true;
            }
        }

        get_parameter("integral_min", pid_calculator_.integral_min);
        get_parameter("integral_max", pid_calculator_.integral_max);

        get_parameter("integral_split_min", pid_calculator_.integral_split_min);
        get_parameter("integral_split_max", pid_calculator_.integral_split_max);

        get_parameter("output_min", pid_calculator_.output_min);
        get_parameter("output_max", pid_calculator_.output_max);
    }

    void update() override {
        if (reset_enabled_ && reset_count_.ready() && *reset_count_ != last_reset_count_) {
            last_reset_count_ = *reset_count_;
            pid_calculator_.reset();
            *control_ = 0.0;
            return;
        }

        auto err = *setpoint_ - *measurement_;
        *control_ = *feedforward_ + pid_calculator_.update(err);
    }

private:
    SmartInput measurement_, setpoint_, feedforward_;

    PidCalculator pid_calculator_;

    OutputInterface<double> control_;
    InputInterface<std::size_t> reset_count_;
    std::size_t last_reset_count_ = 0;
    bool reset_enabled_ = false;
};

} // namespace rmcs_core::controller::pid

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::pid::PidController, rmcs_executor::Component)
