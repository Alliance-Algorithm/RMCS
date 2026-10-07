#include <cmath>
#include <cstddef>
#include <limits>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/nn_pid_calculator.hpp"
#include "controller/pid/smart_input.hpp"

namespace rmcs_core::controller::pid {

class NnPidController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    NnPidController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , measurement_(*this, "measurement")
        , setpoint_(*this, "setpoint", 0.0)
        , feedforward_(*this, "feedforward", 0.0)
        , calculator_(read_config()) {
        get_parameter_or("error_input", error_input_, false);
        register_output(get_parameter("control").as_string(), control_, kNaN);
        std::string name;
        get_parameter_or("reset_interface", name, std::string{});
        if (!name.empty()) {
            register_input(name, reset_count_);
            reset_enabled_ = true;
        }
        get_parameter_or("enable_interface", name, std::string{});
        if (!name.empty()) {
            register_input(name, enable_);
            enable_gated_ = true;
        }
        get_parameter_or("gain_monitor_prefix", name, std::string{});
        if (!name.empty()) {
            register_output(name + "/kp", kp_, calculator_.gains()[0]);
            register_output(name + "/ki", ki_, calculator_.gains()[1]);
            register_output(name + "/kd", kd_, calculator_.gains()[2]);
            register_output(name + "/learning_steps", learning_steps_, 0.0);
            register_output(name + "/saturated", saturated_, 0.0);
            monitor_enabled_ = true;
        }
    }

    void update() override {
        const bool reset = reset_enabled_ && *reset_count_ != last_reset_count_;
        if (reset_enabled_)
            last_reset_count_ = *reset_count_;
        if (reset || (enable_gated_ && !*enable_)) {
            if (active_ || reset)
                calculator_.reset();
            active_ = false;
            *control_ = kNaN;
        } else {
            const double error = error_input_ ? *measurement_ : *setpoint_ - *measurement_;
            if (std::isfinite(error) && std::isfinite(*feedforward_)) {
                *control_ = calculator_.update(error, *feedforward_);
                active_ = true;
            } else {
                if (active_)
                    calculator_.reset();
                active_ = false;
                *control_ = kNaN;
            }
        }
        if (monitor_enabled_) {
            *kp_ = calculator_.gains()[0];
            *ki_ = calculator_.gains()[1];
            *kd_ = calculator_.gains()[2];
            *learning_steps_ = static_cast<double>(calculator_.learning_steps());
            *saturated_ = calculator_.saturated() ? 1.0 : 0.0;
        }
    }

private:
    NnPidCalculator::Config read_config() {
        NnPidCalculator::Config config;
        const std::array<std::string, 3> names{"kp", "ki", "kd"};
        for (std::size_t i = 0; i < 3; ++i) {
            config.base[i] = get_parameter(names[i]).as_double();
            get_parameter(names[i] + "_delta_limit", config.delta[i]);
        }
        get_parameter("error_scale", config.input_scale[0]);
        get_parameter("integral_scale", config.input_scale[1]);
        get_parameter("difference_scale", config.input_scale[2]);
        auto positive_integer = [this](const std::string& name, std::size_t fallback) {
            std::int64_t value;
            get_parameter_or(name, value, static_cast<std::int64_t>(fallback));
            if (value <= 0)
                throw std::invalid_argument(name + " must be positive");
            return static_cast<std::size_t>(value);
        };
        config.hidden_size = positive_integer("hidden_size", config.hidden_size);
        config.learning_interval = positive_integer("learning_interval", config.learning_interval);
        config.seed = static_cast<std::uint32_t>(positive_integer("seed", config.seed));
        get_parameter("learning_rate", config.learning_rate);
        get_parameter("plant_sensitivity", config.plant_sensitivity);
        get_parameter("gradient_limit", config.gradient_limit);
        get_parameter("weight_limit", config.weight_limit);
        get_parameter("learning_error_limit", config.learning_error_limit);
        get_parameter("integral_min", config.integral_min);
        get_parameter("integral_max", config.integral_max);
        get_parameter("integral_split_min", config.integral_split_min);
        get_parameter("integral_split_max", config.integral_split_max);
        config.output_min = get_parameter("output_min").as_double();
        config.output_max = get_parameter("output_max").as_double();
        return config;
    }

    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    SmartInput measurement_, setpoint_, feedforward_;
    OutputInterface<double> control_, kp_, ki_, kd_, learning_steps_, saturated_;
    InputInterface<std::size_t> reset_count_;
    InputInterface<bool> enable_;
    NnPidCalculator calculator_;
    std::size_t last_reset_count_ = 0;
    bool error_input_ = false;
    bool reset_enabled_ = false;
    bool enable_gated_ = false;
    bool monitor_enabled_ = false;
    bool active_ = false;
};

} // namespace rmcs_core::controller::pid

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::pid::NnPidController, rmcs_executor::Component)
