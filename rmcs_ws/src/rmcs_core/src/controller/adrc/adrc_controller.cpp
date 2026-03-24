#include <algorithm>
#include <cmath>
#include <limits>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

#include "ESO.hpp"
#include "NLESF.hpp"
#include "TD.hpp"

namespace rmcs_core::controller::adrc {

class AdrcController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AdrcController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , td_(load_td_config())
        , eso_(load_eso_config())
        , nlesf_(load_nlesf_config()) {
        register_input(get_parameter("measurement").as_string(), measurement_);

        if (has_parameter("setpoint")) {
            register_input(get_parameter("setpoint").as_string(), setpoint_);
            use_error_input_mode_ = false;
        } else if (has_parameter("target")) {
            register_input(get_parameter("target").as_string(), setpoint_);
            use_error_input_mode_ = false;
        } else {
            use_error_input_mode_ = true;
        }

        register_output(get_parameter("control").as_string(), control_);

        kt_ = get_parameter_or("kt", 1.0);
        b0_ = get_parameter_or("b0", 1.0);

        output_min_ = get_parameter_or("output_min", -std::numeric_limits<double>::infinity());
        output_max_ = get_parameter_or("output_max", std::numeric_limits<double>::infinity());
        if (output_min_ > output_max_) {
            std::swap(output_min_, output_max_);
        }

        const double init_reference = get_parameter_or("init_reference", 0.0);
        const double init_measurement = get_parameter_or("init_measurement", 0.0);
        td_.reset(init_reference, 0.0);
        eso_.reset(init_measurement);

        if (use_error_input_mode_) {
            RCLCPP_WARN(
                get_logger(),
                "ADRC V2 setpoint/target not found, using error-input mode: measurement input is treated as error (setpoint - measurement)."
            );
        }
    }

    void update() override {
        const double measurement_or_error = *measurement_;
        if (!std::isfinite(measurement_or_error)) {
            return;
        }

        double reference = 0.0;
        double measurement = 0.0;

        if (use_error_input_mode_) {
            reference = 0.0;
            measurement = -measurement_or_error;
        } else {
            reference = *setpoint_;
            if (!std::isfinite(reference)) {
                return;
            }
            measurement = measurement_or_error;
        }

        const auto td_out = td_.update(reference);
        const auto eso_out = eso_.update(measurement, last_u_);

        const double e1 = td_out.x1 - eso_out.z1;
        const double e2 = td_out.x2 - eso_out.z2;

        const auto nlesf_out = nlesf_.compute(e1, e2, eso_out.z3, b0_);
        const double scaled_u = kt_ * nlesf_out.u;

        const double final_u = std::clamp(scaled_u, output_min_, output_max_);
        *control_ = final_u;
        last_u_ = final_u;
    }

private:
    TD::Config load_td_config() {
        TD::Config cfg;
        const double h = get_parameter_or("dt", 0.001);
        cfg.h = get_parameter_or("td_h", h);
        cfg.r = get_parameter_or("td_r", 300.0);
        cfg.max_vel = get_parameter_or("td_max_vel", std::numeric_limits<double>::infinity());
        cfg.max_acc = get_parameter_or("td_max_acc", std::numeric_limits<double>::infinity());
        return cfg;
    }

    ESO::Config load_eso_config() {
        ESO::Config cfg;
        cfg.h = get_parameter_or("dt", 0.001);
        cfg.b0 = get_parameter_or("b0", 1.0);
        cfg.w0 = get_parameter_or("eso_w0", 80.0);
        cfg.auto_beta = get_parameter_or("eso_auto_beta", true);
        cfg.beta1 = get_parameter_or("eso_beta1", 3.0 * cfg.w0);
        cfg.beta2 = get_parameter_or("eso_beta2", 3.0 * cfg.w0 * cfg.w0);
        cfg.beta3 = get_parameter_or("eso_beta3", cfg.w0 * cfg.w0 * cfg.w0);
        cfg.z3_limit = get_parameter_or("eso_z3_limit", 1e9);
        return cfg;
    }

    NLESF::Config load_nlesf_config() {
        NLESF::Config cfg;
        cfg.k1 = get_parameter_or("k1", 50.0);
        cfg.k2 = get_parameter_or("k2", 5.0);
        cfg.alpha1 = get_parameter_or("alpha1", 0.75);
        cfg.alpha2 = get_parameter_or("alpha2", 1.25);
        cfg.delta = get_parameter_or("delta", 0.01);
        cfg.u_min = get_parameter_or("u_min", -std::numeric_limits<double>::infinity());
        cfg.u_max = get_parameter_or("u_max", std::numeric_limits<double>::infinity());
        return cfg;
    }

    InputInterface<double> measurement_;
    InputInterface<double> setpoint_;
    OutputInterface<double> control_;

    TD td_;
    ESO eso_;
    NLESF nlesf_;

    bool use_error_input_mode_ = false;

    double b0_ = 1.0;
    double kt_ = 1.0;
    double last_u_ = 0.0;

    double output_min_ = -std::numeric_limits<double>::infinity();
    double output_max_ = std::numeric_limits<double>::infinity();
};

} // namespace rmcs_core::controller::adrc

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::adrc::AdrcController, rmcs_executor::Component)
