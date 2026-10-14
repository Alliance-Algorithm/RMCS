#include <algorithm>
#include <cmath>
#include <limits>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "eso.hpp"
#include "nlesf.hpp"
#include "td.hpp"

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
        if (has_parameter("limit")) {
            register_input(get_parameter("limit").as_string(), limit_);
            use_dynamic_limit_ = true;
        }

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

        // Internal-state debug outputs (for Foxglove / offline analysis): the
        // ESO estimates z1..z3, the NLESF u0 and the applied u, the tracking
        // errors, and the raw measurement/reference. Topic names are derived
        // from the instance name, e.g. /wheelleg_balance_controller/debug/z3.
        const std::string dbg = std::string{"/"} + get_component_name() + "/debug/";
        const double nan = std::numeric_limits<double>::quiet_NaN();
        register_output(dbg + "measurement", debug_meas_, nan);
        register_output(dbg + "reference", debug_ref_, nan);
        register_output(dbg + "e1", debug_e1_, nan);
        register_output(dbg + "e2", debug_e2_, nan);
        register_output(dbg + "z1", debug_z1_, nan);
        register_output(dbg + "z2", debug_z2_, nan);
        register_output(dbg + "z3", debug_z3_, nan);
        register_output(dbg + "u0", debug_u0_, nan);
        register_output(dbg + "u", debug_u_, nan);

        if (has_parameter("feedforward")) {
            register_input(get_parameter("feedforward").as_string(), feedforward_);
            use_feedforward_ = true;
        }

        // Optional run-enable: when wired, the controller is gated off (output
        // disabled, TD/ESO reset) while the input is false. Used by the get-up so
        // the wheel balance does not drive the wheels before they are loaded.
        if (has_parameter("enable")) {
            register_input(get_parameter("enable").as_string(), enable_);
            use_enable_ = true;
        }

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
                get_logger(), "ADRC V2 setpoint/target not found, using error-input mode: "
                              "measurement input is treated as error (setpoint - measurement).");
        }
    }

    void update() override {
        if (use_enable_ && (!enable_.ready() || !*enable_)) {
            // Disabled by the run-enable: command ZERO torque (not NaN), so the
            // downstream mixer/bridge cannot fall back to a velocity servo.
            *control_ = 0.0;
            last_u_ = 0.0;
            td_.reset();
            eso_.reset();
            return;
        }
        if (!refresh_limit_()) {
            disable_output();
            return;
        }

        const double measurement_or_error = *measurement_;
        if (!std::isfinite(measurement_or_error)) {
            disable_output();
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
                disable_output();
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

        // Feedforward is added before the clamp and included in last_u_, so the
        // ESO sees the TOTAL applied input (feedback + feedforward) and only
        // estimates the residual disturbance instead of cancelling the FF.
        double feedforward = 0.0;
        if (use_feedforward_) {
            if (!feedforward_.ready() || !std::isfinite(*feedforward_)) {
                disable_output();
                return;
            }
            feedforward = *feedforward_;
        }

        const double final_u = std::clamp(feedforward + scaled_u, output_min_, output_max_);
        *control_ = final_u;
        last_u_ = final_u;

        *debug_meas_ = measurement;
        *debug_ref_ = reference;
        *debug_e1_ = e1;
        *debug_e2_ = e2;
        *debug_z1_ = eso_out.z1;
        *debug_z2_ = eso_out.z2;
        *debug_z3_ = eso_out.z3;
        *debug_u0_ = nlesf_out.u0;
        *debug_u_ = final_u;

        static int dbg_counter = 0;
        if (get_parameter_or("debug", false) && (dbg_counter++ % 500 == 0))
            RCLCPP_INFO(
                get_logger(), "adrc meas=%.5f ref=%.5f e1=%.5f e2=%.5f z1=%.5f z2=%.5f z3=%.3f "
                              "u0=%.4f u=%.4f ff=%.3f",
                measurement, reference, e1, e2, eso_out.z1, eso_out.z2, eso_out.z3, nlesf_out.u0,
                nlesf_out.u, feedforward);
    }

private:
    bool refresh_limit_() {
        if (!use_dynamic_limit_) {
            return true;
        }

        if (!limit_.ready()) {
            return false;
        }

        const double raw_limit = *limit_;
        if (!std::isfinite(raw_limit)) {
            return false;
        }

        const double effective_limit = std::max(0.0, raw_limit);
        if (std::isfinite(last_limit_) && std::abs(effective_limit - last_limit_) <= 1e-9) {
            return true;
        }

        auto cfg = nlesf_.config();
        cfg.u_min = -effective_limit;
        cfg.u_max = effective_limit;
        nlesf_.set_config(cfg);
        last_limit_ = effective_limit;
        return true;
    }

    void disable_output() {
        *control_ = std::numeric_limits<double>::quiet_NaN();
        last_u_ = 0.0;
        td_.reset();
        eso_.reset();
    }

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

    InputInterface<double> limit_;
    InputInterface<double> feedforward_;

    OutputInterface<double> control_;

    OutputInterface<double> debug_meas_{}, debug_ref_{}, debug_e1_{}, debug_e2_{};
    OutputInterface<double> debug_z1_{}, debug_z2_{}, debug_z3_{}, debug_u0_{}, debug_u_{};

    TD td_;
    ESO eso_;
    NLESF nlesf_;

    bool use_error_input_mode_ = false;
    bool use_dynamic_limit_ = false;
    bool use_feedforward_ = false;
    bool use_enable_ = false;

    InputInterface<bool> enable_;

    double b0_ = 1.0;
    double kt_ = 1.0;
    double last_u_ = 0.0;
    double last_limit_ = std::numeric_limits<double>::quiet_NaN();

    double output_min_ = -std::numeric_limits<double>::infinity();
    double output_max_ = std::numeric_limits<double>::infinity();
};

} // namespace rmcs_core::controller::adrc

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::adrc::AdrcController, rmcs_executor::Component)
