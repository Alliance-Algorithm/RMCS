#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/adrc/eso.hpp"
#include "controller/adrc/nlesf.hpp"
#include "controller/adrc/td.hpp"

namespace rmcs_core::controller::chassis {

namespace {

double load_parameter_or(rclcpp::Node& node, const std::string& name, double fallback) {
    double value = fallback;
    node.get_parameter_or(name, value, fallback);
    return value;
}

bool load_parameter_or(rclcpp::Node& node, const std::string& name, bool fallback) {
    bool value = fallback;
    node.get_parameter_or(name, value, fallback);
    return value;
}

} // namespace

class DeformableJointController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit DeformableJointController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input(get_parameter("measurement_angle").as_string(), measurement_angle_);
        register_input(get_parameter("setpoint_angle").as_string(), setpoint_angle_);
        if (has_parameter("setpoint_velocity")) {
            register_input(
                get_parameter("setpoint_velocity").as_string(), setpoint_velocity_, false);
            use_setpoint_velocity_ = true;
        }
        register_output(get_parameter("control").as_string(), control_torque_, nan_);

        load_config_();
        apply_config_();
    }

    void update() override {
        InputSnapshot inputs;
        if (!read_inputs_(inputs)) {
            disable_output_();
            return;
        }

        initialize_if_needed_(inputs);
        double control_torque = nan_;
        if (!run_joint_servo_(inputs, control_torque)) {
            disable_output_();
            return;
        }

        publish_control_output_(control_torque);
    }

private:
    // Joint controller owns only the local angle-servo execution. Chassis publishes the
    // higher-level target angle trajectory; this controller turns that target into motor torque.
    struct ControllerConfig {
        rmcs_core::controller::adrc::TD::Config td;
        rmcs_core::controller::adrc::ESO::Config eso;
        rmcs_core::controller::adrc::NLESF::Config nlesf;
        double output_min = -std::numeric_limits<double>::infinity();
        double output_max = std::numeric_limits<double>::infinity();
    };

    struct InputSnapshot {
        double measurement_angle = std::numeric_limits<double>::quiet_NaN();
        double setpoint_angle = std::numeric_limits<double>::quiet_NaN();
        double setpoint_velocity = std::numeric_limits<double>::quiet_NaN();
    };

    void load_config_() {
        dt_ = load_parameter_or(*this, "dt", 0.001);
        b0_ = load_parameter_or(*this, "b0", 1.0);
        kt_ = load_parameter_or(*this, "kt", 1.0);

        config_.td.h = load_parameter_or(*this, "td_h", dt_);
        config_.td.r = load_parameter_or(*this, "td_r", 300.0);
        config_.td.max_vel =
            load_parameter_or(*this, "td_max_vel", std::numeric_limits<double>::infinity());
        config_.td.max_acc =
            load_parameter_or(*this, "td_max_acc", std::numeric_limits<double>::infinity());

        config_.eso.h = dt_;
        config_.eso.b0 = b0_;
        config_.eso.w0 = load_parameter_or(*this, "eso_w0", 80.0);
        config_.eso.auto_beta = load_parameter_or(*this, "eso_auto_beta", true);
        config_.eso.beta1 = load_parameter_or(*this, "eso_beta1", 3.0 * config_.eso.w0);
        config_.eso.beta2 =
            load_parameter_or(*this, "eso_beta2", 3.0 * config_.eso.w0 * config_.eso.w0);
        config_.eso.beta3 = load_parameter_or(
            *this, "eso_beta3", config_.eso.w0 * config_.eso.w0 * config_.eso.w0);
        config_.eso.z3_limit = load_parameter_or(*this, "eso_z3_limit", 1e9);

        config_.nlesf.k1 = load_parameter_or(*this, "k1", 50.0);
        config_.nlesf.k2 = load_parameter_or(*this, "k2", 5.0);
        config_.nlesf.alpha1 = load_parameter_or(*this, "alpha1", 0.75);
        config_.nlesf.alpha2 = load_parameter_or(*this, "alpha2", 1.25);
        config_.nlesf.delta = load_parameter_or(*this, "delta", 0.01);
        config_.nlesf.u_min =
            load_parameter_or(*this, "u_min", -std::numeric_limits<double>::infinity());
        config_.nlesf.u_max =
            load_parameter_or(*this, "u_max", std::numeric_limits<double>::infinity());
        config_.output_min =
            load_parameter_or(*this, "output_min", -std::numeric_limits<double>::infinity());
        config_.output_max =
            load_parameter_or(*this, "output_max", std::numeric_limits<double>::infinity());
        if (config_.output_min > config_.output_max) {
            std::swap(config_.output_min, config_.output_max);
        }
    }

    bool read_inputs_(InputSnapshot& inputs) const {
        inputs.measurement_angle = *measurement_angle_;
        inputs.setpoint_angle = *setpoint_angle_;
        if (use_setpoint_velocity_ && setpoint_velocity_.ready()
            && std::isfinite(*setpoint_velocity_)) {
            inputs.setpoint_velocity = *setpoint_velocity_;
        }
        return std::isfinite(inputs.measurement_angle) && std::isfinite(inputs.setpoint_angle);
    }

    void apply_config_() {
        td_.set_config(config_.td);
        eso_.set_config(config_.eso);
        nlesf_.set_config(config_.nlesf);
    }

    void initialize_if_needed_(const InputSnapshot& inputs) {
        if (initialized_)
            return;

        reset_states_(inputs.measurement_angle, inputs.setpoint_angle);
        initialized_ = true;
    }

    bool run_joint_servo_(const InputSnapshot& inputs, double& control_torque) {
        const auto eso_out = eso_.update(inputs.measurement_angle, last_u_);

        double reference_angle = inputs.setpoint_angle;
        double reference_velocity = inputs.setpoint_velocity;
        if (!std::isfinite(reference_velocity)) {
            const auto td_out = td_.update(inputs.setpoint_angle);
            reference_angle = td_out.x1;
            reference_velocity = td_out.x2;
        }

        const double e1 = reference_angle - eso_out.z1;
        const double e2 = reference_velocity - eso_out.z2;

        control_torque = kt_ * nlesf_.compute(e1, e2, eso_out.z3, b0_).u;
        control_torque = std::clamp(control_torque, config_.output_min, config_.output_max);
        return std::isfinite(control_torque);
    }

    void reset_states_(double measurement_angle, double setpoint_angle) {
        td_.reset(setpoint_angle, 0.0);
        eso_.reset(measurement_angle);
        last_u_ = 0.0;
    }

    void publish_control_output_(double control_torque) {
        *control_torque_ = control_torque;
        last_u_ = control_torque;
    }

    void disable_output_() {
        initialized_ = false;
        last_u_ = 0.0;
        *control_torque_ = nan_;
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<double> measurement_angle_;
    InputInterface<double> setpoint_angle_;
    InputInterface<double> setpoint_velocity_;

    OutputInterface<double> control_torque_;

    rmcs_core::controller::adrc::TD td_;
    rmcs_core::controller::adrc::ESO eso_;
    rmcs_core::controller::adrc::NLESF nlesf_;

    ControllerConfig config_;

    double dt_ = 0.001;
    double b0_ = 1.0;
    double kt_ = 1.0;
    double last_u_ = 0.0;
    bool use_setpoint_velocity_ = false;
    bool initialized_ = false;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::DeformableJointController, rmcs_executor::Component)
