#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/adrc/ESO.hpp"
#include "controller/adrc/NLESF.hpp"
#include "controller/adrc/TD.hpp"

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
    // Joint controller owns only joint-local servo execution. Higher-level deploy/suspension
    // intent is generated upstream by DeformableChassis via setpoint/mode/feedforward inputs.
    struct ModeConfig {
        rmcs_core::controller::adrc::TD::Config td;
        rmcs_core::controller::adrc::ESO::Config eso;
        rmcs_core::controller::adrc::NLESF::Config nlesf;
        double output_min = -std::numeric_limits<double>::infinity();
        double output_max = std::numeric_limits<double>::infinity();
        double torque_feedforward_gain = 0.0;
    };

    struct InputSnapshot {
        double measurement_angle = std::numeric_limits<double>::quiet_NaN();
        double setpoint_angle = std::numeric_limits<double>::quiet_NaN();
        double joint_torque_feedforward = 0.0;
        bool suspension_mode = false;
    };

    DeformableJointController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_interfaces_();
        load_mode_configs_();
        apply_mode_config_(normal_mode_config_);
    }

    void update() override {
        InputSnapshot inputs;
        if (!read_inputs_(inputs)) {
            disable_output_();
            return;
        }

        update_mode_selection_(inputs.suspension_mode);
        const auto& mode_config = active_mode_config_(inputs.suspension_mode);
        const auto [output_min, output_max] = effective_output_limits_(mode_config);
        if (std::isnan(output_min) || std::isnan(output_max) || output_min > output_max) {
            disable_output_();
            return;
        }

        if (!ensure_feedforward_ready_(mode_config, inputs)) {
            disable_output_();
            return;
        }

        initialize_if_needed_(inputs);
        rmcs_core::controller::adrc::ESO::Output eso_out;
        double control_torque = nan_;
        if (!run_joint_servo_(
                inputs, mode_config, output_min, output_max, eso_out, control_torque)) {
            disable_output_();
            return;
        }

        publish_control_output_(control_torque, eso_out);
    }

private:
    void register_interfaces_() {
        register_input(get_parameter("measurement_angle").as_string(), measurement_angle_);
        register_input(get_parameter("measurement_velocity").as_string(), measurement_velocity_);
        register_input(get_parameter("setpoint_angle").as_string(), setpoint_angle_);

        if (has_parameter("setpoint_velocity")) {
            register_input(
                get_parameter("setpoint_velocity").as_string(), setpoint_velocity_input_);
        } else {
            setpoint_velocity_input_.make_and_bind_directly(0.0);
        }

        if (has_parameter("mode_input")) {
            register_input(get_parameter("mode_input").as_string(), suspension_mode_input_);
        } else {
            suspension_mode_input_.make_and_bind_directly(false);
        }

        if (has_parameter("suspension_torque")) {
            register_input(
                get_parameter("suspension_torque").as_string(), joint_torque_feedforward_input_);
        } else {
            joint_torque_feedforward_input_.make_and_bind_directly(0.0);
        }
        if (has_parameter("limit")) {
            register_input(get_parameter("limit").as_string(), output_limit_);
            use_dynamic_limit_ = true;
        }

        register_output(get_parameter("control").as_string(), control_torque_, nan_);
        if (has_parameter("eso_z2_output")) {
            register_output(get_parameter("eso_z2_output").as_string(), eso_z2_output_, nan_);
        }
        if (has_parameter("eso_z3_output")) {
            register_output(get_parameter("eso_z3_output").as_string(), eso_z3_output_, nan_);
        }
    }

    void load_mode_configs_() {
        dt_ = load_parameter_or(*this, "dt", 0.001);
        b0_ = load_parameter_or(*this, "b0", 1.0);
        kt_ = load_parameter_or(*this, "kt", 1.0);

        normal_mode_config_.td.h = load_parameter_or(*this, "td_h", dt_);
        normal_mode_config_.td.r = load_parameter_or(*this, "td_r", 300.0);
        normal_mode_config_.td.max_vel =
            load_parameter_or(*this, "td_max_vel", std::numeric_limits<double>::infinity());
        normal_mode_config_.td.max_acc =
            load_parameter_or(*this, "td_max_acc", std::numeric_limits<double>::infinity());

        normal_mode_config_.eso.h = dt_;
        normal_mode_config_.eso.b0 = b0_;
        normal_mode_config_.eso.w0 = load_parameter_or(*this, "eso_w0", 80.0);
        normal_mode_config_.eso.auto_beta = load_parameter_or(*this, "eso_auto_beta", true);
        normal_mode_config_.eso.beta1 =
            load_parameter_or(*this, "eso_beta1", 3.0 * normal_mode_config_.eso.w0);
        normal_mode_config_.eso.beta2 = load_parameter_or(
            *this, "eso_beta2", 3.0 * normal_mode_config_.eso.w0 * normal_mode_config_.eso.w0);
        normal_mode_config_.eso.beta3 = load_parameter_or(
            *this, "eso_beta3",
            normal_mode_config_.eso.w0 * normal_mode_config_.eso.w0 * normal_mode_config_.eso.w0);
        normal_mode_config_.eso.z3_limit = load_parameter_or(*this, "eso_z3_limit", 1e9);

        normal_mode_config_.nlesf.k1 = load_parameter_or(*this, "k1", 50.0);
        normal_mode_config_.nlesf.k2 = load_parameter_or(*this, "k2", 5.0);
        normal_mode_config_.nlesf.alpha1 = load_parameter_or(*this, "alpha1", 0.75);
        normal_mode_config_.nlesf.alpha2 = load_parameter_or(*this, "alpha2", 1.25);
        normal_mode_config_.nlesf.delta = load_parameter_or(*this, "delta", 0.01);
        normal_mode_config_.nlesf.u_min =
            load_parameter_or(*this, "u_min", -std::numeric_limits<double>::infinity());
        normal_mode_config_.nlesf.u_max =
            load_parameter_or(*this, "u_max", std::numeric_limits<double>::infinity());
        normal_mode_config_.output_min =
            load_parameter_or(*this, "output_min", -std::numeric_limits<double>::infinity());
        normal_mode_config_.output_max =
            load_parameter_or(*this, "output_max", std::numeric_limits<double>::infinity());
        if (normal_mode_config_.output_min > normal_mode_config_.output_max) {
            std::swap(normal_mode_config_.output_min, normal_mode_config_.output_max);
        }
        normal_mode_config_.torque_feedforward_gain =
            load_parameter_or(*this, "torque_feedforward_gain", 0.0);

        suspension_mode_config_ = normal_mode_config_;
        suspension_mode_config_.td.h =
            load_parameter_or(*this, "suspension_td_h", suspension_mode_config_.td.h);
        suspension_mode_config_.td.r =
            load_parameter_or(*this, "suspension_td_r", suspension_mode_config_.td.r);
        suspension_mode_config_.td.max_vel =
            load_parameter_or(*this, "suspension_td_max_vel", suspension_mode_config_.td.max_vel);
        suspension_mode_config_.td.max_acc =
            load_parameter_or(*this, "suspension_td_max_acc", suspension_mode_config_.td.max_acc);
        suspension_mode_config_.eso.w0 =
            load_parameter_or(*this, "suspension_eso_w0", suspension_mode_config_.eso.w0);
        suspension_mode_config_.eso.auto_beta = load_parameter_or(
            *this, "suspension_eso_auto_beta", suspension_mode_config_.eso.auto_beta);
        suspension_mode_config_.eso.beta1 =
            load_parameter_or(*this, "suspension_eso_beta1", suspension_mode_config_.eso.beta1);
        suspension_mode_config_.eso.beta2 =
            load_parameter_or(*this, "suspension_eso_beta2", suspension_mode_config_.eso.beta2);
        suspension_mode_config_.eso.beta3 =
            load_parameter_or(*this, "suspension_eso_beta3", suspension_mode_config_.eso.beta3);
        suspension_mode_config_.eso.z3_limit = load_parameter_or(
            *this, "suspension_eso_z3_limit", suspension_mode_config_.eso.z3_limit);
        suspension_mode_config_.nlesf.k1 =
            load_parameter_or(*this, "suspension_k1", suspension_mode_config_.nlesf.k1);
        suspension_mode_config_.nlesf.k2 =
            load_parameter_or(*this, "suspension_k2", suspension_mode_config_.nlesf.k2);
        suspension_mode_config_.nlesf.alpha1 =
            load_parameter_or(*this, "suspension_alpha1", suspension_mode_config_.nlesf.alpha1);
        suspension_mode_config_.nlesf.alpha2 =
            load_parameter_or(*this, "suspension_alpha2", suspension_mode_config_.nlesf.alpha2);
        suspension_mode_config_.nlesf.delta =
            load_parameter_or(*this, "suspension_delta", suspension_mode_config_.nlesf.delta);
        suspension_mode_config_.nlesf.u_min =
            load_parameter_or(*this, "suspension_u_min", suspension_mode_config_.nlesf.u_min);
        suspension_mode_config_.nlesf.u_max =
            load_parameter_or(*this, "suspension_u_max", suspension_mode_config_.nlesf.u_max);
        suspension_mode_config_.output_min =
            load_parameter_or(*this, "suspension_output_min", normal_mode_config_.output_min);
        suspension_mode_config_.output_max =
            load_parameter_or(*this, "suspension_output_max", normal_mode_config_.output_max);
        if (suspension_mode_config_.output_min > suspension_mode_config_.output_max) {
            std::swap(suspension_mode_config_.output_min, suspension_mode_config_.output_max);
        }
        suspension_mode_config_.torque_feedforward_gain =
            load_parameter_or(*this, "suspension_torque_feedforward_gain", 1.0);
    }

    bool read_inputs_(InputSnapshot& inputs) const {
        inputs.measurement_angle = *measurement_angle_;
        inputs.setpoint_angle = *setpoint_angle_;
        inputs.joint_torque_feedforward = *joint_torque_feedforward_input_;
        inputs.suspension_mode = *suspension_mode_input_;
        return std::isfinite(inputs.measurement_angle) && std::isfinite(inputs.setpoint_angle);
    }

    void update_mode_selection_(bool suspension_mode) {
        if (suspension_mode != last_suspension_mode_) {
            apply_mode_config_(active_mode_config_(suspension_mode));
            last_suspension_mode_ = suspension_mode;
        }
    }

    const ModeConfig& active_mode_config_(bool suspension_mode) const {
        return suspension_mode ? suspension_mode_config_ : normal_mode_config_;
    }

    void apply_mode_config_(const ModeConfig& mode_config) {
        td_.set_config(mode_config.td);
        eso_.set_config(mode_config.eso);
        nlesf_.set_config(mode_config.nlesf);
    }

    std::pair<double, double> effective_output_limits_(const ModeConfig& mode_config) const {
        double output_min = mode_config.output_min;
        double output_max = mode_config.output_max;
        if (use_dynamic_limit_) {
            if (!output_limit_.ready()) {
                return {nan_, nan_};
            }
            const double limit = *output_limit_;
            if (!std::isfinite(limit)) {
                return {nan_, nan_};
            }

            const double effective_limit = std::max(0.0, limit);
            output_min = std::max(output_min, -effective_limit);
            output_max = std::min(output_max, effective_limit);
        }
        return {output_min, output_max};
    }

    bool ensure_feedforward_ready_(
        const ModeConfig& mode_config, const InputSnapshot& inputs) const {
        return mode_config.torque_feedforward_gain == 0.0
            || std::isfinite(inputs.joint_torque_feedforward);
    }

    void initialize_if_needed_(const InputSnapshot& inputs) {
        if (initialized_)
            return;

        reset_states_(inputs.measurement_angle, inputs.setpoint_angle);
        initialized_ = true;
    }

    bool run_joint_servo_(
        const InputSnapshot& inputs, const ModeConfig& mode_config, double output_min,
        double output_max, rmcs_core::controller::adrc::ESO::Output& eso_out,
        double& control_torque) {
        const auto td_out = td_.update(inputs.setpoint_angle);
        eso_out = eso_.update(inputs.measurement_angle, last_u_);

        const double e1 = td_out.x1 - eso_out.z1;
        const double e2 = td_out.x2 - eso_out.z2;

        control_torque = kt_ * nlesf_.compute(e1, e2, eso_out.z3, b0_).u;
        if (mode_config.torque_feedforward_gain != 0.0) {
            control_torque += mode_config.torque_feedforward_gain * inputs.joint_torque_feedforward;
        }
        control_torque = std::clamp(control_torque, output_min, output_max);
        return std::isfinite(control_torque);
    }

    void reset_states_(double measurement_angle, double setpoint_angle) {
        td_.reset(setpoint_angle, 0.0);
        eso_.reset(measurement_angle);
        last_u_ = 0.0;
    }

    void publish_control_output_(
        double control_torque, const rmcs_core::controller::adrc::ESO::Output& eso_out) {
        *control_torque_ = control_torque;
        last_u_ = control_torque;
        publish_eso_state_(eso_out);
    }

    void publish_eso_state_(const rmcs_core::controller::adrc::ESO::Output& eso_out) {
        if (eso_z2_output_.active()) {
            *eso_z2_output_ = eso_out.z2;
        }
        if (eso_z3_output_.active()) {
            *eso_z3_output_ = eso_out.z3;
        }
    }

    void disable_output_() {
        initialized_ = false;
        last_u_ = 0.0;
        *control_torque_ = nan_;
        if (eso_z2_output_.active()) {
            *eso_z2_output_ = nan_;
        }
        if (eso_z3_output_.active()) {
            *eso_z3_output_ = nan_;
        }
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<double> measurement_angle_;
    InputInterface<double> measurement_velocity_;
    InputInterface<double> setpoint_angle_;
    // Kept for graph compatibility; TD derives the actual servo-rate target locally.
    InputInterface<double> setpoint_velocity_input_;
    InputInterface<bool> suspension_mode_input_;
    InputInterface<double> joint_torque_feedforward_input_;
    InputInterface<double> output_limit_;

    OutputInterface<double> control_torque_;
    OutputInterface<double> eso_z2_output_;
    OutputInterface<double> eso_z3_output_;

    rmcs_core::controller::adrc::TD td_;
    rmcs_core::controller::adrc::ESO eso_;
    rmcs_core::controller::adrc::NLESF nlesf_;

    ModeConfig normal_mode_config_;
    ModeConfig suspension_mode_config_;

    double dt_ = 0.001;
    double b0_ = 1.0;
    double kt_ = 1.0;
    double last_u_ = 0.0;
    bool use_dynamic_limit_ = false;
    bool initialized_ = false;
    bool last_suspension_mode_ = false;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::DeformableJointController, rmcs_executor::Component)
