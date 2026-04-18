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
    DeformableJointController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input(get_parameter("measurement_angle").as_string(), measurement_angle_);
        register_input(get_parameter("measurement_velocity").as_string(), measurement_velocity_);
        register_input(get_parameter("setpoint_angle").as_string(), setpoint_angle_);

        if (has_parameter("setpoint_velocity")) {
            register_input(get_parameter("setpoint_velocity").as_string(), setpoint_velocity_);
        } else {
            setpoint_velocity_.make_and_bind_directly(0.0);
        }

        if (has_parameter("mode_input")) {
            register_input(get_parameter("mode_input").as_string(), suspension_mode_);
        } else {
            suspension_mode_.make_and_bind_directly(false);
        }

        if (has_parameter("suspension_torque")) {
            register_input(get_parameter("suspension_torque").as_string(), suspension_torque_);
        } else {
            suspension_torque_.make_and_bind_directly(0.0);
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

        dt_ = load_parameter_or(*this, "dt", 0.001);
        b0_ = load_parameter_or(*this, "b0", 1.0);
        kt_ = load_parameter_or(*this, "kt", 1.0);
        torque_feedforward_gain_ = load_parameter_or(*this, "torque_feedforward_gain", 0.0);
        suspension_torque_feedforward_gain_ =
            load_parameter_or(*this, "suspension_torque_feedforward_gain", 1.0);

        normal_td_config_.h = load_parameter_or(*this, "td_h", dt_);
        normal_td_config_.r = load_parameter_or(*this, "td_r", 300.0);
        normal_td_config_.max_vel =
            load_parameter_or(*this, "td_max_vel", std::numeric_limits<double>::infinity());
        normal_td_config_.max_acc =
            load_parameter_or(*this, "td_max_acc", std::numeric_limits<double>::infinity());

        normal_eso_config_.h = dt_;
        normal_eso_config_.b0 = b0_;
        normal_eso_config_.w0 = load_parameter_or(*this, "eso_w0", 80.0);
        normal_eso_config_.auto_beta = load_parameter_or(*this, "eso_auto_beta", true);
        normal_eso_config_.beta1 =
            load_parameter_or(*this, "eso_beta1", 3.0 * normal_eso_config_.w0);
        normal_eso_config_.beta2 = load_parameter_or(
            *this, "eso_beta2", 3.0 * normal_eso_config_.w0 * normal_eso_config_.w0);
        normal_eso_config_.beta3 = load_parameter_or(
            *this, "eso_beta3",
            normal_eso_config_.w0 * normal_eso_config_.w0 * normal_eso_config_.w0);
        normal_eso_config_.z3_limit = load_parameter_or(*this, "eso_z3_limit", 1e9);

        normal_nlesf_config_.k1 = load_parameter_or(*this, "k1", 50.0);
        normal_nlesf_config_.k2 = load_parameter_or(*this, "k2", 5.0);
        normal_nlesf_config_.alpha1 = load_parameter_or(*this, "alpha1", 0.75);
        normal_nlesf_config_.alpha2 = load_parameter_or(*this, "alpha2", 1.25);
        normal_nlesf_config_.delta = load_parameter_or(*this, "delta", 0.01);
        normal_nlesf_config_.u_min =
            load_parameter_or(*this, "u_min", -std::numeric_limits<double>::infinity());
        normal_nlesf_config_.u_max =
            load_parameter_or(*this, "u_max", std::numeric_limits<double>::infinity());

        output_min_ =
            load_parameter_or(*this, "output_min", -std::numeric_limits<double>::infinity());
        output_max_ =
            load_parameter_or(*this, "output_max", std::numeric_limits<double>::infinity());
        if (output_min_ > output_max_) {
            std::swap(output_min_, output_max_);
        }

        suspension_td_config_ = normal_td_config_;
        suspension_td_config_.h =
            load_parameter_or(*this, "suspension_td_h", suspension_td_config_.h);
        suspension_td_config_.r =
            load_parameter_or(*this, "suspension_td_r", suspension_td_config_.r);
        suspension_td_config_.max_vel =
            load_parameter_or(*this, "suspension_td_max_vel", suspension_td_config_.max_vel);
        suspension_td_config_.max_acc =
            load_parameter_or(*this, "suspension_td_max_acc", suspension_td_config_.max_acc);

        suspension_eso_config_ = normal_eso_config_;
        suspension_eso_config_.w0 =
            load_parameter_or(*this, "suspension_eso_w0", suspension_eso_config_.w0);
        suspension_eso_config_.auto_beta =
            load_parameter_or(*this, "suspension_eso_auto_beta", suspension_eso_config_.auto_beta);
        suspension_eso_config_.beta1 =
            load_parameter_or(*this, "suspension_eso_beta1", suspension_eso_config_.beta1);
        suspension_eso_config_.beta2 =
            load_parameter_or(*this, "suspension_eso_beta2", suspension_eso_config_.beta2);
        suspension_eso_config_.beta3 =
            load_parameter_or(*this, "suspension_eso_beta3", suspension_eso_config_.beta3);
        suspension_eso_config_.z3_limit =
            load_parameter_or(*this, "suspension_eso_z3_limit", suspension_eso_config_.z3_limit);

        suspension_nlesf_config_ = normal_nlesf_config_;
        suspension_nlesf_config_.k1 =
            load_parameter_or(*this, "suspension_k1", suspension_nlesf_config_.k1);
        suspension_nlesf_config_.k2 =
            load_parameter_or(*this, "suspension_k2", suspension_nlesf_config_.k2);
        suspension_nlesf_config_.alpha1 =
            load_parameter_or(*this, "suspension_alpha1", suspension_nlesf_config_.alpha1);
        suspension_nlesf_config_.alpha2 =
            load_parameter_or(*this, "suspension_alpha2", suspension_nlesf_config_.alpha2);
        suspension_nlesf_config_.delta =
            load_parameter_or(*this, "suspension_delta", suspension_nlesf_config_.delta);
        suspension_nlesf_config_.u_min =
            load_parameter_or(*this, "suspension_u_min", suspension_nlesf_config_.u_min);
        suspension_nlesf_config_.u_max =
            load_parameter_or(*this, "suspension_u_max", suspension_nlesf_config_.u_max);

        suspension_output_min_ = load_parameter_or(*this, "suspension_output_min", output_min_);
        suspension_output_max_ = load_parameter_or(*this, "suspension_output_max", output_max_);
        if (suspension_output_min_ > suspension_output_max_) {
            std::swap(suspension_output_min_, suspension_output_max_);
        }

        td_.set_config(normal_td_config_);
        eso_.set_config(normal_eso_config_);
        nlesf_.set_config(normal_nlesf_config_);
    }

    void update() override {
        const double measurement_angle = *measurement_angle_;
        const double setpoint_angle = *setpoint_angle_;
        if (!std::isfinite(measurement_angle) || !std::isfinite(setpoint_angle)) {
            disable_output_();
            return;
        }

        const bool suspension_mode = *suspension_mode_;
        if (suspension_mode != last_suspension_mode_) {
            switch_adrc_mode_(suspension_mode);
            last_suspension_mode_ = suspension_mode;
        }

        const double torque_feedforward_gain =
            suspension_mode ? suspension_torque_feedforward_gain_ : torque_feedforward_gain_;
        if (torque_feedforward_gain != 0.0 && !std::isfinite(*suspension_torque_)) {
            disable_output_();
            return;
        }

        const auto [output_min, output_max] = effective_output_limits_(suspension_mode);
        if (std::isnan(output_min) || std::isnan(output_max) || output_min > output_max) {
            disable_output_();
            return;
        }

        if (!initialized_) {
            reset_states_(measurement_angle, setpoint_angle);
            initialized_ = true;
        }

        const auto td_out = td_.update(setpoint_angle);
        const auto eso_out = eso_.update(measurement_angle, last_u_);

        const double e1 = td_out.x1 - eso_out.z1;
        const double e2 = td_out.x2 - eso_out.z2;

        double control_torque = kt_ * nlesf_.compute(e1, e2, eso_out.z3, b0_).u;
        if (torque_feedforward_gain != 0.0) {
            control_torque += torque_feedforward_gain * *suspension_torque_;
        }
        control_torque = std::clamp(control_torque, output_min, output_max);
        if (!std::isfinite(control_torque)) {
            disable_output_();
            return;
        }

        *control_torque_ = control_torque;
        last_u_ = control_torque;
        publish_eso_state_(eso_out);
    }

private:
    std::pair<double, double> effective_output_limits_(bool suspension_mode) const {
        double output_min = suspension_mode ? suspension_output_min_ : output_min_;
        double output_max = suspension_mode ? suspension_output_max_ : output_max_;
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

    void switch_adrc_mode_(bool suspension_mode) {
        if (suspension_mode) {
            td_.set_config(suspension_td_config_);
            eso_.set_config(suspension_eso_config_);
            nlesf_.set_config(suspension_nlesf_config_);
            return;
        }

        td_.set_config(normal_td_config_);
        eso_.set_config(normal_eso_config_);
        nlesf_.set_config(normal_nlesf_config_);
    }

    void reset_states_(double measurement_angle, double setpoint_angle) {
        td_.reset(setpoint_angle, 0.0);
        eso_.reset(measurement_angle);
        last_u_ = 0.0;
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
    InputInterface<double> setpoint_velocity_;
    InputInterface<bool> suspension_mode_;
    InputInterface<double> suspension_torque_;
    InputInterface<double> output_limit_;

    OutputInterface<double> control_torque_;
    OutputInterface<double> eso_z2_output_;
    OutputInterface<double> eso_z3_output_;

    rmcs_core::controller::adrc::TD td_;
    rmcs_core::controller::adrc::ESO eso_;
    rmcs_core::controller::adrc::NLESF nlesf_;

    rmcs_core::controller::adrc::TD::Config normal_td_config_;
    rmcs_core::controller::adrc::TD::Config suspension_td_config_;
    rmcs_core::controller::adrc::ESO::Config normal_eso_config_;
    rmcs_core::controller::adrc::ESO::Config suspension_eso_config_;
    rmcs_core::controller::adrc::NLESF::Config normal_nlesf_config_;
    rmcs_core::controller::adrc::NLESF::Config suspension_nlesf_config_;

    double dt_ = 0.001;
    double b0_ = 1.0;
    double kt_ = 1.0;
    double last_u_ = 0.0;
    double output_min_ = -std::numeric_limits<double>::infinity();
    double output_max_ = std::numeric_limits<double>::infinity();
    double suspension_output_min_ = -std::numeric_limits<double>::infinity();
    double suspension_output_max_ = std::numeric_limits<double>::infinity();
    double torque_feedforward_gain_ = 0.0;
    double suspension_torque_feedforward_gain_ = 1.0;
    bool use_dynamic_limit_ = false;
    bool initialized_ = false;
    bool last_suspension_mode_ = false;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::DeformableJointController, rmcs_executor::Component)
