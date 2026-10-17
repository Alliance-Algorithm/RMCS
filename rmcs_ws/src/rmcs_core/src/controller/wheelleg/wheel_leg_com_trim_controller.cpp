#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

namespace rmcs_core::controller::wheelleg {

// Model-free CoM trim: slowly biases the leg angle (`/wheelleg/control/com_trim_alpha`)
// so that the wheel axle ends up under the (UNKNOWN) CoM.
//
// Physics: a stationary stand requires the CoM over the axle (d = 0). With a
// wrong/unknown CoM model the static solve targets the wrong leg angle, so the
// balance loop has to keep a steady wheel torque / lean to hold the robot. That
// steady wheel effort is an observable, model-free proxy for d: integrate it and
// move the leg angle until it vanishes -> the axle is under the CoM.
//
// This is an OUTER, slow loop only: it changes the final hip-angle target, not
// the per-tick reference. It only integrates while the robot is standing still
// (small pitch, small body speed). `enabled` is a true on/off: false outputs 0
// and forgets the learned bias (re-enable starts from 0). On input dropout or
// a standstill-gate trip it FREEZES and holds the learned bias (no jump).
class WheelLegComTrimController
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    using node = rmcs_utility::NodeMixin;

    WheelLegComTrimController()
        : Node{get_component_name(), node::options()} {
        enabled_ = get_parameter_or("enabled", true);
        trim_sign_ = get_parameter_or("trim_sign", 1.0);
        ki_ = get_parameter_or("ki", 0.02);
        lowpass_tau_ = std::max(1e-3, get_parameter_or("lowpass_tau", 0.2));
        deadband_nm_ = get_parameter_or("deadband_nm", 0.1);
        alpha_bias_max_ = get_parameter_or("alpha_bias_max", 0.4);
        pitch_max_ = get_parameter_or("pitch_max_deg", 10.0) * std::numbers::pi / 180.0;
        velocity_max_ = get_parameter_or("velocity_max", 0.5);
        dt_ = std::max(1e-6, get_parameter_or("dt", 0.001));

        // Wheel-torque form of the proxy, from the balance controller (the
        // common-mode command, so no per-wheel sign ambiguity).
        register_input("/wheelleg/control/wheel_torque", wheel_torque_);
        // Standstill gate.
        register_input("/wheelleg/state/pitch", pitch_);
        register_input("/wheelleg/state/velocity", velocity_);

        // Fed to the DRIVE BRIDGE (added to the reduced leg angle), NOT to the
        // trajectory's alpha_bias: the trajectory already feeds the balance and
        // the balance feeds this trim, so routing back into the trajectory would
        // be a circular dependency.
        register_output("/wheelleg/control/com_trim_alpha", alpha_bias_, 0.0);
    }

    void update() override {
        // True on/off: disabled -> output 0 and forget the learned bias, so a
        // later re-enable starts from 0. This is a construction-time parameter
        // (change it in the YAML and relaunch).
        if (!enabled_) {
            u_lpf_ = 0.0;
            bias_ = 0.0;
            *alpha_bias_ = 0.0;
            return;
        }

        // Input dropout: freeze and HOLD the learned bias (do not jump to 0).
        // While the drive is disarmed the balance outputs 0, so the deadband
        // also freezes the integrator naturally.
        const bool inputs_ok = wheel_torque_.ready() && std::isfinite(*wheel_torque_)
                            && pitch_.ready() && std::isfinite(*pitch_)
                            && velocity_.ready() && std::isfinite(*velocity_);
        if (!inputs_ok) {
            *alpha_bias_ = bias_;
            return;
        }

        const double u = *wheel_torque_;
        const double blend = std::min(1.0, dt_ / lowpass_tau_);
        u_lpf_ += (u - u_lpf_) * blend;

        // Only trim near standstill; a large pitch/velocity means a transient
        // (falling, driving, get-up) where the trim must not act.
        if (std::abs(*pitch_) > pitch_max_ || std::abs(*velocity_) > velocity_max_) {
            *alpha_bias_ = bias_;
            return;
        }

        const double e = (std::abs(u_lpf_) < deadband_nm_) ? 0.0 : u_lpf_;
        // Increasing alpha_bias increases the leg angle alpha, which moves the
        // CoM backwards relative to the axle. For a forward CoM (d > 0) the
        // balance loop holds a negative wheel torque, so alpha_bias must grow:
        // alpha_bias += -ki * u. `trim_sign` flips it if the sim/robot disagrees.
        bias_ += -trim_sign_ * ki_ * e * dt_;
        bias_ = std::clamp(bias_, -alpha_bias_max_, alpha_bias_max_);
        *alpha_bias_ = bias_;
    }

private:
    bool enabled_ = true;
    double trim_sign_ = 1.0;
    double ki_ = 0.02;
    double lowpass_tau_ = 0.2;
    double deadband_nm_ = 0.1;
    double alpha_bias_max_ = 0.4;
    double pitch_max_ = 10.0 * std::numbers::pi / 180.0;
    double velocity_max_ = 0.5;
    double dt_ = 0.001;

    double u_lpf_ = 0.0;
    double bias_ = 0.0;

    InputInterface<double> wheel_torque_;
    InputInterface<double> pitch_;
    InputInterface<double> velocity_;

    OutputInterface<double> alpha_bias_;
};

} // namespace rmcs_core::controller::wheelleg

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::wheelleg::WheelLegComTrimController, rmcs_executor::Component)
