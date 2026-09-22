#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::wheelleg {

// LQR balance+velocity controller (plan B) for the wheel-leg.
//
// Linearised reduced plant around the standing equilibrium with the leg held by
// its joint PD:
//   x = [com_x, v_base, pitch, pitch_rate],  u = wheel_torque
// u = -K (x - x_ref).  The gain vector K is designed offline from the MuJoCo
// model (see /tmp/opencode or the model scripts) and passed in via parameters.
class WheelLegLqrController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    WheelLegLqrController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_output(get_parameter("control").as_string(), control_, 0.0);

        register_input(get_parameter("com_x").as_string(), com_x_);
        register_input(get_parameter("velocity").as_string(), velocity_);
        register_input(get_parameter("pitch").as_string(), pitch_);
        register_input(get_parameter("pitch_velocity").as_string(), pitch_velocity_);

        if (has_parameter("setpoint"))
            register_input(get_parameter("setpoint").as_string(), setpoint_);

        kx_ = get_parameter_or("kx", 0.0);
        kv_ = get_parameter_or("kv", 0.0);
        ktheta_ = get_parameter_or("ktheta", 0.0);
        kthetadot_ = get_parameter_or("kthetadot", 0.0);
        ki_ = get_parameter_or("ki", 0.0);
        integral_limit_ = get_parameter_or("integral_limit", 10.0);
        dt_ = get_parameter_or("dt", 0.001);

        feedback_sign_ = get_parameter_or("feedback_sign", 1.0);

        output_min_ = get_parameter_or("output_min", -std::numeric_limits<double>::infinity());
        output_max_ = get_parameter_or("output_max", std::numeric_limits<double>::infinity());
        if (output_min_ > output_max_)
            std::swap(output_min_, output_max_);
    }

    void update() override {
        if (!com_x_.ready() || !velocity_.ready() || !pitch_.ready() || !pitch_velocity_.ready()
            || !std::isfinite(*com_x_) || !std::isfinite(*velocity_) || !std::isfinite(*pitch_)
            || !std::isfinite(*pitch_velocity_)) {
            *control_ = 0.0;
            return;
        }

        const double x_ref =
            (setpoint_.ready() && std::isfinite(*setpoint_)) ? *setpoint_ : 0.0;
        const double ex = *com_x_ - x_ref;

        integral_ = std::clamp(integral_ + dt_ * ex, -integral_limit_, integral_limit_);

        const double u = -(kx_ * ex + kv_ * (*velocity_) + ktheta_ * (*pitch_)
                           + kthetadot_ * (*pitch_velocity_) + ki_ * integral_);
        *control_ = std::clamp(feedback_sign_ * u, output_min_, output_max_);
    }

private:
    InputInterface<double> com_x_;
    InputInterface<double> velocity_;
    InputInterface<double> pitch_;
    InputInterface<double> pitch_velocity_;
    InputInterface<double> setpoint_;

    OutputInterface<double> control_;

    double kx_ = 0.0;
    double kv_ = 0.0;
    double ktheta_ = 0.0;
    double kthetadot_ = 0.0;
    double ki_ = 0.0;
    double integral_limit_ = 10.0;
    double integral_ = 0.0;
    double dt_ = 0.001;
    double feedback_sign_ = 1.0;
    double output_min_ = -std::numeric_limits<double>::infinity();
    double output_max_ = std::numeric_limits<double>::infinity();
};

} // namespace rmcs_core::controller::wheelleg

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::wheelleg::WheelLegLqrController, rmcs_executor::Component)
