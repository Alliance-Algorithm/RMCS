#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

namespace rmcs_core::controller::wheelleg {

// Generic up-to-3-state LQR (state feedback) controller. Fully parameterised:
//
//   u = clamp( sum_i gain_i * (input_i - setpoint_i),  u_min, u_max )
//
// No observer, no b0, no nonlinear function, so the ADRC/ESO high-frequency
// limit cycle cannot occur. One instance drives the balance channel (pitch,
// pitch rate, base velocity -> common wheel torque), another the yaw channel
// (yaw, yaw rate -> differential wheel torque). Gains are pure parameters.
class WheelLegLqrController
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegLqrController()
        : Node{get_component_name(), node::options()} {
        static constexpr std::array<const char*, 4> kSuffix{"x", "y", "z", "w"};
        for (std::size_t i = 0; i < 4; ++i) {
            const std::string input =
                node::param_or(std::string{"input_"} + kSuffix[i], std::string{});
            if (input.empty())
                continue;
            register_input(input, input_[i]);
            gain_[i] = node::param_or(std::string{"gain_"} + kSuffix[i], 0.0);
            // Optional per-input sensor noise, std in the input's units. 0 = off.
            noise_std_[i] = node::param_or(std::string{"noise_"} + kSuffix[i], 0.0);
            const std::string setpoint =
                node::param_or(std::string{"setpoint_"} + kSuffix[i], std::string{});
            if (!setpoint.empty()) {
                register_input(setpoint, setpoint_[i]);
                has_setpoint_[i] = true;
            }
            use_[i] = true;
        }

        register_output(
            node::param_or<std::string>("output", std::string{"/wheelleg/control/wheel_torque"}),
            torque_, kNaN);
        register_output(
            std::string{"/"} + get_component_name() + "/debug/u", debug_u_, kNaN);

        enabled_ = node::param_or("enabled", true);
        u_min_ = node::param_or("u_min", -4.88);
        u_max_ = node::param_or("u_max", 4.88);
        const std::string gate = node::param_or("enable", std::string{});
        if (!gate.empty()) {
            register_input(gate, enable_);
            use_enable_ = true;
        }
    }

    void update() override {
        if (!enabled_ || (use_enable_ && (!enable_.ready() || !*enable_))) {
            *torque_ = 0.0;
            *debug_u_ = 0.0;
            return;
        }

        double u = 0.0;
        for (std::size_t i = 0; i < 4; ++i) {
            if (!use_[i])
                continue;
            if (!input_[i].ready() || !std::isfinite(*input_[i])
                || (has_setpoint_[i]
                    && (!setpoint_[i].ready() || !std::isfinite(*setpoint_[i])))) {
                *torque_ = kNaN;
                *debug_u_ = kNaN;
                return;
            }
            double e = *input_[i];
            if (noise_std_[i] > 0.0)
                e += noise_std_[i] * noise_dist_(rng_);
            if (has_setpoint_[i])
                e -= *setpoint_[i];
            u += gain_[i] * e;
        }

        u = std::clamp(u, u_min_, u_max_);
        *torque_ = u;
        *debug_u_ = u;
    }

private:
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

    std::array<InputInterface<double>, 4> input_, setpoint_;
    std::array<bool, 4> use_{}, has_setpoint_{};
    std::array<double, 4> gain_{};
    std::array<double, 4> noise_std_{};
    std::mt19937 rng_{std::random_device{}()};
    std::normal_distribution<double> noise_dist_{0.0, 1.0};

    InputInterface<bool> enable_;
    OutputInterface<double> torque_, debug_u_;

    bool enabled_ = true;
    bool use_enable_ = false;
    double u_min_ = -4.88;
    double u_max_ = 4.88;
};

} // namespace rmcs_core::controller::wheelleg

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::wheelleg::WheelLegLqrController, rmcs_executor::Component)
