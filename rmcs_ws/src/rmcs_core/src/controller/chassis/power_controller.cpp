#include <cmath>

#include <limits>
#include <memory>
#include <vector>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::chassis {

class PowerController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    PowerController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        auto motor_names = get_parameter("motors").as_string_array();
        motors_.reserve(motor_names.size());
        for (const auto& motor_name : motor_names)
            motors_.push_back(std::make_unique<Motor>(this, motor_name));

        register_input("/chassis/control_power_limit", power_limit_);
    }

    void update() override {
        double power_limit = *power_limit_;
        double k;

        if (std::isnan(power_limit) || power_limit <= 0) {
            k = 0;
        } else {
            double a = 0, b = 0, c = 0;

            for (auto& motor : motors_) {
                auto [ia, ib, ic] = motor->predict_power_formula();
                a += ia;
                b += ib;
                c += ic;
            }

            double power = a + b + c;
            if (power <= power_limit) {
                k = 1;
            } else {
                c -= power_limit;
                k = (-b + std::sqrt(std::pow(b, 2) - 4 * a * c)) / (2 * a);
                if (std::isnan(k))
                    k = 0;
                else
                    k = std::clamp(k, 0.0, 1.0);
            }
        }

        for (auto& motor : motors_) {
            motor->update_control_current(k);
        }
    }

private:
    class Motor {
    public:
        explicit Motor(rmcs_executor::Component* component, const std::string& name) {
            component->register_input(name + "/reduction_ratio", reduction_ratio_);
            component->register_input(name + "/max_torque", max_torque_);

            component->register_input(
                name + "/control_torque_unrestricted", control_torque_unrestricted_);
            component->register_input(name + "/velocity", velocity_);

            component->register_output(
                name + "/control_torque", control_torque_,
                std::numeric_limits<double>::quiet_NaN());
        }

        std::tuple<double, double, double> predict_power_formula() {
            std::tuple<double, double, double> formula;
            auto& [a, b, c] = formula;

            double torque = *control_torque_unrestricted_; // Unit: N*m
            if (std::isnan(torque))
                torque = 0;
            torque = std::clamp(torque, -*max_torque_, *max_torque_);

            double velocity = *velocity_;                  // Unit: rad/s

            a = k1_ * std::pow(torque, 2); // Quadratic term coefficient: copper loss
            b = torque * velocity;         // linear term coefficient: mechanical power
            c = k2_ * std::pow(velocity, 2)
              + no_load_power_;            // Constant term: iron loss and no-load power

            return formula;                // Unit: W
        }

        void update_control_current(double k) {
            double max_torque = *max_torque_;

            double torque = *control_torque_unrestricted_;
            if (std::isnan(torque))
                torque = 0;
            torque = std::clamp(torque, -max_torque, max_torque);
            torque *= k;

            *control_torque_ = torque;
        }

    private:
        static constexpr double k1_ = 2.958580e+00, k2_ = 3.082190e-03, no_load_power_ = 0.68;

        rmcs_executor::Component::InputInterface<double> reduction_ratio_;
        rmcs_executor::Component::InputInterface<double> max_torque_;

        InputInterface<double> control_torque_unrestricted_;
        InputInterface<double> velocity_;

        OutputInterface<double> control_torque_;
    };

    std::vector<std::unique_ptr<Motor>> motors_;
    InputInterface<double> power_limit_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::PowerController, rmcs_executor::Component)