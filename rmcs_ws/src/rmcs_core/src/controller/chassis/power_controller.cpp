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

        register_input("/referee/robot/chassis_power", chassis_power_referee_);
    }

    void update() override {
        double a = 0, b = 0, c = 0;

        for (auto& motor : motors_) {
            auto [ia, ib, ic] = motor->predict_power_formula();
            a += ia;
            b += ib;
            c += ic;
        }

        double power               = a + b + c;
        constexpr double power_max = 80.0;
        double k;

        if (power <= power_max) {
            k = 1;
        } else {
            c -= power_max;
            k = (-b + std::sqrt(std::pow(b, 2) - 4 * a * c)) / (2 * a);
            if (std::isnan(k))
                k = 0;
            else
                k = std::clamp(k, 0.0, 1.0);
        }

        for (auto& motor : motors_) {
            motor->update_control_current(k);
        }
    }

private:
    class Motor {
    public:
        explicit Motor(rmcs_executor::Component* component, const std::string& name) {
            component->register_input(name + "/scale", scale_);
            component->register_input(
                name + "/control_current_unrestricted", control_current_unrestricted_);
            component->register_input(name + "/velocity", velocity_);

            component->register_output(
                name + "/control_current", control_current_,
                std::numeric_limits<double>::quiet_NaN());
        }

        std::tuple<double, double, double> predict_power_formula() {
            std::tuple<double, double, double> formula;
            auto& [a, b, c] = formula;

            double current = *control_current_unrestricted_; // Unit: A
            if (!std::isfinite(current))
                current = 0;
            current       = current / *scale_;
            current       = std::clamp(current, -20.0, 20.0);
            double torque = (0.3 * 187 / 3591) * current;    // Unit: N * m

            double velocity = *velocity_;                    // Unit: rpm
            velocity        = velocity / *scale_;
            velocity        = velocity * 60 / (2 * std::numbers::pi);

            a = k1_ * std::pow(torque, 2); // Quadratic term coefficient: copper loss
            b = torque * velocity / 9.55;  // linear term coefficient: mechanical power
            c = k2_ * std::pow(velocity, 2)
              + no_load_power_;            // Constant term: iron loss and no-load power

            return formula;                // Unit: W
        }

        void update_control_current(double k) {
            double current = *control_current_unrestricted_;

            if (!std::isfinite(current))
                current = 0;
            else {
                current = current / *scale_;
                current = std::clamp(current, -20.0, 20.0);
                current = current * k;
                current = current * *scale_;
            }

            *control_current_                                   = current;
            const_cast<double&>(*control_current_unrestricted_) = current;
        }

    private:
        double k1_ = 5.0e+02, k2_ = 2.0e-07, no_load_power_ = 0.68;

        InputInterface<double> scale_;
        InputInterface<double> control_current_unrestricted_;
        InputInterface<double> velocity_;

        OutputInterface<double> control_current_;
    };

    std::vector<std::unique_ptr<Motor>> motors_;
    InputInterface<double> chassis_power_referee_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::PowerController, rmcs_executor::Component)