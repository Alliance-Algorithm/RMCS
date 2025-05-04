#include <cmath>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/detail/float64__struct.hpp>
#include <std_msgs/msg/float64.hpp>

namespace rmcs_core::controller::chassis {

class PowerController
    : public rmcs_executor::Component
    , public rclcpp::Node {
    using Formula = std::tuple<double, double, double>;

public:
    PowerController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , k1_(get_parameter("k1").as_double())
        , k2_(get_parameter("k2").as_double())
        , no_load_power_(get_parameter("no_load_power").as_double())
        , power_ratio_(get_parameter("power_ratio").as_double()) {

        register_input("/chassis/control_power_limit", power_limit_, false);
        register_input("/chassis/steering/power", steering_power_);
        register_input("/chassis/power", chassis_power_);

        auto motors  = get_parameter("motors").as_string_array();
        motor_count_ = motors.size();

        motor_velocities_ = std::make_unique<InputInterface<double>[]>(motor_count_);
        motor_control_torques_unrestricted_ =
            std::make_unique<InputInterface<double>[]>(motor_count_);
        motor_control_torques_ = std::make_unique<OutputInterface<double>[]>(motor_count_);

        register_input(motors[0] + "/max_torque", wheel_motor_max_control_torque_);

        size_t index = 0;
        for (const auto& motor : motors) {
            register_input(motor + "/velocity", motor_velocities_[index]);
            register_input(
                motor + "/control_torque_unrestricted", motor_control_torques_unrestricted_[index]);

            register_output(motor + "/control_torque", motor_control_torques_[index++], nan_);
        }

        steering_predict_power_publisher_ =
            create_publisher<std_msgs::msg::Float64>("chassis/steering/predict_power", 20);
    }

    void update() override {
        double power_limit = *power_limit_ * power_ratio_;

        double k = 0;
        double a = 0, b = 0, c = 0;
        if (std::isnan(power_limit)) {
            k = 0;
        } else {
            for (size_t index = 0; index < motor_count_; index++) {
                auto [ia, ib, ic] = update_formula(
                    *motor_velocities_[index], *motor_control_torques_unrestricted_[index]);
                a += ia;
                b += ib;
                c += ic;
            }
            auto predict_power = a + b + c;
            if (get_component_name() == "steering_power_controller") {
                std_msgs::msg::Float64 msg;
                msg.data = predict_power;
                steering_predict_power_publisher_->publish(msg);
            }
            if (predict_power < power_limit) {
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

        for (size_t index = 0; index < motor_count_; index++) {
            *motor_control_torques_[index] = k * *motor_control_torques_unrestricted_[index];
        }
    }

private:
    Formula update_formula(double wheel_velocity, double unrestricted_torque) {
        double max_torque = *wheel_motor_max_control_torque_;
        double a = 0, b = 0, c = 0;
        if (std::isnan(unrestricted_torque))
            unrestricted_torque = 0;
        unrestricted_torque = std::clamp(unrestricted_torque, -max_torque, max_torque);

        a = k1_ * std::pow(unrestricted_torque, 2);
        b = wheel_velocity * unrestricted_torque;
        c = k2_ * std::pow(wheel_velocity, 2) + no_load_power_ / static_cast<double>(motor_count_);

        return Formula{a, b, c};
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    size_t motor_count_;

    const double k1_, k2_, no_load_power_;
    const double power_ratio_;

    std::unique_ptr<InputInterface<double>[]> motor_velocities_;
    std::unique_ptr<InputInterface<double>[]> motor_control_torques_unrestricted_;

    InputInterface<double> wheel_motor_max_control_torque_;
    InputInterface<double> steering_power_;
    InputInterface<double> chassis_power_;
    InputInterface<double> power_limit_;

    std::unique_ptr<OutputInterface<double>[]> motor_control_torques_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_predict_power_publisher_;
};
} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::PowerController, rmcs_executor::Component)