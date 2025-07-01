#include "rmcs_utility/eigen_structured_bindings.hpp"
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::chassis {

class PowerControllerTest
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    explicit PowerControllerTest()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , wheel_parameters_(
              {get_parameter("wheel_k1").as_double(),    //
               get_parameter("wheel_k2").as_double(),    //
               get_parameter("wheel_static_power").as_double()})
        , steering_parameters_(
              {get_parameter("steering_k1").as_double(), //
               get_parameter("steering_k2").as_double(), //
               get_parameter("steering_static_power").as_double()})
        , power_ratio_(get_parameter("power_ratio").as_double()) {

        register_input("/chassis/control_power_limit", power_limit_);
        // register_input("/chassis/steering/power", steering_power_);
        // register_input("/chassis/power", chassis_power_);

        register_input("/chassis/left_front_steering/velocity", left_front_steering_velocity_);
        register_input("/chassis/left_back_steering/velocity", left_back_steering_velocity_);
        register_input("/chassis/right_back_steering/velocity", right_back_steering_velocity_);
        register_input("/chassis/right_front_steering/velocity", right_front_steering_velocity_);

        register_input("/chassis/left_front_wheel/velocity", left_front_wheel_velocity_);
        register_input("/chassis/left_back_wheel/velocity", left_back_wheel_velocity_);
        register_input("/chassis/right_back_wheel/velocity", right_back_wheel_velocity_);
        register_input("/chassis/right_front_wheel/velocity", right_front_wheel_velocity_);

        register_input(
            "/chassis/left_front_steering/control_torque_unrestricted",
            left_front_steering_control_torque_unrestricted_);
        register_input(
            "/chassis/left_back_steering/control_torque_unrestricted",
            left_back_steering_control_torque_unrestricted_);
        register_input(
            "/chassis/right_back_steering/control_torque_unrestricted",
            right_back_steering_control_torque_unrestricted_);
        register_input(
            "/chassis/right_front_steering/control_torque_unrestricted",
            right_front_steering_control_torque_unrestricted_);
        register_input(
            "/chassis/left_front_wheel/control_torque_unrestricted",
            left_front_wheel_control_torque_unrestricted_);
        register_input(
            "/chassis/left_back_wheel/control_torque_unrestricted",
            left_back_wheel_control_torque_unrestricted_);
        register_input(
            "/chassis/right_back_wheel/control_torque_unrestricted",
            right_back_wheel_control_torque_unrestricted_);
        register_input(
            "/chassis/right_front_wheel/control_torque_unrestricted",
            right_front_wheel_control_torque_unrestricted_);

        register_output(
            "/chassis/left_front_steering/control_torque", left_front_steering_control_torque_);
        register_output(
            "/chassis/left_back_steering/control_torque", left_back_steering_control_torque_);
        register_output(
            "/chassis/right_back_steering/control_torque", right_back_steering_control_torque_);
        register_output(
            "/chassis/right_front_steering/control_torque", right_front_steering_control_torque_);
        register_output(
            "/chassis/left_front_wheel/control_torque", left_front_wheel_control_torque_);
        register_output("/chassis/left_back_wheel/control_torque", left_back_wheel_control_torque_);
        register_output(
            "/chassis/right_back_wheel/control_torque", right_back_wheel_control_torque_);
        register_output(
            "/chassis/right_front_wheel/control_torque", right_front_wheel_control_torque_);
        register_output("/chassis/power_predicted", max_power_predicted_);

        // register_input("/chassis/left_front_steering/angle", left_front_steering_angle_);
        // register_input("/chassis/left_back_steering/angle", left_back_steering_angle_);
        // register_input("/chassis/right_back_steering/angle", right_back_steering_angle_);
        // register_input("/chassis/right_front_steering/angle", right_front_steering_angle_);
    }

    void update() override {

        auto steering_velocities_unrestricted = calculate_steering_velocities();
        auto steering_formula                 = update_power_formula(
            steering_parameters_, steering_velocities_unrestricted,
            steering_velocities_unrestricted);

        const auto& steering_torques_unrestricted = calculate_steering_torque_unrestricted();
        auto steering_control_torques             = calculate_control_torques(
            *power_limit_ * power_ratio_, steering_formula, steering_torques_unrestricted);
        set_steering_control_torque(steering_control_torques);

        auto wheel_velocities_unrestricted = calculate_wheel_velocities();
        auto wheel_formula                 = update_power_formula(
            wheel_parameters_, wheel_velocities_unrestricted, wheel_velocities_unrestricted);

        const auto& wheel_torques_unrestricted = calculate_wheel_torques_unrestricted();
        auto wheel_control_torques             = calculate_control_torques(
            (1 - *power_limit_) * power_ratio_, wheel_formula, wheel_torques_unrestricted);
        set_wheel_control_torque(wheel_control_torques);
    }

private:
    struct Paremeters {
        double k1;
        double k2;
        double static_power;
    };

    static Eigen::Vector3d update_power_formula(
        Paremeters& parameters, const Eigen::Vector4d& motor_velocities,
        const Eigen::Vector4d& motor_control_torques_unrestricted) {
        Eigen::Vector3d formula;

        auto& [a, b, c] = formula;

        a = parameters.k1 * motor_control_torques_unrestricted.squaredNorm();
        b = (motor_control_torques_unrestricted.array() * motor_velocities.array())
                .cwiseAbs()
                .sum();
        c = parameters.k2 * motor_velocities.squaredNorm() + parameters.static_power;

        return formula;
    };

    static Eigen::Vector4d calculate_control_torques(
        const double& power_limit, Eigen::Vector3d& formula,
        const Eigen::Vector4d& motor_control_torque_unrestricted) {

        double k               = 0;
        auto& [a, b, c]        = formula;
        double power_predicted = a + b + c;

        if (power_predicted < power_limit) {
            k = 1;
        } else {
            c -= power_limit;
            double delta = b * b - 4 * a * c;
            k            = (-b + std::sqrt(delta)) / (2 * a);
            if (std::isnan(k)) {
                k = 0;
            } else {
                k = std::clamp(k, 0.0, 1.0);
            }
        }

        return motor_control_torque_unrestricted.array() * k; // or use error-based scaling
        // if (delta > 1e-6) {
        //     double root1 = (-b + sqrt(delta)) / (2 * a);
        //     double root2 = (-b - sqrt(delta)) / (2 * a);
        //     if (root1 * velocities_err.sum() > 0) {
        //         torque_predicted = root1;
        //     } else if (root2 * velocities_err.sum() > 0) {
        //         torque_predicted = root2;
        //     }
        // } else {
        //     torque_predicted = -b / (2 * a);
        // }

        // Eigen::Vector4d confidence_coefficient;
        // if (torque_predicted > *power_limit_) {
        //     confidence_coefficient << 1, 1, 1, 1;
        // } else {
        //     confidence_coefficient = velocities_err.array().abs() / velocities_err.sum();
        // }
        // return torque_predicted * confidence_coefficient * motor_control_torque_unstricted;
    }

    Eigen::Vector4d calculate_wheel_velocities() const {
        return {
            *left_front_wheel_velocity_,
            *left_back_wheel_velocity_,
            *right_back_wheel_velocity_,
            *right_front_wheel_velocity_,
        };
    }

    Eigen::Vector4d calculate_steering_velocities() const {
        return {
            *left_front_steering_velocity_,
            *left_back_steering_velocity_,
            *right_back_steering_velocity_,
            *right_front_steering_velocity_,
        };
    }

    Eigen::Vector4d calculate_steering_torque_unrestricted() const {
        return {
            *left_front_steering_control_torque_unrestricted_,
            *left_back_steering_control_torque_unrestricted_,
            *right_back_steering_control_torque_unrestricted_,
            *right_front_steering_control_torque_unrestricted_,
        };
    }

    Eigen::Vector4d calculate_wheel_torques_unrestricted() const {
        return {
            *left_front_wheel_control_torque_unrestricted_,
            *left_back_wheel_control_torque_unrestricted_,
            *right_back_wheel_control_torque_unrestricted_,
            *right_front_wheel_control_torque_unrestricted_,
        };
    }

    void set_wheel_control_torque(const Eigen::Vector4d& wheel_control_torque) {
        *left_front_wheel_control_torque_  = wheel_control_torque[0];
        *left_back_wheel_control_torque_   = wheel_control_torque[1];
        *right_back_wheel_control_torque_  = wheel_control_torque[2];
        *right_front_wheel_control_torque_ = wheel_control_torque[3];
    }

    void set_steering_control_torque(const Eigen::Vector4d& steering_control_torque) {
        *left_front_steering_control_torque_  = steering_control_torque[0];
        *left_back_steering_control_torque_   = steering_control_torque[1];
        *right_back_steering_control_torque_  = steering_control_torque[2];
        *right_front_steering_control_torque_ = steering_control_torque[3];
    }

private:
    Paremeters wheel_parameters_;
    Paremeters steering_parameters_;

    const double power_ratio_;

    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    Eigen::Vector4d motor_control_torque_unstricted_;

    // InputInterface<double> steering_power_;
    // InputInterface<double> chassis_power_;
    InputInterface<double> power_limit_;

    // InputInterface<double> left_front_steering_angle_;
    // InputInterface<double> left_back_steering_angle_;
    // InputInterface<double> right_back_steering_angle_;
    // InputInterface<double> right_front_steering_angle_;
    InputInterface<double> left_front_steering_velocity_;
    InputInterface<double> left_back_steering_velocity_;
    InputInterface<double> right_back_steering_velocity_;
    InputInterface<double> right_front_steering_velocity_;

    InputInterface<double> left_front_wheel_velocity_;
    InputInterface<double> left_back_wheel_velocity_;
    InputInterface<double> right_back_wheel_velocity_;
    InputInterface<double> right_front_wheel_velocity_;

    InputInterface<double> left_front_steering_control_torque_unrestricted_;
    InputInterface<double> left_back_steering_control_torque_unrestricted_;
    InputInterface<double> right_back_steering_control_torque_unrestricted_;
    InputInterface<double> right_front_steering_control_torque_unrestricted_;

    InputInterface<double> left_front_wheel_control_torque_unrestricted_;
    InputInterface<double> left_back_wheel_control_torque_unrestricted_;
    InputInterface<double> right_back_wheel_control_torque_unrestricted_;
    InputInterface<double> right_front_wheel_control_torque_unrestricted_;

    OutputInterface<double> left_front_steering_control_torque_;
    OutputInterface<double> left_back_steering_control_torque_;
    OutputInterface<double> right_back_steering_control_torque_;
    OutputInterface<double> right_front_steering_control_torque_;
    OutputInterface<double> left_front_wheel_control_torque_;
    OutputInterface<double> left_back_wheel_control_torque_;
    OutputInterface<double> right_back_wheel_control_torque_;
    OutputInterface<double> right_front_wheel_control_torque_;

    OutputInterface<double> max_power_predicted_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::PowerControllerTest, rmcs_executor::Component)