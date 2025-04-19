#include <limits>
#include <numeric>

#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>

#include "controller/pid/matrix_pid_calculator.hpp"
#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::chassis {

class SteeringWheelController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit SteeringWheelController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , translational_velocity_pid_calculator_(5., 0., 0.)
        , angular_velocity_pid_calculator_(20., 0., 0.) {
        register_input("/chassis/left_front_wheel/max_torque", wheel_motor_max_control_torque_);
        register_input(
            "/chassis/left_front_steering/max_torque", steering_motor_max_control_torque_);

        register_input("/chassis/left_front_steering/angle", left_front_steering_angle_);
        register_input("/chassis/left_back_steering/angle", left_back_steering_angle_);
        register_input("/chassis/right_back_steering/angle", right_back_steering_angle_);
        register_input("/chassis/right_front_steering/angle", right_front_steering_angle_);

        register_input("/chassis/left_front_wheel/velocity", left_front_wheel_velocity_);
        register_input("/chassis/left_back_wheel/velocity", left_back_wheel_velocity_);
        register_input("/chassis/right_back_wheel/velocity", right_back_wheel_velocity_);
        register_input("/chassis/right_front_wheel/velocity", right_front_wheel_velocity_);

        register_input("/chassis/control_velocity", control_velocity_, false);
        register_input("/chassis/control_power_limit", power_limit_, false);
        register_input("/chassis/control_mode", mode_, false);

        register_output(
            "/chassis/left_front_wheel/control_torque", left_front_wheel_control_torque_, nan_);
        register_output(
            "/chassis/left_back_wheel/control_torque", left_back_wheel_control_torque_, nan_);
        register_output(
            "/chassis/right_back_wheel/control_torque", right_back_wheel_control_torque_, nan_);
        register_output(
            "/chassis/right_front_wheel/control_torque", right_front_wheel_control_torque_, nan_);

        register_output(
            "/chassis/left_front_steering/control_angle_error",
            left_front_steering_control_angle_error_, nan_);
        register_output(
            "/chassis/left_back_steering/control_angle_error",
            left_back_steering_control_angle_error_, nan_);
        register_output(
            "/chassis/right_back_steering/control_angle_error",
            right_back_steering_control_angle_error_, nan_);
        register_output(
            "/chassis/right_front_steering/control_angle_error",
            right_front_steering_control_angle_error_, nan_);
    }

    void before_updating() override {
        RCLCPP_INFO(
            get_logger(), "Max control torque of wheel motor: %.f",
            *wheel_motor_max_control_torque_);
    }

    void update() override {
        Eigen::Vector3d wheel_velocities[4] = {
            Eigen::AngleAxisd(*left_front_steering_angle_, Eigen::Vector3d::UnitZ())
                * Eigen::Vector3d::UnitX() * *left_front_wheel_velocity_ * wheel_radius_,
            Eigen::AngleAxisd(*left_back_steering_angle_, Eigen::Vector3d::UnitZ())
                * Eigen::Vector3d::UnitX() * *left_back_wheel_velocity_ * wheel_radius_,
            Eigen::AngleAxisd(*right_back_steering_angle_, Eigen::Vector3d::UnitZ())
                * Eigen::Vector3d::UnitX() * *right_back_wheel_velocity_ * wheel_radius_,
            Eigen::AngleAxisd(*right_front_steering_angle_, Eigen::Vector3d::UnitZ())
                * Eigen::Vector3d::UnitX() * *right_front_wheel_velocity_ * wheel_radius_,
        };

        auto control_velocity = control_velocity_->vector;

        auto [translational_control_torque, angular_control_torque] =
            update_control_torque(wheel_velocities, control_velocity);

        update_wheels_control_torque(translational_control_torque, angular_control_torque);
        update_steerings_control_angle(translational_control_torque, angular_control_torque);
    }

private:
    auto update_control_torque(
        const Eigen::Vector3d (&velocities)[4], const Eigen::Vector3d& control_vector)
        -> std::pair<Eigen::Vector2d, double> {

        std::pair<Eigen::Vector2d, double> result{};
        auto& [translational_control_torque_vector, angular_control_torque] = result;

        const Eigen::Vector2d vels[4] = {
            velocities[0].head<2>(),
            velocities[1].head<2>(),
            velocities[2].head<2>(),
            velocities[3].head<2>(),
        };

        auto translational_velocity = Eigen::Vector2d{};
        translational_velocity =
            std::accumulate(std::begin(vels), std::end(vels), Eigen::Vector2d{})
            / static_cast<double>(std::size(vels));

        double angular_velocity = 0.;
        for (auto& vel : velocities)
            angular_velocity += (vel.head<2>() - translational_velocity).norm();
        angular_velocity = angular_velocity / 4. / chassis_radius_;

        auto translational_control_velocity =
            Eigen::Vector2d{control_vector.x(), control_vector.y()};

        translational_control_torque_vector =
            translational_velocity_pid_calculator_.update(Eigen::Vector2d{
                translational_control_velocity.x() - translational_velocity.x(),
                translational_control_velocity.y() - translational_velocity.y()});

        auto angular_control_velocity = control_vector.z();

        if (!std::isnan(angular_control_velocity)) {
            angular_control_torque = -wheel_radius_
                                   * angular_velocity_pid_calculator_.update(
                                       angular_control_velocity - angular_velocity);
        } else {
            angular_control_velocity = 0;
        }

        return result;
    }

    void update_wheels_control_torque(
        const Eigen::Vector2d& translational_control_tor, const double angular_control_tor) {
        Eigen::Vector2d lf_control_torque =
            Eigen::Vector2d{-angular_control_tor / sqrt2_, angular_control_tor / sqrt2_}
            + translational_control_tor;
        Eigen::Vector2d lb_control_torque =
            Eigen::Vector2d{-angular_control_tor / sqrt2_, -angular_control_tor / sqrt2_}
            + translational_control_tor;
        Eigen::Vector2d rb_control_torque =
            Eigen::Vector2d{angular_control_tor / sqrt2_, -angular_control_tor / sqrt2_}
            + translational_control_tor;
        Eigen::Vector2d rf_control_torque =
            Eigen::Vector2d{angular_control_tor / sqrt2_, angular_control_tor / sqrt2_}
            + translational_control_tor;

        auto test_vector = Eigen::Vector2d{};

        // *left_front_wheel_control_torque_  = lf_control_torque.norm();
        // *left_back_wheel_control_torque_   = lb_control_torque.norm();
        // *right_back_wheel_control_torque_  = rb_control_torque.norm();
        // *right_front_wheel_control_torque_ = rf_control_torque.norm();

        RCLCPP_INFO(
            get_logger(), "rb norm:%f,rf norm:%f", rb_control_torque.norm(),
            rf_control_torque.norm());
    }

    void update_steerings_control_angle(
        const Eigen::Vector2d& translational_control_tor, const double angular_control_tor) {
        Eigen::Vector2d lf_control_vector =
            Eigen::Vector2d{-angular_control_tor / sqrt2_, angular_control_tor / sqrt2_}
            + translational_control_tor;
        Eigen::Vector2d lb_control_vector =
            Eigen::Vector2d{-angular_control_tor / sqrt2_, -angular_control_tor / sqrt2_}
            + translational_control_tor;
        Eigen::Vector2d rb_control_vector =
            Eigen::Vector2d{angular_control_tor / sqrt2_, -angular_control_tor / sqrt2_}
            + translational_control_tor;
        Eigen::Vector2d rf_control_vector =
            Eigen::Vector2d{angular_control_tor / sqrt2_, angular_control_tor / sqrt2_}
            + translational_control_tor;

        double current_angle[4] = {
            (*left_front_steering_angle_ < std::numbers::pi && *left_front_steering_angle_ > 0)
                ? *left_front_steering_angle_
                : *left_front_steering_angle_ - 2 * std::numbers::pi,
            (*left_back_steering_angle_ < std::numbers::pi && *left_back_steering_angle_ > 0)
                ? *left_back_steering_angle_
                : *left_back_steering_angle_ - 2 * std::numbers::pi,
            (*right_back_steering_angle_ < std::numbers::pi && *right_back_steering_angle_ > 0)
                ? *right_back_steering_angle_
                : *right_back_steering_angle_ - 2 * std::numbers::pi,
            (*right_front_steering_angle_ < std::numbers::pi && *right_front_steering_angle_ > 0)
                ? *right_front_steering_angle_
                : *right_front_steering_angle_ - 2 * std::numbers::pi};

        // *left_front_steering_control_angle_error_ =
        //     atan2(lf_control_vector.y(), lf_control_vector.x())
        //     // revise_angle_error(atan2(lf_control_vector.y(),
        //     // lf_control_vector.x()))
        //     - *left_front_steering_angle_;

        // *left_back_steering_control_angle_error_ =
        //     atan2(lb_control_vector.y(), lb_control_vector.x())
        //     // revise_angle_error(atan2(lb_control_vector.y(), lb_control_vector.x()))
        //     - *left_back_steering_angle_;

        *right_back_steering_control_angle_error_ =
            atan2(rb_control_vector.y(), rb_control_vector.x())
            // revise_angle_error(atan2(rb_control_vector.y(), rb_control_vector.x()))
            - current_angle[2];

        *right_front_steering_control_angle_error_ =
            atan2(rf_control_vector.y(), rf_control_vector.x())
            // revise_angle_error(atan2(rf_control_vector.y(), rf_control_vector.x()))
            - current_angle[3];

        // RCLCPP_INFO(
        //     get_logger(), "rb target:%f,rf target:%f",
        //     (atan2(rf_control_vector.y(), rf_control_vector.x())), current_angle[3]);
    }

    static double revise_angle_error(double angle_error) {
        if (angle_error < std::numbers::pi && angle_error > std::numbers::pi / 2) {
            angle_error -= std::numbers::pi;
        } else if (angle_error > -std::numbers::pi && angle_error < -std::numbers::pi / 2) {
            angle_error += std::numbers::pi;
        }
        return angle_error;
    }

private:
    static constexpr double inf_   = std::numeric_limits<double>::infinity();
    static constexpr double nan_   = std::numeric_limits<double>::quiet_NaN();
    static constexpr double sqrt2_ = std::numbers::sqrt2;

    static constexpr double chassis_radius_ = 0.5617 / 2.;
    static constexpr double wheel_radius_   = 0.11;

    static constexpr double k1_ = 1., k2_ = 1., no_load_power_ = 2.9;

    static constexpr double font_freq = 0.01, translation_accelerate_limit = 1.5;

    static constexpr double filter_tau_   = 1.0;
    static constexpr double update_freq_  = 0.001;
    static constexpr double filter_alpha_ = update_freq_ / (filter_tau_ + update_freq_);

    pid::MatrixPidCalculator<2> translational_velocity_pid_calculator_;
    pid::PidCalculator angular_velocity_pid_calculator_;

    InputInterface<double> wheel_motor_max_control_torque_;
    InputInterface<double> steering_motor_max_control_torque_;

    InputInterface<double> left_front_steering_angle_;
    InputInterface<double> left_back_steering_angle_;
    InputInterface<double> right_back_steering_angle_;
    InputInterface<double> right_front_steering_angle_;

    InputInterface<double> left_front_wheel_velocity_;
    InputInterface<double> left_back_wheel_velocity_;
    InputInterface<double> right_back_wheel_velocity_;
    InputInterface<double> right_front_wheel_velocity_;

    InputInterface<rmcs_description::BaseLink::DirectionVector> control_velocity_;
    InputInterface<rmcs_msgs::ChassisMode> mode_;
    InputInterface<double> power_limit_;

    OutputInterface<double> left_front_wheel_control_torque_;
    OutputInterface<double> left_back_wheel_control_torque_;
    OutputInterface<double> right_back_wheel_control_torque_;
    OutputInterface<double> right_front_wheel_control_torque_;

    OutputInterface<double> left_front_steering_control_angle_error_;
    OutputInterface<double> left_back_steering_control_angle_error_;
    OutputInterface<double> right_back_steering_control_angle_error_;
    OutputInterface<double> right_front_steering_control_angle_error_;
}; // namespace rmcs_core::controller::chassis
} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::SteeringWheelController, rmcs_executor::Component)