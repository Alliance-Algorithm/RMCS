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
        , translational_velocity_pid_calculator_(0.685, 0., 0.)
        , angular_velocity_pid_calculator_(0.5, 0., 0.) {
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

        // register_output(
        //     "/chassis/left_front_wheel/control_torque", left_front_wheel_control_torque_, nan_);
        // register_output(
        //     "/chassis/left_back_wheel/control_torque", left_back_wheel_control_torque_, nan_);
        // register_output(
        //     "/chassis/right_back_wheel/control_torque", right_back_wheel_control_torque_, nan_);
        // register_output(
        //     "/chassis/right_front_wheel/control_torque", right_front_wheel_control_torque_,
        //     nan_);

        register_output(
            "/chassis/left_front_wheel/control_velocity", left_front_wheel_control_velocity_, nan_);
        register_output(
            "/chassis/left_back_wheel/control_velocity", left_back_wheel_control_velocity_, nan_);
        register_output(
            "/chassis/right_back_wheel/control_velocity", right_back_wheel_control_velocity_, nan_);
        register_output(
            "/chassis/right_front_wheel/control_velocity", right_front_wheel_control_velocity_,
            nan_);

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

        RCLCPP_INFO(
            get_logger(), "set:%f,measure:%f,control:%f", translational_control_velocity.norm(),
            translational_velocity.norm(), translational_control_torque_vector.norm());

        if (translational_control_velocity.norm() <= 0.005) {
            translational_control_torque_vector = Eigen::Vector2d::Zero();
        }

        auto angular_control_velocity = control_vector.z();

        if (!std::isnan(angular_control_velocity)) {
            angular_control_torque = wheel_radius_
                                   * angular_velocity_pid_calculator_.update(
                                       angular_control_velocity - angular_velocity);
        }

        return result;
    }

    void update_wheels_control_torque(
        const Eigen::Vector2d& translational_control_tor, const double angular_control_tor) {

        auto control_velocity = control_velocity_->vector;

        auto translational_control_velocity =
            Eigen::Vector2d{control_velocity.x(), control_velocity.y()};

        auto angular_control_velocity = 0.;

        if (!std::isnan(control_velocity.z())) {
            angular_control_velocity = control_velocity.z();
        }

        Eigen::Vector2d lf_control_velocity =
            Eigen::Vector2d{-angular_control_velocity / sqrt2_, angular_control_velocity / sqrt2_}
            + translational_control_velocity;
        Eigen::Vector2d lb_control_velocity =
            Eigen::Vector2d{-angular_control_velocity / sqrt2_, -angular_control_velocity / sqrt2_}
            + translational_control_velocity;
        Eigen::Vector2d rb_control_velocity =
            Eigen::Vector2d{angular_control_velocity / sqrt2_, -angular_control_velocity / sqrt2_}
            + translational_control_velocity;
        Eigen::Vector2d rf_control_velocity =
            Eigen::Vector2d{angular_control_velocity / sqrt2_, angular_control_velocity / sqrt2_}
            + translational_control_velocity;

        *left_front_wheel_control_velocity_  = lf_control_velocity.norm() / wheel_radius_;
        *left_back_wheel_control_velocity_   = lb_control_velocity.norm() / wheel_radius_;
        *right_back_wheel_control_velocity_  = rb_control_velocity.norm() / wheel_radius_;
        *right_front_wheel_control_velocity_ = rf_control_velocity.norm() / wheel_radius_;

        // Eigen::Vector2d lf_control_torque =
        //     Eigen::Vector2d{-angular_control_tor / sqrt2_, angular_control_tor / sqrt2_}
        //     + translational_control_tor;
        // Eigen::Vector2d lb_control_torque =
        //     Eigen::Vector2d{-angular_control_tor / sqrt2_, -angular_control_tor / sqrt2_}
        //     + translational_control_tor;
        // Eigen::Vector2d rb_control_torque =
        //     Eigen::Vector2d{angular_control_tor / sqrt2_, -angular_control_tor / sqrt2_}
        //     + translational_control_tor;
        // Eigen::Vector2d rf_control_torque =
        //     Eigen::Vector2d{angular_control_tor / sqrt2_, angular_control_tor / sqrt2_}
        //     + translational_control_tor;

        // *left_front_wheel_control_torque_  = lf_control_torque.norm();
        // *left_back_wheel_control_torque_   = lb_control_torque.norm();
        // *right_back_wheel_control_torque_  = rb_control_torque.norm();
        // *right_front_wheel_control_torque_ = rf_control_torque.norm();

        // RCLCPP_INFO(
        //     get_logger(), "lf:%f,lb:%f,rb:%f,rf:%f", *left_front_wheel_control_torque_,
        //     *left_back_wheel_control_torque_, *right_back_wheel_control_torque_,
        //     *right_front_wheel_control_torque_);
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

        double control_angle[4] = {
            atan2(lf_control_vector.y(), lf_control_vector.x()),
            atan2(lb_control_vector.y(), lb_control_vector.x()),
            atan2(rb_control_vector.y(), rb_control_vector.x()),
            atan2(rf_control_vector.y(), rf_control_vector.x())};

        *left_front_steering_control_angle_error_ =
            revise_angle_error(control_angle[0], *left_front_steering_angle_);

        *left_back_steering_control_angle_error_ =
            revise_angle_error(control_angle[1], *left_back_steering_angle_);

        *right_back_steering_control_angle_error_ =
            revise_angle_error(control_angle[2], *right_back_steering_angle_);

        *right_front_steering_control_angle_error_ =
            revise_angle_error(control_angle[3], *right_front_steering_angle_);
    }

    double revise_angle_error(double target_angle, double measure_angle) {

        if (measure_angle > std::numbers::pi) {
            measure_angle -= 2 * std::numbers::pi;
        }

        auto raw_error = target_angle - measure_angle;

        if (raw_error >= std::numbers::pi) {
            target_angle -= 2 * std::numbers::pi;
        } else if (raw_error <= -std::numbers::pi) {
            target_angle += 2 * std::numbers::pi;
        }

        auto error = target_angle - measure_angle;

        // if (error < std::numbers::pi && error > std::numbers::pi / 2)
        //     target_angle -= std::numbers::pi;
        // else if (error > -std::numbers::pi && error < -std::numbers::pi / 2)
        //     target_angle += std::numbers::pi;

        return error;
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

    double last_raw_error_ = 0.;

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

    // OutputInterface<double> left_front_wheel_control_torque_;
    // OutputInterface<double> left_back_wheel_control_torque_;
    // OutputInterface<double> right_back_wheel_control_torque_;
    // OutputInterface<double> right_front_wheel_control_torque_;

    OutputInterface<double> left_front_wheel_control_velocity_;
    OutputInterface<double> left_back_wheel_control_velocity_;
    OutputInterface<double> right_back_wheel_control_velocity_;
    OutputInterface<double> right_front_wheel_control_velocity_;

    OutputInterface<double> left_front_steering_control_angle_error_;
    OutputInterface<double> left_back_steering_control_angle_error_;
    OutputInterface<double> right_back_steering_control_angle_error_;
    OutputInterface<double> right_front_steering_control_angle_error_;
};
} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::SteeringWheelController, rmcs_executor::Component)