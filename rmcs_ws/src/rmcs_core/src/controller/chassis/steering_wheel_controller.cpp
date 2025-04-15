#include <limits>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

#include "controller/pid/matrix_pid_calculator.hpp"
#include "controller/pid/pid_calculator.hpp"
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::chassis {

class SteeringWheelController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit SteeringWheelController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , wheels_velocity_pid_calculator_(0., 0., 0.)
        , angular_velocity_pid_calculator_(0., 0., 0.) {

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

        register_output(
            "/chassis/left_front_wheel/control_torque", left_front_wheel_control_torque_, nan_);
        register_output(
            "/chassis/left_back_wheel/control_torque", left_back_wheel_control_torque_, nan_);
        register_output(
            "/chassis/right_back_wheel/control_torque", right_back_wheel_control_torque_, nan_);
        register_output(
            "/chassis/right_front_wheel/control_torque", right_front_wheel_control_torque_, nan_);

        register_output(
            "/chassis/left_front_steering/control_angle", left_front_steering_control_angle_, nan_);
        register_output(
            "/chassis/left_back_steering/control_angle", left_back_steering_control_angle_, nan_);
        register_output(
            "/chassis/right_back_steering/control_angle", right_back_steering_control_angle_, nan_);
        register_output(
            "/chassis/right_front_steering/control_angle", right_front_steering_control_angle_,
            nan_);
    }

    void update() override {
        double wheel_velocities[] = {
            *left_front_wheel_velocity_, *left_back_wheel_velocity_, *right_back_wheel_velocity_,
            *right_front_wheel_velocity_};

        Eigen::Vector2d translational_control_velocity = control_velocity_->vector.head<2>();
        auto angular_control_velocity                  = control_velocity_->vector[2];

        auto angular_control_velocity_vector = Eigen::Vector3d{0., 0., angular_control_velocity};
        auto chassis_radius_vector =
            Eigen::Vector3d{chassis_radius_ / sqrt2_, chassis_radius_ / sqrt2_, 0.};

        Eigen::Vector3d tangential_velocity =
            angular_control_velocity_vector.cross(chassis_radius_vector);
        Eigen::Vector2d angular_control_to_linear_velocity = tangential_velocity.head<2>();

        Eigen::Vector2d wheels_control_velocity =
            translational_control_velocity + angular_control_to_linear_velocity;
    }

private:
    double update_wheels_control_torques(
        const double (&wheel_velocities)[4], const Eigen::Vector2d& wheels_control_velocity) {

        auto wheels_control_velocity_value = wheels_control_velocity.norm();
        auto wheels_velocity =
            std::accumulate(std::begin(wheel_velocities), std::end(wheel_velocities), 0.) / 4;

        return wheels_velocity_pid_calculator_.update(
            wheels_control_velocity_value - wheels_velocity);
    }

    void update_steerings_angles() {}

private:
    static constexpr double inf_   = std::numeric_limits<double>::infinity();
    static constexpr double nan_   = std::numeric_limits<double>::quiet_NaN();
    static constexpr double sqrt2_ = std::numbers::sqrt2;

    static constexpr double chassis_radius_ = 0.5617 / 2.;
    static constexpr double wheel_radius_   = 0.11;

    pid::PidCalculator wheels_velocity_pid_calculator_;
    pid::PidCalculator angular_velocity_pid_calculator_;

    InputInterface<double> left_front_steering_angle_;
    InputInterface<double> left_back_steering_angle_;
    InputInterface<double> right_back_steering_angle_;
    InputInterface<double> right_front_steering_angle_;

    InputInterface<double> left_front_wheel_velocity_;
    InputInterface<double> left_back_wheel_velocity_;
    InputInterface<double> right_back_wheel_velocity_;
    InputInterface<double> right_front_wheel_velocity_;

    InputInterface<rmcs_description::BaseLink::DirectionVector> control_velocity_;
    InputInterface<double> power_limit_;

    OutputInterface<double> left_front_wheel_control_torque_;
    OutputInterface<double> left_back_wheel_control_torque_;
    OutputInterface<double> right_back_wheel_control_torque_;
    OutputInterface<double> right_front_wheel_control_torque_;

    OutputInterface<double> left_front_steering_control_angle_;
    OutputInterface<double> left_back_steering_control_angle_;
    OutputInterface<double> right_back_steering_control_angle_;
    OutputInterface<double> right_front_steering_control_angle_;
};
} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::SteeringWheelController, rmcs_executor::Component)