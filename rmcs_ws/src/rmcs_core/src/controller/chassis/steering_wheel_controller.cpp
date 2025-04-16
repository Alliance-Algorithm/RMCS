#include <limits>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

#include "controller/pid/pid_calculator.hpp"
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>

namespace rmcs_core::controller::chassis {

class SteeringWheelController
    : public rmcs_executor::Component
    , public rclcpp::Node {

    using WheelVelocityVector = Eigen::Vector2d;

public:
    explicit SteeringWheelController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , wheels_velocity_pid_calculator_(0., 0., 0.)
        , angular_velocity_pid_calculator_(0., 0., 0.) {
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
            "/chassis/left_front_steering/control_angle", left_front_steering_control_angle_, nan_);
        register_output(
            "/chassis/left_back_steering/control_angle", left_back_steering_control_angle_, nan_);
        register_output(
            "/chassis/right_back_steering/control_angle", right_back_steering_control_angle_, nan_);
        register_output(
            "/chassis/right_front_steering/control_angle", right_front_steering_control_angle_,
            nan_);
    }

    void before_updating() override {
        RCLCPP_INFO(
            get_logger(), "Max control torque of wheel motor: %.f",
            *wheel_motor_max_control_torque_);
    }

    void update() override {
        double wheel_velocities[] = {
            *left_front_wheel_velocity_, *left_back_wheel_velocity_, *right_back_wheel_velocity_,
            *right_front_wheel_velocity_};

        std::array<std::reference_wrapper<OutputInterface<double>>, 4> wheels_control_torques = {
            left_front_wheel_control_torque_,
            left_back_wheel_control_torque_,
            right_back_wheel_control_torque_,
            right_front_wheel_control_torque_,
        };

        std::array<std::reference_wrapper<OutputInterface<double>>, 4> steerings_control_angles = {
            left_front_steering_control_angle_,
            left_back_steering_control_angle_,
            right_back_steering_control_angle_,
            right_front_steering_control_angle_,
        };

        WheelVelocityVector translational_control_velocity = control_velocity_->vector.head<2>();
        auto angular_control_velocity                      = control_velocity_->vector[2];

        auto angular_control_velocity_vector = Eigen::Vector3d{0., 0., angular_control_velocity};
        auto chassis_radius_vector =
            Eigen::Vector3d{chassis_radius_ / sqrt2_, chassis_radius_ / sqrt2_, 0.};
        Eigen::Vector3d tangential_velocity =
            angular_control_velocity_vector.cross(chassis_radius_vector);

        WheelVelocityVector angular_control_to_linear_velocity = tangential_velocity.head<2>();

        WheelVelocityVector wheels_control_velocity_vector =
            translational_control_velocity + angular_control_to_linear_velocity;

        double wheels_control_torque =
            update_wheels_control_torque(wheel_velocities, wheels_control_velocity_vector);

        double steerings_control_angle = update_steerings_angles(wheels_control_velocity_vector);

        for (auto& ref : wheels_control_torques) {
            *(ref.get()) = wheels_control_torque;
        }

        for (auto& ref : steerings_control_angles) {
            *(ref.get()) = steerings_control_angle;
        }
    }

private:
    double update_wheels_control_torque(
        const double (&wheel_velocities)[4],
        const WheelVelocityVector& wheels_control_velocity_vector) {

        auto wheels_control_velocity_value = wheels_control_velocity_vector.norm();
        auto wheels_velocity =
            std::accumulate(std::begin(wheel_velocities), std::end(wheel_velocities), 0.)
            / static_cast<double>(std::size(wheel_velocities));

        return wheels_velocity_pid_calculator_.update(
            wheels_control_velocity_value - wheels_velocity);
    }

    double update_steerings_angles(const WheelVelocityVector& wheels_control_velocity_vector) {
        const double delta_angle =
            std::atan2(wheels_control_velocity_vector.y(), wheels_control_velocity_vector.x());
        return delta_angle;
    }

    void update_chassis_control_power() { std::tuple<double, double, double> formula; }

private:
    static constexpr double inf_   = std::numeric_limits<double>::infinity();
    static constexpr double nan_   = std::numeric_limits<double>::quiet_NaN();
    static constexpr double sqrt2_ = std::numbers::sqrt2;

    static constexpr double chassis_radius_ = 0.5617 / 2.;
    static constexpr double wheel_radius_   = 0.11;

    static constexpr double k1_ = 0., k2_ = 0., no_load_power_ = 2.9;

    pid::PidCalculator wheels_velocity_pid_calculator_;
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

    OutputInterface<double> left_front_steering_control_angle_;
    OutputInterface<double> left_back_steering_control_angle_;
    OutputInterface<double> right_back_steering_control_angle_;
    OutputInterface<double> right_front_steering_control_angle_;
};
} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::SteeringWheelController, rmcs_executor::Component)