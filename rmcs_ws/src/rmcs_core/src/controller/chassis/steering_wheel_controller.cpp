#include <limits>
#include <numeric>

#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>

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
            "/chassis/left_front_steering/control_angle_error", left_front_steering_control_angle_,
            nan_);
        register_output(
            "/chassis/left_back_steering/control_angle_error", left_back_steering_control_angle_,
            nan_);
        register_output(
            "/chassis/right_back_steering/control_angle_error", right_back_steering_control_angle_,
            nan_);
        register_output(
            "/chassis/right_front_steering/control_angle_error",
            right_front_steering_control_angle_, nan_);
    }

    void before_updating() override {
        RCLCPP_INFO(
            get_logger(), "Max control torque of wheel motor: %.f",
            *wheel_motor_max_control_torque_);
    }

    void update() override {
        // Eigen::Vector3d wheel_velocities[4] = {
        //     Eigen::AngleAxisd(*left_front_steering_angle_, Eigen::Vector3d::UnitZ())
        //         * Eigen::Vector3d::UnitX() * *left_front_wheel_velocity_ * wheel_radius_,
        //     Eigen::AngleAxisd(*left_back_steering_angle_, Eigen::Vector3d::UnitZ())
        //         * Eigen::Vector3d::UnitX() * *left_back_wheel_velocity_ * wheel_radius_,
        //     Eigen::AngleAxisd(*right_back_steering_angle_, Eigen::Vector3d::UnitZ())
        //         * Eigen::Vector3d::UnitX() * *right_back_wheel_velocity_ * wheel_radius_,
        //     Eigen::AngleAxisd(*right_front_steering_angle_, Eigen::Vector3d::UnitZ())
        //         * Eigen::Vector3d::UnitX() * *right_front_wheel_velocity_ * wheel_radius_,
        // };

        // auto [translational_velocity, angular_velocity] =
        // update_current_velocity(wheel_velocities);
    }

private:
    auto update_current_velocity(const Eigen::Vector3d (&velocities)[4])
        -> std::pair<Eigen::Vector2d, double> {

        // std::pair<Eigen::Vector2d, double> result{};
        // auto& [translational_velocity, angular_velocity] = result;

        // const Eigen::Vector2d vels[4] = {
        //     velocities[0].head<2>(),
        //     velocities[1].head<2>(),
        //     velocities[2].head<2>(),
        //     velocities[3].head<2>(),
        // };

        // translational_velocity =
        //     std::accumulate(std::begin(vels), std::end(vels), Eigen::Vector2d{})
        //     / static_cast<double>(std::size(vels));

        // for (auto& vel : velocities)
        //     angular_velocity += (vel.head<2>() - translational_velocity).norm();
        // angular_velocity = angular_velocity / 4. / chassis_radius_;

        // RCLCPP_INFO(get_logger(), "");
        // return result;
    }

    // double update_wheels_control_torque(
    // const Eigen::Vector2d& translational_control_vel, const double angular_control_vel) {
    // auto lf_control_velocity =
    //     Eigen::Vector2d{-angular_control_vel / sqrt2_, angular_control_vel / sqrt2_}
    //     + translational_control_vel;
    // auto lb_control_velocity =
    //     Eigen::Vector2d{-angular_control_vel / sqrt2_, angular_control_vel / sqrt2_}
    //     + translational_control_vel;
    // auto rb_control_velocity =
    //     Eigen::Vector2d{-angular_control_vel / sqrt2_, angular_control_vel / sqrt2_}
    //     + translational_control_vel;
    // auto rf_control_velocity =
    //     Eigen::Vector2d{-angular_control_vel / sqrt2_, angular_control_vel / sqrt2_}
    //     + translational_control_vel;
    // }

    // double update_steerings_control_angle(
    //     const Eigen::Vector2d& translational_control_vel, const double angular_control_vel) {
    // auto lf_control_vector =
    //     Eigen::Vector2d{-angular_control_vel / sqrt2_, angular_control_vel / sqrt2_}
    //     + translational_control_vel;
    // auto lb_control_vector =
    //     Eigen::Vector2d{-angular_control_vel / sqrt2_, angular_control_vel / sqrt2_}
    //     + translational_control_vel;
    // auto rb_control_vector =
    //     Eigen::Vector2d{-angular_control_vel / sqrt2_, angular_control_vel / sqrt2_}
    //     + translational_control_vel;
    // auto rf_control_vector =
    //     Eigen::Vector2d{-angular_control_vel / sqrt2_, angular_control_vel / sqrt2_}
    //     + translational_control_vel;

    // *left_front_steering_control_angle_ = revise_angle_error(
    //     atan2(lf_control_vector.y(), lf_control_vector.x()) - *left_front_steering_angle_);

    // *left_back_steering_control_angle_ = revise_angle_error(
    //     atan2(lb_control_vector.y(), lb_control_vector.x()) - *left_back_steering_angle_);

    // *right_back_steering_control_angle_ = revise_angle_error(
    //     atan2(rb_control_vector.y(), rb_control_vector.x()) - *right_back_steering_angle_);

    // *right_front_steering_control_angle_ = revise_angle_error(
    //     atan2(rf_control_vector.y(), rf_control_vector.x()) - *right_front_steering_angle_);
    // }

    // static double revise_angle_error(double angle_error) {}

private:
    static constexpr double inf_   = std::numeric_limits<double>::infinity();
    static constexpr double nan_   = std::numeric_limits<double>::quiet_NaN();
    static constexpr double sqrt2_ = std::numbers::sqrt2;

    static constexpr double chassis_radius_ = 0.5617 / 2.;
    static constexpr double wheel_radius_   = 0.11;

    static constexpr double k1_ = 1., k2_ = 1., no_load_power_ = 2.9;

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
}; // namespace rmcs_core::controller::chassis
} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::SteeringWheelController, rmcs_executor::Component)