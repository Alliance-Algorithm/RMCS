#include <cmath>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

#include <controller/pid/matrix_pid_calculator.hpp>

#include "controller/pid/pid_calculator.hpp"
#include "rmcs_utility/eigen_structured_bindings.hpp"

namespace rmcs_core::controller::chassis {

class SteeringWheelController
    : public rmcs_executor::Component
    , public rclcpp::Node {

    using Formula = std::tuple<double, double, double>;

public:
    explicit SteeringWheelController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , chassis_translational_velocity_pid_(2.0, 0.0, 0.0)
        , chassis_angular_velocity_pid_(2.0, 0.0, 0.0)
        , cos_varphi_(1, 0, -1, 0) // 0, pi/2, pi, 3pi/2
        , sin_varphi_(0, 1, 0, -1)
        , steering_velocity_pid_(0.15, 0.0, 0.0)
        , steering_angle_pid_(80.0, 0.0, 0.0) {

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);

        register_input("/chassis/left_front_steering/angle", left_front_steering_angle_);
        register_input("/chassis/left_back_steering/angle", left_back_steering_angle_);
        register_input("/chassis/right_back_steering/angle", right_back_steering_angle_);
        register_input("/chassis/right_front_steering/angle", right_front_steering_angle_);

        register_input("/chassis/left_front_steering/velocity", left_front_steering_velocity_);
        register_input("/chassis/left_back_steering/velocity", left_back_steering_velocity_);
        register_input("/chassis/right_back_steering/velocity", right_back_steering_velocity_);
        register_input("/chassis/right_front_steering/velocity", right_front_steering_velocity_);

        register_input("/chassis/left_front_wheel/velocity", left_front_wheel_velocity_);
        register_input("/chassis/left_back_wheel/velocity", left_back_wheel_velocity_);
        register_input("/chassis/right_back_wheel/velocity", right_back_wheel_velocity_);
        register_input("/chassis/right_front_wheel/velocity", right_front_wheel_velocity_);

        register_input("/chassis/control_velocity", chassis_control_velocity_);

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
    }

    void update() override {
        auto wheel_velocities = calculate_wheel_velocities();
        auto steering_status  = calculate_steering_status();
        auto chassis_velocity = calculate_chassis_velocity(wheel_velocities, steering_status);

        // RCLCPP_INFO(
        //     get_logger(), "Chassis velocity: %.2f, %.2f, %.2f", chassis_velocity.x(),
        //     chassis_velocity.y(), chassis_velocity.z());

        Eigen::Vector3d chassis_acceleration =
            calculate_chassis_control_acceleration(chassis_velocity);

        // chassis_velocity = {0, 0, 0};

        // chassis_acceleration.x() = joystick_right_->x();
        // chassis_acceleration.y() = joystick_right_->y();
        // chassis_acceleration.z() = joystick_left_->y();
        if (chassis_acceleration.norm() < 1e-1) {
            chassis_acceleration = {nan_, nan_, nan_};
        }

        // RCLCPP_INFO(
        //     get_logger(), "Chassis acceleration: %.2f, %.2f, %.2f", chassis_acceleration.x(),
        //     chassis_acceleration.y(), chassis_acceleration.z());

        // Eigen::Vector3d chassis_acceleration;
        // chassis_acceleration << Eigen::Rotation2Dd(-std::numbers::pi / 4) * *joystick_right_,
        //     joystick_left_->y();
        // chassis_acceleration *= 4.0;

        update_steering_control_torques(steering_status, chassis_velocity, chassis_acceleration);
        update_wheel_torques(steering_status, chassis_acceleration);
    }

private:
    struct SteeringStatus {
        Eigen::Vector4d angle, cos_angle, sin_angle;
        Eigen::Vector4d velocity;
    };

    Eigen::Vector4d calculate_wheel_velocities() {
        return {
            *left_front_wheel_velocity_,    //
            *left_back_wheel_velocity_,     //
            *right_back_wheel_velocity_,    //
            *right_front_wheel_velocity_    //
        };
    }

    SteeringStatus calculate_steering_status() {
        SteeringStatus steering_status;

        steering_status.angle = {
            *left_front_steering_angle_,    //
            *left_back_steering_angle_,     //
            *right_back_steering_angle_,    //
            *right_front_steering_angle_    //
        };
        steering_status.angle.array() -= std::numbers::pi / 4;
        steering_status.cos_angle = steering_status.angle.array().cos();
        steering_status.sin_angle = steering_status.angle.array().sin();

        steering_status.velocity = {
            *left_front_steering_velocity_, //
            *left_back_steering_velocity_,  //
            *right_back_steering_velocity_, //
            *right_front_steering_velocity_ //
        };

        return steering_status;
    }

    static Eigen::Vector3d calculate_chassis_velocity(
        const Eigen::Vector4d& wheel_velocities, const SteeringStatus& steering_status) {
        Eigen::Vector3d velocity;
        double one_quarter_r = wheel_radius_ / 4.0;
        velocity.x() =
            one_quarter_r * (wheel_velocities.array() * steering_status.cos_angle.array()).sum();
        velocity.y() =
            one_quarter_r * (wheel_velocities.array() * steering_status.sin_angle.array()).sum();
        velocity.z() = -one_quarter_r / vehicle_radius_
                     * (-wheel_velocities[0] * steering_status.sin_angle[0]
                        + wheel_velocities[1] * steering_status.cos_angle[1]
                        + wheel_velocities[2] * steering_status.sin_angle[2]
                        - wheel_velocities[3] * steering_status.cos_angle[3]);
        return velocity;
    }

    Eigen::Vector3d
        calculate_chassis_control_acceleration(const Eigen::Vector3d& chassis_velocity) {
        Eigen::Vector2d translational_control_velocity =
            Eigen::Rotation2Dd(-std::numbers::pi / 4) * chassis_control_velocity_->vector.head<2>();
        Eigen::Vector2d translational_velocity = chassis_velocity.head<2>();
        Eigen::Vector2d translational_control_acceleration =
            chassis_translational_velocity_pid_.update(
                translational_control_velocity - translational_velocity);

        double angular_control_velocity = chassis_control_velocity_->vector[2];
        // angular_control_velocity        = joystick_right_->x();

        double angular_velocity = chassis_velocity[2];
        double angular_control_acceleration =
            chassis_angular_velocity_pid_.update(angular_control_velocity - angular_velocity);

        Eigen::Vector3d chassis_control_acceleration;
        chassis_control_acceleration << translational_control_acceleration,
            angular_control_acceleration;
        return chassis_control_acceleration;
    }

    void update_steering_control_torques(
        const SteeringStatus& steering_status, const Eigen::Vector3d& chassis_velocity,
        const Eigen::Vector3d& chassis_acceleration) {
        if (std::isnan(chassis_acceleration[0])) {
            *left_front_steering_control_torque_  = 0.0;
            *left_back_steering_control_torque_   = 0.0;
            *right_back_steering_control_torque_  = 0.0;
            *right_front_steering_control_torque_ = 0.0;
            return;
        }

        const auto& [vx, vy, vz] = chassis_velocity;
        const auto& [ax, ay, az] = chassis_acceleration;

        Eigen::Vector4d dot_rx        = vx - vehicle_radius_ * vz * sin_varphi_.array();
        Eigen::Vector4d dot_ry        = vy + vehicle_radius_ * vz * cos_varphi_.array();
        Eigen::Vector4d dot_r_squared = dot_rx.array().square() + dot_ry.array().square();

        Eigen::Vector4d steering_control_velocities =
            vx * ay - vy * ax - vz * (vx * vx + vy * vy)
            + vehicle_radius_ * (az * vx - vz * (ax + vz * vy)) * cos_varphi_.array()
            + vehicle_radius_ * (az * vy - vz * (ay - vz * vx)) * sin_varphi_.array();
        Eigen::Vector4d steering_control_angles;

        for (int i = 0; i < steering_control_velocities.size(); ++i) {
            if (dot_r_squared[i] > 1e-2) {
                steering_control_velocities[i] /= dot_r_squared[i];
                steering_control_angles[i] = std::atan2(dot_ry[i], dot_rx[i]);
            } else {
                auto x = ax - vehicle_radius_ * (az * sin_varphi_[i] + 0 * cos_varphi_[i]);
                auto y = ay + vehicle_radius_ * (az * cos_varphi_[i] - 0 * sin_varphi_[i]);
                if (x * x + y * y > 1e-6) {
                    steering_control_velocities[i] = 0.0;
                    steering_control_angles[i]     = std::atan2(y, x);
                } else {
                    steering_control_velocities[i] = nan_;
                    steering_control_angles[i]     = nan_;
                }
            }
        }

        // steering_control_angles = {0, 0, 0, 0};

        Eigen::Vector4d control_torques = steering_velocity_pid_.update(
            steering_control_velocities
            + steering_angle_pid_.update(
                (steering_control_angles - steering_status.angle).unaryExpr([](double diff) {
                    diff = std::fmod(diff, std::numbers::pi);
                    if (diff < -std::numbers::pi / 2) {
                        diff += std::numbers::pi;
                    } else if (diff > std::numbers::pi / 2) {
                        diff -= std::numbers::pi;
                    }
                    return diff;
                }))
            - steering_status.velocity);

        *left_front_steering_control_torque_  = control_torques[0];
        *left_back_steering_control_torque_   = control_torques[1];
        *right_back_steering_control_torque_  = control_torques[2];
        *right_front_steering_control_torque_ = control_torques[3];
    }

    void update_wheel_torques(
        const SteeringStatus& steering_status, const Eigen::Vector3d& chassis_acceleration) {
        if (std::isnan(chassis_acceleration[0])) {
            *left_front_wheel_control_torque_  = 0.0;
            *left_back_wheel_control_torque_   = 0.0;
            *right_back_wheel_control_torque_  = 0.0;
            *right_front_wheel_control_torque_ = 0.0;
            return;
        }

        const auto& [ax, ay, az] = chassis_acceleration;
        Eigen::Vector4d wheel_torques =
            wheel_radius_
            * (ax * mess_ * steering_status.cos_angle.array()
               + ay * mess_ * steering_status.sin_angle.array()
               + az * moment_of_inertia_
                     * (cos_varphi_.array() * steering_status.sin_angle.array()
                        - sin_varphi_.array() * steering_status.cos_angle.array())
                     / vehicle_radius_)
            / 4.0;

        *left_front_wheel_control_torque_  = wheel_torques[0];
        *left_back_wheel_control_torque_   = wheel_torques[1];
        *right_back_wheel_control_torque_  = wheel_torques[2];
        *right_front_wheel_control_torque_ = wheel_torques[3];
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double inf_ = std::numeric_limits<double>::infinity();

    static constexpr double mess_                = 22.0;
    static constexpr float moment_of_inertia_    = 5.0;
    static constexpr float vehicle_radius_       = 0.2 * std::numbers::sqrt2;
    static constexpr float wheel_radius_         = 0.055;
    static constexpr float friction_coefficient_ = 0.5;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;

    InputInterface<double> left_front_steering_angle_;
    InputInterface<double> left_back_steering_angle_;
    InputInterface<double> right_back_steering_angle_;
    InputInterface<double> right_front_steering_angle_;

    InputInterface<double> left_front_steering_velocity_;
    InputInterface<double> left_back_steering_velocity_;
    InputInterface<double> right_back_steering_velocity_;
    InputInterface<double> right_front_steering_velocity_;

    InputInterface<double> left_front_wheel_velocity_;
    InputInterface<double> left_back_wheel_velocity_;
    InputInterface<double> right_back_wheel_velocity_;
    InputInterface<double> right_front_wheel_velocity_;

    InputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;

    OutputInterface<double> left_front_steering_control_torque_;
    OutputInterface<double> left_back_steering_control_torque_;
    OutputInterface<double> right_back_steering_control_torque_;
    OutputInterface<double> right_front_steering_control_torque_;

    OutputInterface<double> left_front_wheel_control_torque_;
    OutputInterface<double> left_back_wheel_control_torque_;
    OutputInterface<double> right_back_wheel_control_torque_;
    OutputInterface<double> right_front_wheel_control_torque_;

    pid::MatrixPidCalculator<2> chassis_translational_velocity_pid_;
    pid::PidCalculator chassis_angular_velocity_pid_;

    const Eigen::Vector4d cos_varphi_, sin_varphi_;

    pid::MatrixPidCalculator<4> steering_velocity_pid_, steering_angle_pid_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::SteeringWheelController, rmcs_executor::Component)