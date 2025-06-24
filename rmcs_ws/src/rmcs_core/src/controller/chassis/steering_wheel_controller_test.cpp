#include <cmath>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Array.h>
#include <eigen3/Eigen/src/Core/Matrix.h>

#include "rmcs_utility/eigen_structured_bindings.hpp"
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

#include <controller/pid/matrix_pid_calculator.hpp>

namespace rmcs_core::controller::chassis {

class SteeringWheelControllerTest
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    explicit SteeringWheelControllerTest()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , steering_angle_pid_(0, 0, 0)
        , wheel_velocity_pid_(0, 0, 0) {
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
    };

    void update() override {
        if (std::isnan(chassis_control_velocity_->vector[0])
            || std::isnan(chassis_control_velocity_->vector[1])
            || std::isnan(chassis_control_velocity_->vector[2])) {
            reset_all_controls();
        }

        // auto chassis_control_velocity = calculate_chassis_control_velocity(); // 获取底盘速度
        auto chassis_status_expected = calculate_chassis_status_expected();

        auto wheel_velocities = calculate_wheel_velocities();
        auto steering_status  = calculate_steering_status();

        update_steering_control_torques(steering_status, chassis_status_expected);
        update_wheel_torques(wheel_velocities, chassis_status_expected);
    }

private:
    struct SteeringStatus {
        Eigen::Vector4d angle, cos_angle, sin_angle;
        Eigen::Vector4d velocity;
    };

    struct ChassisStatus {
        Eigen::Vector3d velocity; //(x,y,w)
        // SteeringStatus steering_status;

        // Eigen::Vector4d wheel_velocity;   //(LF, LB, RB, RF)
        Eigen::Vector4d wheel_frame_velocity_x; //(LF, LB, RB, RF)
        Eigen::Vector4d wheel_frame_velocity_y; // velocity in the Y direction of the wheel frame
    };

    struct WheelProjection {
        Eigen::Vector4d vehicle_radius_x;
        Eigen::Vector4d vehicle_radius_y;

        WheelProjection() {
            vehicle_radius_x =
                vehicle_radius_ * std::cos(vehicle_radius_angle_x) * Eigen::Vector4d(1, -1, -1, 1);
            vehicle_radius_y =
                vehicle_radius_ * std::sin(vehicle_radius_angle_x) * Eigen::Vector4d(1, 1, -1, -1);
        }
    } wheel_projection_;

    void reset_all_controls() {
        chassis_velocity_expected_.vector = Eigen::Vector3d::Zero();

        *left_front_steering_control_torque_  = 0.0;
        *left_back_steering_control_torque_   = 0.0;
        *right_back_steering_control_torque_  = 0.0;
        *right_front_steering_control_torque_ = 0.0;

        *left_front_wheel_control_torque_  = 0.0;
        *left_back_wheel_control_torque_   = 0.0;
        *right_back_wheel_control_torque_  = 0.0;
        *right_front_wheel_control_torque_ = 0.0;
    }

    Eigen::Vector3d calculate_chassis_control_velocity() {
        Eigen::Vector3d chassis_control_velocity =
            fast_tf::cast<rmcs_description::BaseLink>(*chassis_control_velocity_, *tf_).vector;
        return chassis_control_velocity;
    }

    ChassisStatus calculate_chassis_status_expected() {
        ChassisStatus chassis_status_expected;

        const auto& [vx, vy, vz] = chassis_status_expected.velocity;

        chassis_status_expected.velocity =
            fast_tf::cast<rmcs_description::BaseLink>(chassis_velocity_expected_, *tf_).vector;

        chassis_status_expected.wheel_frame_velocity_x =
            vx - vz * wheel_projection_.vehicle_radius_x.array();
        chassis_status_expected.wheel_frame_velocity_y =
            vy + vz * wheel_projection_.vehicle_radius_y.array();

        return chassis_status_expected;
    }

    void update_steering_control_torques(
        const SteeringStatus& steering_status, const ChassisStatus& chassis_status_expected) {
        const auto& [vx, vy, vz] = chassis_status_expected.velocity;

        Eigen::Array4d steering_control_angles_x =
            vx - vz * wheel_projection_.vehicle_radius_y.array();
        Eigen::Array4d steering_control_angles_y =
            vy + vz * wheel_projection_.vehicle_radius_x.array();
        Eigen::Vector4d steering_control_angles = steering_control_angles_x.binaryExpr(
            steering_control_angles_y, [](double x, double y) { return std::atan2(y, x); });

        Eigen::Vector4d steering_control_torques = steering_angle_pid_.update(
            (steering_control_angles - steering_status.angle).unaryExpr([](double diff) {
                diff = std::fmod(diff, std::numbers::pi);
                if (diff < -std::numbers::pi / 2) {
                    diff += std::numbers::pi;
                } else if (diff > std::numbers::pi / 2) {
                    diff -= std::numbers::pi;
                }
                return diff;
            }));

        *left_front_steering_control_torque_  = steering_control_torques[0];
        *left_back_steering_control_torque_   = steering_control_torques[1];
        *right_back_steering_control_torque_  = steering_control_torques[2];
        *right_front_steering_control_torque_ = steering_control_torques[3];
    }

    void update_wheel_torques(
        const Eigen::Vector4d& wheel_velocities, const ChassisStatus& chassis_status_expected) {
        const Eigen::Vector4d& wheel_velocities_expected =
            (chassis_status_expected.wheel_frame_velocity_x.array().square()
             + chassis_status_expected.wheel_frame_velocity_y.array().square())
                .sqrt()
            / wheel_radius_;

        Eigen::Vector4d wheel_torques =
            wheel_velocity_pid_.update(wheel_velocities_expected - wheel_velocities);

        *left_front_wheel_control_torque_  = wheel_torques[0];
        *left_back_wheel_control_torque_   = wheel_torques[1];
        *right_back_wheel_control_torque_  = wheel_torques[2];
        *right_front_wheel_control_torque_ = wheel_torques[3];
    }

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

        steering_status.angle.array() -= 0.0;
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

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double inf_ = std::numeric_limits<double>::infinity();

    // static constexpr double mess_                 = 22.0;
    // static constexpr double moment_of_inertia_    = 4.0;
    // static constexpr double vehicle_radius_       = 0.2 * std::numbers::sqrt2;
    // static constexpr double wheel_radius_         = 0.055;
    // static constexpr double friction_coefficient_ = 0.6;

    static constexpr double mess_                 = 19.0;
    static constexpr double moment_of_inertia_    = 2.0;
    static constexpr double vehicle_radius_       = 0.24678;
    static constexpr double wheel_radius_         = 0.055;
    static constexpr double friction_coefficient_ = 0.6;
    static constexpr double vehicle_radius_angle_x =
        std::numbers::pi / 4; // Angle between wheel[0]'s radius and the X-axis

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

    InputInterface<rmcs_description::Tf> tf_;
    InputInterface<rmcs_description::YawLink::DirectionVector> chassis_control_velocity_;
    InputInterface<double> power_limit_;

    OutputInterface<double> left_front_steering_control_torque_;
    OutputInterface<double> left_back_steering_control_torque_;
    OutputInterface<double> right_back_steering_control_torque_;
    OutputInterface<double> right_front_steering_control_torque_;

    OutputInterface<double> left_front_wheel_control_torque_;
    OutputInterface<double> left_back_wheel_control_torque_;
    OutputInterface<double> right_back_wheel_control_torque_;
    OutputInterface<double> right_front_wheel_control_torque_;

    rmcs_description::YawLink::DirectionVector chassis_velocity_expected_;

    pid::MatrixPidCalculator<4> steering_angle_pid_, wheel_velocity_pid_;
};
} // namespace rmcs_core::controller::chassis
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::SteeringWheelControllerTest, rmcs_executor::Component)