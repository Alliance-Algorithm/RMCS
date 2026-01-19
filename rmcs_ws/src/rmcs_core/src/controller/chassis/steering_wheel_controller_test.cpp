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

#include "filter/low_pass_filter.hpp"

namespace rmcs_core::controller::chassis {

class SteeringWheelTest
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    explicit SteeringWheelTest()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , cos_varphi_(1, 0, -1, 0)
        , sin_varphi_(0, 1, 0, -1)           // 角度分量，用于速度分解
        , steering_velocity_pid_(0.15, 0, 0.1)
        , steering_angle_pid_(30, 0, 0)
        , wheel_velocity_pid_(0.09, 0, 0.02) // pid控制设置
        , steering_angle_filter_(5.0, 1000.0) {

        auto steering_velocity_pid =
            get_parameter("steering_velocity_pid_parameters").as_double_array();
        auto steering_angle_pid = get_parameter("steering_angle_pid_parameters").as_double_array();
        auto wheel_velocity_pid = get_parameter("wheel_velocity_pid_parameters").as_double_array();

        steering_angle_pid_.kp = steering_angle_pid[0];
        steering_angle_pid_.ki = steering_angle_pid[1];
        steering_angle_pid_.kd = steering_angle_pid[2];

        steering_velocity_pid_.kp = steering_velocity_pid[0];
        steering_velocity_pid_.ki = steering_velocity_pid[1];
        steering_velocity_pid_.kd = steering_velocity_pid[2];

        wheel_velocity_pid_.kp = wheel_velocity_pid[0];
        wheel_velocity_pid_.ki = wheel_velocity_pid[1];
        wheel_velocity_pid_.kd = wheel_velocity_pid[2];

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

        register_input("/tf", tf_);

        register_input("/chassis/control_velocity", chassis_control_velocity_);

        // register_input("/chassis/control_power_limit", chassis_control_power_limit_);
        // register_input("/chassis/power", chassis_power_);

        register_output(
            "/chassis/left_front_steering/control_torque_unrestricted",
            left_front_steering_control_torque_unrestricted_);
        register_output(
            "/chassis/left_back_steering/control_torque_unrestricted",
            left_back_steering_control_torque_unrestricted_);
        register_output(
            "/chassis/right_back_steering/control_torque_unrestricted",
            right_back_steering_control_torque_unrestricted_);
        register_output(
            "/chassis/right_front_steering/control_torque_unrestricted",
            right_front_steering_control_torque_unrestricted_);

        register_output(
            "/chassis/left_front_wheel/control_torque_unrestricted",
            left_front_wheel_control_torque_unrestricted_);
        register_output(
            "/chassis/left_back_wheel/control_torque_unrestricted",
            left_back_wheel_control_torque_unrestricted_);
        register_output(
            "/chassis/right_back_wheel/control_torque_unrestricted",
            right_back_wheel_control_torque_unrestricted_);
        register_output(
            "/chassis/right_front_wheel/control_torque_unrestricted",
            right_front_wheel_control_torque_unrestricted_);

        register_output(
            "/chassis/left_front_steering/angle_error", left_front_steering_angle_error_);
        register_output("/chassis/left_back_steering/angle_error", left_back_steering_angle_error_);
        register_output(
            "/chassis/right_back_steering/angle_error", right_back_steering_angle_error_);
        register_output(
            "/chassis/right_front_steering/angle_error", right_front_steering_angle_error_);
    };

    void update() override {
        if (std::isnan(chassis_control_velocity_->vector[0])
            || std::isnan(chassis_control_velocity_->vector[1])
            || std::isnan(chassis_control_velocity_->vector[2])) {
            reset_all_controls();
        } // 速度向量有三个--x，y，角速度

        auto chassis_status_expected = calculate_chassis_status_expected(); // 轮组

        auto wheel_velocities = calculate_wheel_velocities();
        auto steering_status = calculate_steering_status();

        update_steering_control_torques(steering_status, chassis_status_expected);
        update_wheel_torques(wheel_velocities, chassis_status_expected, steering_status);
    }

private:
    struct SteeringStatus {
        Eigen::Vector4d angle, cos_angle, sin_angle;
        Eigen::Vector4d velocity;
    };

    /**
     * @brief Chassiss current status
     * velocity:(v_x,v_y,w)
     * wheel_frame_velocity: velocity of the wheel frame,(LF, LB, RB, RF)
     */
    struct ChassisStatus {
        Eigen::Vector3d velocity;

        Eigen::Vector4d wheel_frame_velocity_x;
        Eigen::Vector4d wheel_frame_velocity_y;
    };

    void reset_all_controls() {
        chassis_velocity_expected_.vector = Eigen::Vector3d::Zero();

        *left_front_steering_control_torque_unrestricted_ = nan_;
        *left_back_steering_control_torque_unrestricted_ = nan_;
        *right_back_steering_control_torque_unrestricted_ = nan_;
        *right_front_steering_control_torque_unrestricted_ = nan_;

        *left_front_wheel_control_torque_unrestricted_ = nan_;
        *left_back_wheel_control_torque_unrestricted_ = nan_;
        *right_back_wheel_control_torque_unrestricted_ = nan_;
        *right_front_wheel_control_torque_unrestricted_ = nan_;
    }

    /**
     * @brief Calculates the expected chassis status, including velocity and wheel-frame velocities.
     * @details The `chassis_control_velocity_` is transformed into the coordinate frame of the
     * robot's base link (`rmcs_description::BaseLink`).The chassis coordinate system is a
     * right-handed system, with the x-axis pointing from the center of the chassis to a wheel.
     * @return ChassisStatus An object containing the calculated expected chassis velocity(in the
     * base link frame after rotation) and the individual wheel-frame velocities.
     */

    ChassisStatus calculate_chassis_status_expected() {
        ChassisStatus chassis_status_expected;

        chassis_status_expected.velocity =
            fast_tf::cast<rmcs_description::BaseLink>(*chassis_control_velocity_, *tf_).vector;

        chassis_status_expected.velocity.head<2>() =
            Eigen::Rotation2Dd(-std::numbers::pi / 4) * chassis_status_expected.velocity.head<2>();

        const auto& [vx, vy, vz] = chassis_status_expected.velocity;
        chassis_status_expected.wheel_frame_velocity_x =
            vx - vz * vehicle_radius_ * sin_varphi_.array();
        chassis_status_expected.wheel_frame_velocity_y =
            vy + vz * vehicle_radius_ * cos_varphi_.array(); // 坐标变化--从底盘到轮组

        if (chassis_status_expected.velocity.norm() < 1e-3) {
            chassis_status_expected.velocity.setZero();
            chassis_status_expected.wheel_frame_velocity_x.setZero();
            chassis_status_expected.wheel_frame_velocity_y.setZero();
        }
        return chassis_status_expected;
    }

    Eigen::Vector4d static calculate_steering_angles_expected(
        const ChassisStatus& chassis_status_expected) {
        const Eigen::Array4d steering_angles_expected_x =
            chassis_status_expected.wheel_frame_velocity_x;
        const Eigen::Array4d steering_angles_expected_y =
            chassis_status_expected.wheel_frame_velocity_y;

        Eigen::Vector4d steering_angles_expected = steering_angles_expected_x.binaryExpr(
            steering_angles_expected_y, [](double x, double y) { return std::atan2(y, x); });

        if (chassis_status_expected.velocity.norm() < 1e-2) {
            steering_angles_expected[0] = std::numbers::pi / 2;
            steering_angles_expected[2] = std::numbers::pi / 2;
            steering_angles_expected[1] = 0;
            steering_angles_expected[3] = 0;
        };

        return steering_angles_expected;
    }

    void update_steering_control_torques(
        const SteeringStatus& steering_status, const ChassisStatus& chassis_status_expected) {

        const auto& steering_angles_expected =
            calculate_steering_angles_expected(chassis_status_expected);

        Eigen::Vector4d angle_error =
            (steering_angles_expected - steering_status.angle).unaryExpr([](double diff) {
                diff = std::fmod(diff, std::numbers::pi);
                if (diff < -std::numbers::pi / 2) {
                    diff += std::numbers::pi;
                } else if (diff > std::numbers::pi / 2) {
                    diff -= std::numbers::pi;
                }
                return diff;                                 // 舵向电机就近转位
            });

        *left_front_steering_angle_error_ = angle_error[0];
        *left_back_steering_angle_error_ = angle_error[1];
        *right_back_steering_angle_error_ = angle_error[2];
        *right_front_steering_angle_error_ = angle_error[3];

        Eigen::Vector4d steering_velocity_expected =
            steering_angle_pid_.update(steering_angle_filter_.update(angle_error));

        Eigen::Vector4d velocity_error = steering_velocity_expected - steering_status.velocity;

        Eigen::Vector4d steering_control_torques = steering_velocity_pid_.update(velocity_error);

        *left_front_steering_control_torque_unrestricted_ = steering_control_torques[0];
        *left_back_steering_control_torque_unrestricted_ = steering_control_torques[1];
        *right_back_steering_control_torque_unrestricted_ = steering_control_torques[2];
        *right_front_steering_control_torque_unrestricted_ = steering_control_torques[3];
    }

    void update_wheel_torques(
        const Eigen::Vector4d& wheel_velocities, const ChassisStatus& chassis_status_expected,
        const SteeringStatus& steering_status) {
        const Eigen::Vector4d& wheel_velocities_expected =
            (chassis_status_expected.wheel_frame_velocity_x.array()
                 * steering_status.cos_angle.array()
             + chassis_status_expected.wheel_frame_velocity_y.array()
                   * steering_status.sin_angle.array())
            / wheel_radius_;

        *left_front_wheel_control_velocity_ = wheel_velocities_expected[0];
        *left_back_wheel_control_velocity_ = wheel_velocities_expected[1];
        *right_back_wheel_control_velocity_ = wheel_velocities_expected[2];
        *right_front_wheel_control_velocity_ = wheel_velocities_expected[3];

        Eigen::Vector4d wheel_torques =
            wheel_velocity_pid_.update(wheel_velocities_expected - wheel_velocities);

        *left_front_wheel_control_torque_unrestricted_ = wheel_torques[0];
        *left_back_wheel_control_torque_unrestricted_ = wheel_torques[1];
        *right_back_wheel_control_torque_unrestricted_ = wheel_torques[2];
        *right_front_wheel_control_torque_unrestricted_ = wheel_torques[3];
    }

    Eigen::Vector4d calculate_wheel_velocities() {
        return {
            *left_front_wheel_velocity_,                     //
            *left_back_wheel_velocity_,                      //
            *right_back_wheel_velocity_,                     //
            *right_front_wheel_velocity_                     //
        };
    }

    SteeringStatus calculate_steering_status() {
        SteeringStatus steering_status;

        steering_status.angle = {
            *left_front_steering_angle_,                     //
            *left_back_steering_angle_,                      //
            *right_back_steering_angle_,                     //
            *right_front_steering_angle_                     //
        };

        steering_status.angle.array() -= std::numbers::pi / 4;
        ;
        steering_status.cos_angle = steering_status.angle.array().cos();
        steering_status.sin_angle = steering_status.angle.array().sin();

        steering_status.velocity = {
            *left_front_steering_velocity_,                  //
            *left_back_steering_velocity_,                   //
            *right_back_steering_velocity_,                  //
            *right_front_steering_velocity_                  //
        };

        return steering_status;
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double inf_ = std::numeric_limits<double>::infinity();
    static constexpr double sqrt_2_ = std::numbers::sqrt2;

    static constexpr double mess_ = 19.0;
    static constexpr double moment_of_inertia_ = 2.0;
    static constexpr double vehicle_radius_ = 0.24678;
    static constexpr double wheel_radius_ = 0.055;
    static constexpr double friction_coefficient_ = 0.6;

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
    InputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;

    InputInterface<double> chassis_control_power_limit_;
    InputInterface<double> chassis_power_;

    OutputInterface<double> left_front_steering_control_torque_unrestricted_;
    OutputInterface<double> left_back_steering_control_torque_unrestricted_;
    OutputInterface<double> right_back_steering_control_torque_unrestricted_;
    OutputInterface<double> right_front_steering_control_torque_unrestricted_;

    OutputInterface<double> left_front_wheel_control_torque_unrestricted_;
    OutputInterface<double> left_back_wheel_control_torque_unrestricted_;
    OutputInterface<double> right_back_wheel_control_torque_unrestricted_;
    OutputInterface<double> right_front_wheel_control_torque_unrestricted_;

    OutputInterface<double> left_front_steering_angle_error_;
    OutputInterface<double> left_back_steering_angle_error_;
    OutputInterface<double> right_back_steering_angle_error_;
    OutputInterface<double> right_front_steering_angle_error_;

    OutputInterface<double> left_front_wheel_control_velocity_;
    OutputInterface<double> left_back_wheel_control_velocity_;
    OutputInterface<double> right_back_wheel_control_velocity_;
    OutputInterface<double> right_front_wheel_control_velocity_;

    rmcs_description::YawLink::DirectionVector chassis_velocity_expected_;

    const Eigen::Vector4d cos_varphi_, sin_varphi_;

    pid::MatrixPidCalculator<4> steering_velocity_pid_, steering_angle_pid_, wheel_velocity_pid_;

    filter::LowPassFilter<4> steering_angle_filter_;
};
} // namespace rmcs_core::controller::chassis
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::SteeringWheelTest, rmcs_executor::Component)