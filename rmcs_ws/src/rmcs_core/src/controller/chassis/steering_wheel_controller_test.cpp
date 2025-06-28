#include <chassis_mode.hpp>
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
        , cos_varphi_(sqrt_2_ / 2.0, -sqrt_2_ / 2.0, -sqrt_2_ / 2.0, sqrt_2_ / 2.0)
        , sin_varphi_(sqrt_2_ / 2, sqrt_2_ / 2, -sqrt_2_ / 2, -sqrt_2_ / 2)
        , steering_velocity_pid_(0, 0, 0)
        , steering_angle_pid_(0, 0, 0)
        , wheel_velocity_pid_(0, 0, 0) {

        // auto steering_velocity_pid =
        // get_parameter("steering_velocity_pid_parameters").as_double_array();
        auto steering_angle_pid = get_parameter("steering_angle_pid_parameters").as_double_array();
        auto wheel_velocity_pid = get_parameter("wheel_velocity_pid_parameters").as_double_array();

        steering_angle_pid_.kp = steering_angle_pid[0];
        steering_angle_pid_.ki = steering_angle_pid[1];
        steering_angle_pid_.kd = steering_angle_pid[2];

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
        register_input("/chassis/control_mode", mode_);

        register_input("/chassis/supercap/control_enable", supercap_control_enabled_);
        register_input("/chassis/supercap/charge_power_limit", supercap_charge_power_limit_);
        register_input("/chassis/control_power_limit", chassis_control_power_limit_);
        register_input("/chassis/supercap/voltage/dead_line", supercap_voltage_dead_line_);
        register_input("/chassis/supercap/voltage/control_line", supercap_voltage_control_line_);
        register_input("/chassis/supercap/voltage/base_line", supercap_voltage_base_line_);

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
        auto chassis_status_expected = calculate_chassis_status_expected(); // 轮组

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
        Eigen::Vector3d velocity;                                           //(x,y,w)

        Eigen::Vector4d wheel_frame_velocity_x;                             //(LF, LB, RB, RF)
        Eigen::Vector4d wheel_frame_velocity_y; // velocity in the Y direction of the wheel frame
    };

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

    // Eigen::Vector3d calculate_chassis_control_velocity() {
    //     Eigen::Vector3d chassis_control_velocity =
    //         fast_tf::cast<rmcs_description::BaseLink>(*chassis_control_velocity_, *tf_).vector;
    //     return chassis_control_velocity;
    // }

    ChassisStatus calculate_chassis_status_expected() {
        ChassisStatus chassis_status_expected;

        chassis_status_expected.velocity =
            fast_tf::cast<rmcs_description::BaseLink>(*chassis_control_velocity_, *tf_).vector;

        const auto& [vx, vy, vz] = chassis_status_expected.velocity;
        chassis_status_expected.wheel_frame_velocity_x =
            vx - vz * vehicle_radius_ * sin_varphi_.array();
        chassis_status_expected.wheel_frame_velocity_y =
            vy + vz * vehicle_radius_ * cos_varphi_.array();

        if (chassis_status_expected.velocity.norm() < 1e-1) {
            chassis_status_expected.velocity.setZero();
            chassis_status_expected.wheel_frame_velocity_x.setZero();
            chassis_status_expected.wheel_frame_velocity_y.setZero();
        }

        return chassis_status_expected;
    }

    void update_steering_control_torques(
        const SteeringStatus& steering_status, const ChassisStatus& chassis_status_expected) {

        const Eigen::Array4d steering_control_angles_x =
            chassis_status_expected.wheel_frame_velocity_x;
        const Eigen::Array4d steering_control_angles_y =
            chassis_status_expected.wheel_frame_velocity_y;

        Eigen::Vector4d steering_control_angles = steering_control_angles_x.binaryExpr(
            steering_control_angles_y, [](double x, double y) { return std::atan2(y, x); });

        Eigen::Vector4d steering_control_torques = steering_velocity_pid_.update(

            steering_angle_pid_.update(
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
            wheel_velocity_pid_.update(wheel_velocities - wheel_velocities_expected);

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

    static constexpr double nan_    = std::numeric_limits<double>::quiet_NaN();
    static constexpr double inf_    = std::numeric_limits<double>::infinity();
    static constexpr double sqrt_2_ = std::numbers::sqrt2;

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
    // InputInterface<double> power_limit_;

    InputInterface<rmcs_msgs::ChassisMode> mode_;
    InputInterface<bool> supercap_control_enabled_;

    InputInterface<double> supercap_charge_power_limit_;
    InputInterface<double> chassis_control_power_limit_;
    InputInterface<double> supercap_voltage_dead_line_;
    InputInterface<double> supercap_voltage_control_line_;
    InputInterface<double> supercap_voltage_base_line_;

    OutputInterface<double> left_front_steering_control_torque_;
    OutputInterface<double> left_back_steering_control_torque_;
    OutputInterface<double> right_back_steering_control_torque_;
    OutputInterface<double> right_front_steering_control_torque_;

    OutputInterface<double> left_front_wheel_control_torque_;
    OutputInterface<double> left_back_wheel_control_torque_;
    OutputInterface<double> right_back_wheel_control_torque_;
    OutputInterface<double> right_front_wheel_control_torque_;

    rmcs_description::YawLink::DirectionVector chassis_velocity_expected_;

    const Eigen::Vector4d cos_varphi_, sin_varphi_;

    pid::MatrixPidCalculator<4> steering_velocity_pid_, steering_angle_pid_, wheel_velocity_pid_;
};
} // namespace rmcs_core::controller::chassis
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::SteeringWheelControllerTest, rmcs_executor::Component)