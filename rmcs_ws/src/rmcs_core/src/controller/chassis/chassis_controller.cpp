#include "controller/pid/pid_calculator.hpp"
#include "hardware/device/trajectory.hpp"
#include "rmcs_msgs/arm_mode.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <numbers>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/arm_mode.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/leg_mode.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>
namespace rmcs_core::controller::chassis {
class Chassis_Controller
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Chassis_Controller()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , following_velocity_controller_(1.0, 0.0, 0.0) {
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        register_output("/chassis_and_leg/enable_flag", is_chassis_and_leg_enable, true);\
        register_input("yaw_imu_angle", yaw_imu_angle);
        register_input("/arm/Joint1/theta", joint1_theta);

        register_input("/arm/mode", arm_mode);

        register_output(
            "/chassis/big_yaw/target_angle_error", chassis_big_yaw_target_angle_error, NAN);
        register_input("/chassis/big_yaw/angle", chassis_big_yaw_angle);
        register_output(
            "/chassis/control_power_limit", chassis_control_power_limit_, 0.0); 
        register_output("/chassis/control_velocity", chassis_control_velocity_);  
        register_input("/chassis/power", chassis_power_);
        register_input("/referee/chassis/buffer_energy", chassis_buffer_energy_referee_);
    }
    void update() override {
        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto mouse        = *mouse_;
        auto keyboard     = *keyboard_;
        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            *is_chassis_and_leg_enable = false;
            reset_motor();
            yaw_control_theta_in_IMU = *yaw_imu_angle;
            is_yaw_imu_control       = true;
        } else {
            mode_selection();

            *is_chassis_and_leg_enable = true;
            Eigen::Vector2d move_;
            double angular_velocity = 0.0;
            switch (chassis_mode) {
            case rmcs_msgs::ChassisMode::Flow: {
                double chassis_theta = *chassis_big_yaw_angle;
                angular_velocity =
                    std::clamp(following_velocity_controller_.update(chassis_theta), -1.0, 1.0);
                break;
            }
            case rmcs_msgs::ChassisMode::SPIN: {
                angular_velocity = 5;
                yaw_control_theta_in_IMU += joystick_right_->y() * 0.002;
                break;
            }
            case rmcs_msgs::ChassisMode::Up_Stairs: {
                is_yaw_imu_control           = false;
                yaw_set_theta_in_YawFreeMode = 0.0;
                speed_limit                  = 1.5;
                move_                        = *joystick_left_;
                break;
            }
            default: break;
            }
            Eigen::Rotation2D<double> rotation(*chassis_big_yaw_angle + *joint1_theta);
            move_ = rotation * (*joystick_left_);
            chassis_control_velocity_->vector << (move_ * speed_limit), angular_velocity;
            yaw_control();
            // power control
        update_virtual_buffer_energy();
        update_control_power_limit();
        }
    }

private:
    void mode_selection() {
        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto mouse        = *mouse_;
        auto keyboard     = *keyboard_;
        using namespace rmcs_msgs;
        if (switch_left == Switch::MIDDLE && switch_right == Switch::MIDDLE) {
            chassis_mode       = ChassisMode::Flow;
            is_yaw_imu_control = true;
        } else if (switch_left == Switch::MIDDLE && switch_right == Switch::DOWN) {
            chassis_mode       = ChassisMode::Flow;
            is_yaw_imu_control = true;
        } else if (switch_left == Switch::MIDDLE && switch_right == Switch::UP) {
            chassis_mode       = ChassisMode::SPIN;
            is_yaw_imu_control = true;
        } else if (switch_left == Switch::DOWN && switch_right == Switch::UP) {
            if (keyboard.c) {
                if (!keyboard.shift && !keyboard.ctrl) {
                    speed_limit = 4.5;
                }
                if (keyboard.shift && !keyboard.ctrl) {
                    speed_limit = 2.5;
                }
                if (!keyboard.shift && keyboard.ctrl) {
                    speed_limit = 0.8;
                }
            }
            if (keyboard.q) {
                if (!keyboard.shift && !keyboard.ctrl) {
                    chassis_mode       = rmcs_msgs::ChassisMode::Flow;
                    is_yaw_imu_control = true;
                }
            }
            if (keyboard.d) {
                chassis_mode       = rmcs_msgs::ChassisMode::SPIN;
                is_yaw_imu_control = true;
            }
            if (keyboard.b) {
                chassis_mode = rmcs_msgs::ChassisMode::Up_Stairs;
            }
            // execute right change according to arm_mode
            if (last_arm_mode != *arm_mode) {

                if (*arm_mode == rmcs_msgs::ArmMode::Auto_Gold_Left
                    || *arm_mode == rmcs_msgs::ArmMode::Auto_Gold_Mid
                    || *arm_mode == rmcs_msgs::ArmMode::Auto_Gold_Right
                    || *arm_mode == rmcs_msgs::ArmMode::Auto_Sliver
                    || *arm_mode == rmcs_msgs::ArmMode::Auto_Ground
                    || *arm_mode == rmcs_msgs::ArmMode::Auto_Storage_LB
                    || *arm_mode == rmcs_msgs::ArmMode::Auto_Storage_RB
                    || *arm_mode == rmcs_msgs::ArmMode::Auto_Extract
                    || *arm_mode == rmcs_msgs::ArmMode::Customer
                    || *arm_mode == rmcs_msgs::ArmMode::Auto_Up_Stairs) {
                    speed_limit        = 1.6;
                    is_yaw_imu_control = false;
                    if (*arm_mode == rmcs_msgs::ArmMode::Customer) {
                        chassis_mode                 = rmcs_msgs::ChassisMode::Yaw_Free;
                        yaw_set_theta_in_YawFreeMode = 0.0;
                    } else {
                        chassis_mode = ChassisMode::Yaw_Free;
                        if (*arm_mode == rmcs_msgs::ArmMode::Auto_Gold_Left) {
                            yaw_set_theta_in_YawFreeMode = std::numbers::pi / 2.0;
                        } else if (*arm_mode == rmcs_msgs::ArmMode::Auto_Gold_Right) {
                            yaw_set_theta_in_YawFreeMode = -std::numbers::pi / 2.0;
                        } else if (*arm_mode == rmcs_msgs::ArmMode::Auto_Gold_Mid) {
                            yaw_set_theta_in_YawFreeMode = 0.0;

                        } else if (
                            *arm_mode == rmcs_msgs::ArmMode::Auto_Storage_LB
                            || *arm_mode == rmcs_msgs::ArmMode::Auto_Storage_RB) {
                            if (last_arm_mode == rmcs_msgs::ArmMode::Auto_Gold_Left) {
                                yaw_set_theta_in_YawFreeMode = std::numbers::pi / 2.0;
                            } else if (last_arm_mode == rmcs_msgs::ArmMode::Auto_Gold_Right) {
                                yaw_set_theta_in_YawFreeMode = -std::numbers::pi / 2.0;
                            } else {
                                yaw_set_theta_in_YawFreeMode = 0.0;
                            }

                        } else {
                            yaw_set_theta_in_YawFreeMode = 0.0;
                        }
                    }
                    yaw_trajectory_controller.set_start_point({*chassis_big_yaw_angle})
                        .set_total_step(1400)
                        .set_end_point({yaw_set_theta_in_YawFreeMode})
                        .reset();
                } else if (*arm_mode == rmcs_msgs::ArmMode::Auto_Walk) {
                    speed_limit        = 4.5;
                    chassis_mode       = rmcs_msgs::ChassisMode::Flow;
                    is_yaw_imu_control = true;
                }
            }
        } else {
            chassis_mode = rmcs_msgs::ChassisMode::None;
        }
        if (last_chassis_mode != chassis_mode) {
            if (last_chassis_mode == rmcs_msgs::ChassisMode::SPIN) {
                speed_limit = 4.5;
            }
            if (last_chassis_mode == rmcs_msgs::ChassisMode::Yaw_Free
                || last_chassis_mode == rmcs_msgs::ChassisMode::Up_Stairs) {
                if (last_is_yaw_imu_control != is_yaw_imu_control) {
                    yaw_control_theta_in_IMU = *yaw_imu_angle;
                }
            }
        }
        last_chassis_mode       = chassis_mode;
        last_arm_mode           = *arm_mode;
        last_is_yaw_imu_control = is_yaw_imu_control;
    }

    void yaw_control() {
        if (chassis_mode != rmcs_msgs::ChassisMode::Up_Stairs) {
            yaw_control_theta_in_IMU += joystick_right_->y() * 0.002;
        }
        if (is_yaw_imu_control) {
            *chassis_big_yaw_target_angle_error =
                normalizeAngle(yaw_control_theta_in_IMU - *yaw_imu_angle);
        } else {
            std::array<double, 6> result = yaw_trajectory_controller.trajectory();
            *chassis_big_yaw_target_angle_error =
                normalizeAngle(result[0] - *chassis_big_yaw_angle);
        }
    }
    void update_virtual_buffer_energy() {
        constexpr double dt = 1e-3;
        virtual_buffer_energy_ += dt * (chassis_power_limit_expected_ - *chassis_power_);
        virtual_buffer_energy_ = std::clamp(
            virtual_buffer_energy_, 0.0,
            std::min(*chassis_buffer_energy_referee_, virtual_buffer_energy_limit_));
    }

    void update_control_power_limit() {
        double power_limit;

            power_limit = chassis_power_limit_referee_;
        chassis_power_limit_expected_ = power_limit;

        constexpr double excess_power_limit = 15;

        power_limit += excess_power_limit;
        power_limit *= virtual_buffer_energy_ / virtual_buffer_energy_limit_;

        *chassis_control_power_limit_ = power_limit;//
    }

    void reset_motor() {
        *chassis_big_yaw_target_angle_error = NAN;
        *chassis_control_velocity_          = {nan, nan, nan};
        *chassis_control_power_limit_       = 0.0;
        virtual_buffer_energy_        = virtual_buffer_energy_limit_;//
    }
    static double normalizeAngle(double angle) {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }

    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    InputInterface<rmcs_msgs::ArmMode> arm_mode;
    rmcs_msgs::ArmMode last_arm_mode;
    pid::PidCalculator following_velocity_controller_;

    double speed_limit              = 4.5;  // m/s
    constexpr static double chassis_power_limit_referee_ = 120.0f;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    OutputInterface<bool> is_chassis_and_leg_enable;
    // —————————————————————————leg————————————————————————————————
    InputInterface<double> theta_lf;
    InputInterface<double> theta_lb;
    InputInterface<double> theta_rb;
    InputInterface<double> theta_rf;

    bool is_yaw_imu_control      = false;
    bool last_is_yaw_imu_control = false;
    InputInterface<double> yaw_imu_angle;
    InputInterface<double> joint1_theta;
    double yaw_control_theta_in_IMU     = NAN;
    double yaw_set_theta_in_YawFreeMode = NAN;
    hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT> yaw_trajectory_controller;

    OutputInterface<double> chassis_big_yaw_target_angle_error;
    InputInterface<double> chassis_big_yaw_angle;
    rmcs_msgs::ChassisMode chassis_mode      = rmcs_msgs::ChassisMode::None;
    rmcs_msgs::ChassisMode last_chassis_mode = rmcs_msgs::ChassisMode::None;

    OutputInterface<double> chassis_control_power_limit_;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;
    double virtual_buffer_energy_;
    static constexpr double virtual_buffer_energy_limit_ = 30.0;
    double chassis_power_limit_expected_;
    InputInterface<double> chassis_power_;
    InputInterface<double> chassis_buffer_energy_referee_;
};
} // namespace rmcs_core::controller::chassis
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::Chassis_Controller, rmcs_executor::Component)
