#include "controller/pid/pid_calculator.hpp"
#include "hardware/device/trajectory.hpp"
#include <algorithm>
#include "rmcs_msgs/arm_mode.hpp"
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
class Chassisonly_Controller
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Chassisonly_Controller()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , following_velocity_controller_(0.46, 0.00, 0.089){

        register_input("/remote/joystick/right", joystick_right_); 
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        register_output("/chassis_and_leg/enable_flag", is_chassis_and_leg_enable, true);
        register_input("/steering/steering/lf/angle", steering_lf_angle);
        register_input("/steering/steering/lb/angle", steering_lb_angle);
        register_input("/steering/steering/rb/angle", steering_rb_angle);
        register_input("/steering/steering/rf/angle", steering_rf_angle);
        register_output(
            "/steering/steering/lf/target_angle_error", steering_lf_target_angle_error, NAN);
        register_output(
            "/steering/steering/lb/target_angle_error", steering_lb_target_angle_error, NAN);
        register_output(
            "/steering/steering/rb/target_angle_error", steering_rb_target_angle_error, NAN);
        register_output(
            "/steering/steering/rf/target_angle_error", steering_rf_target_angle_error, NAN);

        register_output("/steering/wheel/lf/target_vel", steering_wheel_lf_target_vel, NAN);
        register_output("/steering/wheel/lb/target_vel", steering_wheel_lb_target_vel, NAN);
        register_output("/steering/wheel/rb/target_vel", steering_wheel_rb_target_vel, NAN);
        register_output("/steering/wheel/rf/target_vel", steering_wheel_rf_target_vel, NAN);

        register_input("yaw_imu_velocity", yaw_imu_velocity);
        register_input("yaw_imu_angle", yaw_imu_angle);
        register_input("/arm/Joint1/theta", joint1_theta);

        register_input("/arm/mode", arm_mode);

        register_output(
            "/chassis/big_yaw/target_angle_error", chassis_big_yaw_target_angle_error, NAN);
        register_input("/chassis/big_yaw/angle", chassis_big_yaw_angle);
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
            yaw_control_theta                   = *yaw_imu_angle;
            is_yaw_imu_control                  = true;
        } else {
            *is_chassis_and_leg_enable = true;
            mode_selection();
            switch (chassis_mode) {
            case rmcs_msgs::ChassisMode::Flow: {
                double chassis_theta = *chassis_big_yaw_angle;
                double spin_speed =
                    std::clamp(following_velocity_controller_.update(chassis_theta), -1.0, 1.0);
                Eigen::Rotation2D<double> rotation(chassis_theta + *joint1_theta);
                Eigen::Vector2d move_ = *joystick_left_;
                yaw_control_theta += joystick_right_->y() * 0.002;
                yaw_control_theta = normalizeAngle(yaw_control_theta);
                steering_control(move_, spin_speed);
                break;
            }
            case rmcs_msgs::ChassisMode::SPIN: {
                double spin_speed = 0.8;
                speed_limit       = 5.0;
                Eigen::Rotation2D<double> rotation(*chassis_big_yaw_angle + *joint1_theta);
                Eigen::Vector2d move_ = rotation * (*joystick_left_);
                yaw_control_theta += joystick_right_->y() * 0.002;
                steering_control(move_, spin_speed);

                break;

            }
            case rmcs_msgs::ChassisMode::Up_Stairs: {
                is_yaw_imu_control           = false;
                yaw_set_theta_in_YawFreeMode = 0.0;
                speed_limit                  = 1.5;
                Eigen::Vector2d move_        = *joystick_left_;
                steering_control(move_, joystick_right_->y());
                break;
            }
            case rmcs_msgs::ChassisMode::Yaw_Free: {
                Eigen::Rotation2D<double> rotation(*chassis_big_yaw_angle + *joint1_theta);
                Eigen::Vector2d move_ = rotation * (*joystick_left_);
                steering_control(move_, joystick_right_->y() / 1.5);
                break;
            }
            default: break;
            }
            yaw_control();
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
                chassis_mode           = rmcs_msgs::ChassisMode::Up_Stairs;
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
                    yaw_control_theta = *yaw_imu_angle;
                }
            }
        }
        last_chassis_mode       = chassis_mode;
        last_arm_mode           = *arm_mode;
        last_is_yaw_imu_control = is_yaw_imu_control;
    }

    void yaw_control() {
        if (is_yaw_imu_control) {
            *chassis_big_yaw_target_angle_error =
                normalizeAngle(yaw_control_theta - *yaw_imu_angle);
        } else {
            std::array<double, 6> result = yaw_trajectory_controller.trajectory();
            yaw_control_theta            = result[0];
            *chassis_big_yaw_target_angle_error =
                normalizeAngle(yaw_control_theta - *chassis_big_yaw_angle);
        }
    }
    void steering_control(const Eigen::Vector2d& move, double spin_speed) {

        Eigen::Vector2d lf_vel = Eigen::Vector2d{-spin_speed, spin_speed} + move;
        Eigen::Vector2d lb_vel = Eigen::Vector2d{-spin_speed, -spin_speed} + move;
        Eigen::Vector2d rb_vel = Eigen::Vector2d{spin_speed, -spin_speed} + move;
        Eigen::Vector2d rf_vel = Eigen::Vector2d{spin_speed, spin_speed} + move;

        Eigen::Vector2d lb_vel_angle = lb_vel;
        Eigen::Vector2d lf_vel_angle = lf_vel;
        Eigen::Vector2d rb_vel_angle = rb_vel;
        Eigen::Vector2d rf_vel_angle = rf_vel;

        double err[4] = {
            -atan2(lb_vel_angle.y(), lb_vel_angle.x()) - *steering_lb_angle,
            -atan2(lf_vel_angle.y(), lf_vel_angle.x()) - *steering_lf_angle,
            -atan2(rb_vel_angle.y(), rb_vel_angle.x()) - *steering_rb_angle,
            -atan2(rf_vel_angle.y(), rf_vel_angle.x()) - *steering_rf_angle};

        *steering_lf_target_angle_error = norm_error_angle(err[1]);
        *steering_lb_target_angle_error = norm_error_angle(err[0]);
        *steering_rb_target_angle_error = norm_error_angle(err[2]);
        *steering_rf_target_angle_error = norm_error_angle(err[3]);
        *steering_wheel_lf_target_vel =
            lf_vel.norm() * (speed_limit / wheel_r) * check_error_angle(err[1]);
        *steering_wheel_lb_target_vel =
            lb_vel.norm() * (speed_limit / wheel_r) * check_error_angle(err[0]);
        *steering_wheel_rb_target_vel =
            rb_vel.norm() * (speed_limit / wheel_r) * check_error_angle(err[2]);
        *steering_wheel_rf_target_vel =
            rf_vel.norm() * (speed_limit / wheel_r) * check_error_angle(err[3]);
    }

    void reset_motor() {
        *steering_lf_target_angle_error     = NAN;
        *steering_lb_target_angle_error     = NAN;
        *steering_rb_target_angle_error     = NAN;
        *steering_rf_target_angle_error     = NAN;
        *steering_lf_target_angle_error     = NAN;
        *steering_lb_target_angle_error     = NAN;
        *steering_rb_target_angle_error     = NAN;
        *steering_rf_target_angle_error     = NAN;
        *steering_wheel_lf_target_vel       = NAN;
        *steering_wheel_lb_target_vel       = NAN;
        *steering_wheel_rb_target_vel       = NAN;
        *steering_wheel_rf_target_vel       = NAN;
        *chassis_big_yaw_target_angle_error = NAN;
    }
    static inline double norm_error_angle(const double& angle) {
        double tmp = angle;
        while (tmp > 2 * M_PI)
            tmp -= 2 * M_PI;
        while (tmp <= 0)
            tmp += 2 * M_PI;
        if (tmp > 2 * M_PI - tmp)
            tmp = tmp - 2 * M_PI;
        if (tmp > M_PI / 2)
            tmp = tmp - M_PI;
        else if (tmp < -M_PI / 2)
            tmp = tmp + M_PI;
        return tmp;
    }
    static inline double check_error_angle(const double& angle) {
        double tmp = angle;
        while (tmp > 2 * M_PI)
            tmp -= 2 * M_PI;
        while (tmp <= 0)
            tmp += 2 * M_PI;
        tmp -= M_PI;

        return sin(abs(tmp) - M_PI_2);
    }
    static double normalizeAngle(double angle) {

        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }
    
    InputInterface<rmcs_msgs::ArmMode> arm_mode;
    rmcs_msgs::ArmMode last_arm_mode;
    pid::PidCalculator following_velocity_controller_;

    double speed_limit              = 4.5;  // m/s
    static constexpr double wheel_r = 0.11; // m

    const Eigen::Vector2d lf_vel_{1, -1};
    const Eigen::Vector2d lb_vel_{1, 1};
    const Eigen::Vector2d rb_vel_{-1, -1};
    const Eigen::Vector2d rf_vel_{-1, 1};
    Eigen::Vector2d lf_vel_last_{1, -1};
    Eigen::Vector2d lb_vel_last_{1, 1};
    Eigen::Vector2d rb_vel_last_{-1, -1};
    Eigen::Vector2d rf_vel_last_{-1, 1};
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    OutputInterface<bool> is_chassis_and_leg_enable;
    // ————————————————————————steering——————————————————————————
    InputInterface<double> steering_lf_angle;
    InputInterface<double> steering_lb_angle;
    InputInterface<double> steering_rb_angle;
    InputInterface<double> steering_rf_angle;

    OutputInterface<double> steering_lf_target_angle_error;
    OutputInterface<double> steering_lb_target_angle_error;
    OutputInterface<double> steering_rb_target_angle_error;
    OutputInterface<double> steering_rf_target_angle_error;
    OutputInterface<double> steering_wheel_lf_target_vel;
    OutputInterface<double> steering_wheel_lb_target_vel;
    OutputInterface<double> steering_wheel_rb_target_vel;
    OutputInterface<double> steering_wheel_rf_target_vel;
    // —————————————————————————leg————————————————————————————————
    InputInterface<double> theta_lf;
    InputInterface<double> theta_lb;
    InputInterface<double> theta_rb;
    InputInterface<double> theta_rf;

    bool is_yaw_imu_control      = false;
    bool last_is_yaw_imu_control = false;
    InputInterface<double> yaw_imu_velocity;
    InputInterface<double> yaw_imu_angle;
    InputInterface<double> joint1_theta;
    double yaw_control_theta            = NAN;
    double yaw_set_theta_in_YawFreeMode = NAN;
    hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT> yaw_trajectory_controller;

    OutputInterface<double> chassis_big_yaw_target_angle_error;
    InputInterface<double> chassis_big_yaw_angle;
    rmcs_msgs::ChassisMode chassis_mode      = rmcs_msgs::ChassisMode::None;
    rmcs_msgs::ChassisMode last_chassis_mode = rmcs_msgs::ChassisMode::None;
};
} // namespace rmcs_core::controller::chassis
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::Chassisonly_Controller, rmcs_executor::Component)
