#include "controller/pid/pid_calculator.hpp"
#include "hardware/device/trajectory.hpp"
#include <algorithm>
#include <arm_mode.hpp>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <numbers>
#include <random>
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
class Chassis_and_LegController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Chassis_and_LegController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , following_velocity_controller_(0.46, 0.00, 0.089)
        , leg_lf_error_pid_controller(240.0, 0.0, 5.0)
        , leg_lf_velocity_pid_controller(2.0, 0.0, 0.2)
        , leg_rf_error_pid_controller(240.0, 0.0, 5.0)
        , leg_rf_velocity_pid_controller(2.0, 0.0, 0.2) {

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
        register_output("/leg/omni/l/target_vel", omni_l_target_vel, NAN);
        register_output("/leg/omni/r/target_vel", omni_r_target_vel, NAN);

        register_input("/leg/encoder/lf/angle", theta_lf);
        register_input("/leg/encoder/lb/angle", theta_lb);
        register_input("/leg/encoder/rb/angle", theta_rb);
        register_input("/leg/encoder/rf/angle", theta_rf);
        register_output("/leg/joint/lf/control_torque", leg_joint_lf_control_torque, NAN);
        register_output("/leg/joint/lb/target_theta", leg_lb_target_theta, NAN);
        register_output("/leg/joint/rb/target_theta", leg_rb_target_theta, NAN);
        register_output("/leg/joint/rf/control_torque", leg_joint_rf_control_torque, NAN);

        register_input("/leg/joint/lf/velocity", leg_joint_lf_velocity);
        register_input("/leg/joint/rf/velocity", leg_joint_rf_velocity);
        register_input("/leg/joint/lf/torque", leg_joint_lf_torque);
        register_input("/leg/joint/rf/torque", leg_joint_rf_torque);

        register_input("yaw_imu_velocity", yaw_imu_velocity);
        register_input("yaw_imu_angle", yaw_imu_angle);
        register_input("/arm/Joint1/theta", joint1_theta);

        register_input("/arm/mode", arm_mode);

        register_output(
            "/chassis/big_yaw/target_angle_error", chassis_big_yaw_target_angle_error, NAN);
        register_input("/chassis/big_yaw/angle", chassis_big_yaw_angle);

        std::array<double, 2> four_wheel_angle = leg_inverse_kinematic(237.0, 221.0, false, false);
        four_wheel_trajectory
            .set_end_point(
                {four_wheel_angle[0], four_wheel_angle[1], four_wheel_angle[1], four_wheel_angle[0],
                 0, 0})
            .set_total_step(500.0);
        std::array<double, 2> six_wheel_angle = leg_inverse_kinematic(250.0, 221.0, false, false);
        six_wheel_trajectory
            .set_end_point(
                {six_wheel_angle[0], six_wheel_angle[1], six_wheel_angle[1], six_wheel_angle[0], 0,
                 0})
            .set_total_step(500.0);

        up_stairs_initial.set_end_point({1.700270, 1.680, 1.680, 1.700270, 0, 0})
            .set_total_step(2000);
        up_stairs_leg_press.set_end_point({0.506814, 0.995241, 0.995241, 0.466814, 0, 0})
            .set_total_step(1000);
        up_stairs_leg_lift
            .set_end_point({1.2193598767, 1.261730456, 1.261730456, 1.2193598767, 0, 0})
            .set_total_step(3100);
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
            is_leg_forward_joint_torque_control = false;
            leg_mode                            = rmcs_msgs::LegMode::None;
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
                if (leg_mode == rmcs_msgs::LegMode::Four_Wheel) {
                    omniwheel_control(Eigen::Vector2d{NAN, NAN});
                } else {
                    omniwheel_control(move_);
                }
                steering_control(move_, spin_speed);
                break;
            }
            case rmcs_msgs::ChassisMode::SPIN: {
                // leg_mode          = rmcs_msgs::LegMode::Four_Wheel;
                double spin_speed = 0.8;
                speed_limit       = 5.0;
                Eigen::Rotation2D<double> rotation(*chassis_big_yaw_angle + *joint1_theta);
                Eigen::Vector2d move_ = rotation * (*joystick_left_);
                yaw_control_theta += joystick_right_->y() * 0.002;
                steering_control(move_, spin_speed);
                omniwheel_control(Eigen::Vector2d{NAN, NAN});

                break;

            }
            case rmcs_msgs::ChassisMode::Up_Stairs: {
                is_yaw_imu_control           = false;
                yaw_set_theta_in_YawFreeMode = 0.0;
                speed_limit                  = 1.5;
                Eigen::Vector2d move_        = *joystick_left_;
                steering_control(move_, joystick_right_->y());
                omniwheel_control(move_);
                break;
            }
            case rmcs_msgs::ChassisMode::Yaw_Free: {
                Eigen::Rotation2D<double> rotation(*chassis_big_yaw_angle + *joint1_theta);
                Eigen::Vector2d move_ = rotation * (*joystick_left_);
                steering_control(move_, joystick_right_->y() / 1.5);
                omniwheel_control(move_);
                break;
            }
            default: break;
            }
            yaw_control();
            leg_control();
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
            leg_mode           = rmcs_msgs::LegMode::Four_Wheel;
        } else if (switch_left == Switch::MIDDLE && switch_right == Switch::DOWN) {
            chassis_mode       = ChassisMode::Flow;
            is_yaw_imu_control = true;
            leg_mode           = rmcs_msgs::LegMode::Six_Wheel;

        } else if (switch_left == Switch::MIDDLE && switch_right == Switch::UP) {
            chassis_mode       = ChassisMode::SPIN;
            is_yaw_imu_control = true;
            leg_mode           = rmcs_msgs::LegMode::Four_Wheel;

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
            if (keyboard.v) {
                if (!keyboard.shift && !keyboard.ctrl) {
                    leg_mode = rmcs_msgs::LegMode::Six_Wheel;
                }
                if (keyboard.shift && !keyboard.ctrl) {
                    leg_mode = rmcs_msgs::LegMode::Four_Wheel;
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
                leg_mode               = rmcs_msgs::LegMode::Up_Stairs;
                up_stairs_is_leg_press = false;
                up_stairs_is_leg_lift  = false;
                up_stairs_initial.reset();
                up_stairs_leg_press.reset();
                up_stairs_leg_lift.reset();
                up_stairs_initial.set_start_point(
                    {*theta_lf, *theta_lb, *theta_rb, *theta_rf, 0.0, 0.0});
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
                        leg_mode                     = rmcs_msgs::LegMode::Four_Wheel;
                        chassis_mode                 = rmcs_msgs::ChassisMode::Yaw_Free;
                        yaw_set_theta_in_YawFreeMode = 0.0;
                    } else {
                        chassis_mode = ChassisMode::Yaw_Free;
                        leg_mode     = rmcs_msgs::LegMode::Six_Wheel;

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
                    leg_mode           = rmcs_msgs::LegMode::Six_Wheel;
                    chassis_mode       = rmcs_msgs::ChassisMode::Flow;
                    is_yaw_imu_control = true;
                }
            }
        } else {
            chassis_mode = rmcs_msgs::ChassisMode::None;
            leg_mode     = rmcs_msgs::LegMode::None;
        }

        if (last_leg_mode != leg_mode) {
            if (leg_mode == rmcs_msgs::LegMode::Four_Wheel) {
                four_wheel_trajectory.reset();
                four_wheel_trajectory.set_start_point(
                    {*theta_lf, *theta_lb, *theta_rb, *theta_rf, 0.0, 0.0});

            } else if (leg_mode == rmcs_msgs::LegMode::Six_Wheel) {
                six_wheel_trajectory.reset();
                six_wheel_trajectory.set_start_point(
                    {*theta_lf, *theta_lb, *theta_rb, *theta_rf, 0.0, 0.0});
            }
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
        last_leg_mode           = leg_mode;
        last_is_yaw_imu_control = is_yaw_imu_control;
    }

    void leg_control() {

        static std::array<double, 6> result;

        if (leg_mode == rmcs_msgs::LegMode::Four_Wheel) {
            is_leg_forward_joint_torque_control = false;
            result                              = four_wheel_trajectory.trajectory();

        } else if (leg_mode == rmcs_msgs::LegMode::Six_Wheel) {
            if (!six_wheel_trajectory.get_complete()) {
                is_leg_forward_joint_torque_control = false;
                result                              = six_wheel_trajectory.trajectory();
            } else {
                is_leg_forward_joint_torque_control = false;
                result                              = six_wheel_trajectory.trajectory();
                if (*theta_lf < std::numbers::pi / 2.0 || *theta_rf < std::numbers::pi / 2.0) {
                    result[0] = NAN;
                    result[3] = NAN;
                }
            }
        } else if (leg_mode == rmcs_msgs::LegMode::Up_Stairs) {
            is_leg_forward_joint_torque_control = false;
            if (!up_stairs_initial.get_complete()) {
                result = up_stairs_initial.trajectory();
                up_stairs_leg_press.set_start_point(
                    {*theta_lf, *theta_lb, *theta_rb, *theta_rf, 0.0, 0.0});
                up_stairs_is_leg_press = false;
            } else {
                if (keyboard_->shift) {
                    up_stairs_is_leg_press = true;
                }
                if (keyboard_->ctrl) {
                    up_stairs_is_leg_lift = true;
                } else if (up_stairs_is_leg_press) {
                    if (!up_stairs_leg_press.get_complete()) {
                        result = up_stairs_leg_press.trajectory();
                        up_stairs_leg_lift.set_start_point(
                            {*theta_lf, *theta_lb, *theta_rb, *theta_rf, 0.0, 0.0});
                    } else if (up_stairs_is_leg_lift) {
                        result = up_stairs_leg_lift.trajectory();
                        if (up_stairs_leg_lift.get_complete()) {
                            result[0] = 0.8;
                            result[3] = 0.8;
                        }
                    }
                }
            }
        } else if (leg_mode == rmcs_msgs::LegMode::None) {
            is_leg_forward_joint_torque_control = false;
            result[0]                           = *theta_lf;
            result[1]                           = *theta_lb;
            result[2]                           = *theta_rb;
            result[3]                           = *theta_rf;
        }
        leg_joint_controller(result[0], result[1], result[2], result[3]);
    }

    void leg_joint_controller(double lf, double lb, double rb, double rf) {
        if (is_leg_forward_joint_torque_control) {
            set_leg_forward_joint_torque(lf, rf);
            *leg_lb_target_theta = lb;
            *leg_rb_target_theta = rb;

        } else {
            leg_lf_target_theta  = lf;
            leg_rf_target_theta  = rf;
            *leg_lb_target_theta = lb;
            *leg_rb_target_theta = rb;
            double diff          = (*theta_rf);
            double leg_joint_lf_control_vel =
                leg_lf_error_pid_controller.update(normalizeAngle(leg_lf_target_theta - *theta_lf));
            double leg_joint_rf_control_vel =
                leg_rf_error_pid_controller.update(normalizeAngle(leg_rf_target_theta - *theta_rf));
            set_leg_forward_joint_torque(
                leg_lf_velocity_pid_controller.update(
                    leg_joint_lf_control_vel - *leg_joint_lf_velocity),
                leg_rf_velocity_pid_controller.update(
                    leg_joint_rf_control_vel - *leg_joint_rf_velocity));
        }
    };
    void set_leg_forward_joint_torque(double lf, double rf) {
        *leg_joint_lf_control_torque = lf;
        *leg_joint_rf_control_torque = rf;
    }
    static std::array<double, 2> leg_inverse_kinematic(
        double f_x, double b_x, bool is_front_ecd_obtuse, bool is_back_ecd_obtuse) {
        constexpr double link1 = 240.0f, link2 = 120.0f, link3 = 160.0f;
        double theta_f, theta_b;
        double theta_b_ = asin(b_x / link1);
        if (is_back_ecd_obtuse) {
            theta_b = std::numbers::pi - theta_b_;
        } else {
            theta_b = theta_b_;
        }
        double x_link2  = link2 * sin(5.0 * std::numbers::pi / 6.0 - theta_b);
        double theta_f_ = asin((f_x - x_link2) / link3);
        if (is_front_ecd_obtuse) {
            theta_f = std::numbers::pi - theta_f_;
        } else {
            theta_f = theta_f_;
        }
        return {theta_f, theta_b};
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
    void omniwheel_control(const Eigen::Vector2d& move) {
        *omni_l_target_vel = move.x() * (speed_limit / wheel_r);
        *omni_r_target_vel = move.x() * (speed_limit / wheel_r);
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
        *omni_l_target_vel                  = NAN;
        *omni_r_target_vel                  = NAN;
        *steering_wheel_lf_target_vel       = NAN;
        *steering_wheel_lb_target_vel       = NAN;
        *steering_wheel_rb_target_vel       = NAN;
        *steering_wheel_rf_target_vel       = NAN;
        leg_lf_target_theta                 = NAN;
        *leg_lb_target_theta                = NAN;
        *leg_rb_target_theta                = NAN;
        leg_rf_target_theta                 = NAN;
        *leg_joint_lf_control_torque        = NAN;
        *leg_joint_rf_control_torque        = NAN;
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
    int generateRandomInt(int min, int max) {
        // 使用随机设备生成随机数种子
        std::random_device rd;
        std::default_random_engine gen(rd());

        // 使用 uniform_int_distribution 来生成指定范围内的随机整数
        std::uniform_int_distribution<int> dist(min, max);

        return dist(gen);
    }
    InputInterface<rmcs_msgs::ArmMode> arm_mode;
    rmcs_msgs::ArmMode last_arm_mode;
    pid::PidCalculator following_velocity_controller_;

    pid::PidCalculator leg_lf_error_pid_controller;
    pid::PidCalculator leg_lf_velocity_pid_controller;
    pid::PidCalculator leg_rf_error_pid_controller;
    pid::PidCalculator leg_rf_velocity_pid_controller;

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
    OutputInterface<double> omni_l_target_vel;
    OutputInterface<double> omni_r_target_vel;
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

    bool is_leg_forward_joint_torque_control = false;
    double leg_lf_target_theta               = NAN;
    OutputInterface<double> leg_lb_target_theta;
    OutputInterface<double> leg_rb_target_theta;
    double leg_rf_target_theta  = NAN;
    bool up_stairs_is_leg_press = false;
    bool up_stairs_is_leg_lift  = false;

    InputInterface<double> leg_joint_lf_velocity;
    InputInterface<double> leg_joint_rf_velocity;
    InputInterface<double> leg_joint_lf_torque;
    InputInterface<double> leg_joint_rf_torque;
    OutputInterface<double> leg_joint_lf_control_torque;
    OutputInterface<double> leg_joint_rf_control_torque;

    OutputInterface<double> chassis_big_yaw_target_angle_error;
    InputInterface<double> chassis_big_yaw_angle;
    rmcs_msgs::ChassisMode chassis_mode      = rmcs_msgs::ChassisMode::None;
    rmcs_msgs::ChassisMode last_chassis_mode = rmcs_msgs::ChassisMode::None;
    rmcs_msgs::LegMode leg_mode              = rmcs_msgs::LegMode::Six_Wheel;
    rmcs_msgs::LegMode last_leg_mode         = rmcs_msgs::LegMode::None;

    hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT> four_wheel_trajectory;
    hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT> six_wheel_trajectory;
    hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT> up_stairs_initial;
    hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT> up_stairs_leg_press;
    hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT> up_stairs_leg_lift;
};
} // namespace rmcs_core::controller::chassis
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::Chassis_and_LegController, rmcs_executor::Component)
