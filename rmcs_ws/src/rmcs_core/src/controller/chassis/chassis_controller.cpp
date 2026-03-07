#include "controller/arm/trajectory.hpp"
#include "controller/pid/pid_calculator.hpp"
#include "rmcs_msgs/arm_mode.hpp"
#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/switch.hpp>
#include <vector>
namespace rmcs_core::controller::chassis {

enum class YawControlMode : uint8_t {
    IMU,
    Encoder,
};

enum class SpeedGear : uint8_t {
    High,
    Medium,
    Low,
};

inline double speed_gear_value(SpeedGear gear) {
    switch (gear) {
    case SpeedGear::High: return 3.5;
    case SpeedGear::Medium: return 2.5;
    case SpeedGear::Low: return 0.8;
    }
    return 3.5;
}

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
        register_input("/remote/keyboard", keyboard_);

        register_input("yaw_imu_angle", yaw_imu_angle);
        register_input("/arm/joint_1/theta", joint1_theta);
        register_input("/arm/mode", arm_mode);

        register_output(
            "/chassis/big_yaw/target_angle_error", chassis_big_yaw_target_angle_error, NAN);
        register_input("/chassis/big_yaw/angle", chassis_big_yaw_angle);
        register_output("/chassis/control_power_limit", chassis_control_power_limit_, 120.0);
        register_output("/chassis/control_velocity", chassis_control_velocity_);

        register_output("/move_speed_limit", speed_limit_, 3.0);
    }
    void update() override {
        using namespace rmcs_msgs;

        static bool initial_check_done_ = false;

        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        double angular_velocity{0.0};

        if (!initial_check_done_) {
            reset_motor();
            yaw_target_angle_ = *yaw_imu_angle;
            yaw_control_mode_ = YawControlMode::IMU;
            if (switch_left == Switch::DOWN && switch_right == Switch::DOWN) {
                initial_check_done_ = true;
            }
        } else {
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                reset_motor();
            } else {
                mode_selection();

                switch (chassis_mode) {
                case rmcs_msgs::ChassisMode::Flow: {
                    set_yaw_mode(YawControlMode::IMU);
                    yaw_target_angle_ += joystick_right_->y() * 0.002;
                    angular_velocity = std::clamp(
                        following_velocity_controller_.update(*chassis_big_yaw_angle),
                        -angular_velocity_limit, angular_velocity_limit);
                    break;
                }
                case rmcs_msgs::ChassisMode::SPIN: {
                    set_yaw_mode(YawControlMode::IMU);
                    yaw_target_angle_ += joystick_right_->y() * 0.002;
                    angular_velocity = 5.0;
                    break;
                }
                case rmcs_msgs::ChassisMode::Yaw_Free: {
                    set_yaw_mode(YawControlMode::Encoder);
                    if (*arm_mode == rmcs_msgs::ArmMode::Auto_Up_One_Stairs
                        || *arm_mode == rmcs_msgs::ArmMode::Auto_Up_Two_Stairs
                        || *arm_mode == rmcs_msgs::ArmMode::Auto_Down_Stairs) {
                        const auto result = yaw_trajectory_controller.trajectory();
                        yaw_target_angle_ = !result.empty() ? result[0] : 0.0;
                    } else {
                        yaw_target_angle_ += joystick_right_->y() * 0.002;
                    }
                    angular_velocity = joystick_right_->y() * angular_velocity_limit;

                    break;
                }
                default: break;
                }

                if (chassis_mode != rmcs_msgs::ChassisMode::None) {
                    auto move = *joystick_left_;
                    if (*arm_mode == rmcs_msgs::ArmMode::Auto_Up_One_Stairs
                        || *arm_mode == rmcs_msgs::ArmMode::Auto_Up_Two_Stairs
                        || *arm_mode == rmcs_msgs::ArmMode::Auto_Down_Stairs) {
                        move.x() = 0.0;
                    }
                    // Eigen::Rotation2D<double> rotation(*chassis_big_yaw_angle + *joint1_theta);
                    // move_ = rotation * (*joystick_left_);
                    chassis_control_velocity_->vector << (move * *speed_limit_), angular_velocity;
                } else {
                    chassis_control_velocity_->vector << NAN, NAN, NAN;
                }

                *chassis_big_yaw_target_angle_error =
                    normalizeAngle(yaw_target_angle_ - get_yaw_feedback());
            }
        }
        last_arm_mode = *arm_mode;
    }

private:
    void mode_selection() {
        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto keyboard     = *keyboard_;
        using namespace rmcs_msgs;
        if (switch_left == Switch::MIDDLE
            && (switch_right == Switch::MIDDLE || switch_right == Switch::DOWN)) {
            chassis_mode = ChassisMode::Flow;
        } else if (switch_left == Switch::MIDDLE && switch_right == Switch::UP) {
            chassis_mode = ChassisMode::SPIN;
        } else if (switch_left == Switch::DOWN && switch_right == Switch::UP) {
            if (keyboard.c) {
                if (!keyboard.shift && !keyboard.ctrl) {
                    set_speed_gear(SpeedGear::High);
                }
                if (keyboard.shift && !keyboard.ctrl) {
                    set_speed_gear(SpeedGear::Medium);
                }
                if (!keyboard.shift && keyboard.ctrl) {
                    set_speed_gear(SpeedGear::Low);
                }
            }
            if (keyboard.a) {
                if (!keyboard.shift && !keyboard.ctrl) {
                    chassis_mode = rmcs_msgs::ChassisMode::Flow;
                }
            }
            if (keyboard.s) {
                if (!keyboard.shift && !keyboard.ctrl) {
                    chassis_mode = rmcs_msgs::ChassisMode::SPIN;
                }
            }
            if (last_arm_mode != *arm_mode) {
                switch (*arm_mode) {
                case rmcs_msgs::ArmMode::Customer:
                    set_speed_gear(SpeedGear::Low);
                    chassis_mode = rmcs_msgs::ChassisMode::Yaw_Free;
                    break;
                case rmcs_msgs::ArmMode::Auto_Walk:
                    set_speed_gear(SpeedGear::High);
                    chassis_mode = rmcs_msgs::ChassisMode::Flow;
                    break;
                case rmcs_msgs::ArmMode::Auto_Up_One_Stairs:
                case rmcs_msgs::ArmMode::Auto_Up_Two_Stairs:
                case rmcs_msgs::ArmMode::Auto_Down_Stairs:
                    set_speed_gear(SpeedGear::Medium);
                    chassis_mode = rmcs_msgs::ChassisMode::SPIN;
                    yaw_trajectory_controller
                        .set_start_point(std::vector<double>{*chassis_big_yaw_angle})
                        .set_total_step(600)
                        .set_end_point(std::vector<double>{0.0})
                        .reset();
                    break;
                default: set_speed_gear(SpeedGear::Medium); break;
                }
            }

        } else {
            chassis_mode = rmcs_msgs::ChassisMode::None;
        }
    }

    double get_yaw_feedback() {
        return (yaw_control_mode_ == YawControlMode::IMU) ? *yaw_imu_angle : *chassis_big_yaw_angle;
    }
    void set_speed_gear(SpeedGear gear) { *speed_limit_ = speed_gear_value(gear); }

    void set_yaw_mode(YawControlMode new_mode) {
        if (new_mode == yaw_control_mode_)
            return;
        double old_feedback = get_yaw_feedback();
        yaw_control_mode_   = new_mode;
        double new_feedback = get_yaw_feedback();
        yaw_target_angle_ += (new_feedback - old_feedback);
    }
    void reset_motor() {
        *chassis_big_yaw_target_angle_error = NAN;
        *chassis_control_velocity_          = {NAN, NAN, NAN};
        *chassis_control_power_limit_       = 0.0;
    }
    static double normalizeAngle(double angle) {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    InputInterface<rmcs_msgs::ArmMode> arm_mode;
    rmcs_msgs::ArmMode last_arm_mode    = rmcs_msgs::ArmMode::None;
    rmcs_msgs::ChassisMode chassis_mode = rmcs_msgs::ChassisMode::None;

    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;
    OutputInterface<double> chassis_control_power_limit_;
    OutputInterface<double> speed_limit_;
    double angular_velocity_limit = 6.0;
    pid::PidCalculator following_velocity_controller_;
    InputInterface<double> joint1_theta;

    YawControlMode yaw_control_mode_ = YawControlMode::IMU;
    InputInterface<double> yaw_imu_angle;
    InputInterface<double> chassis_big_yaw_angle;
    OutputInterface<double> chassis_big_yaw_target_angle_error;
    double yaw_target_angle_ = NAN;
    rmcs_core::controller::arm::Trajectory<rmcs_core::controller::arm::TrajectoryType::JOINT>
        yaw_trajectory_controller{1};
};
} // namespace rmcs_core::controller::chassis
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::Chassis_Controller, rmcs_executor::Component)
