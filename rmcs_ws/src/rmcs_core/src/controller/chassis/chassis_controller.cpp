#include "controller/arm/trajectory.hpp"
#include "controller/pid/pid_calculator.hpp"
#include "rmcs_msgs/arm_mode.hpp"
#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/switch.hpp>
#include <vector>
namespace rmcs_core::controller::chassis {

class ChassisController
    : public rmcs_executor::Component
    , public rclcpp::Node {
    enum class YawControlMode : uint8_t {
        IMU,
        Encoder,
    };

    enum class SpeedGear : uint8_t {
        High,
        Medium,
        Low,
    };

public:
    ChassisController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , following_velocity_controller_(7.0, 0.0, 0.0) {
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/keyboard", keyboard_);

        register_input("yaw_imu_angle", yaw_imu_angle_);
        register_input("/arm/joint_1/theta", joint1_theta_);
        register_input("/arm/mode", arm_mode_);
        register_input("/arm/custom_big_yaw", custom_big_yaw_);

        register_input("/steering/power_meter/power", chassis_power_);

        register_output(
            "/chassis/big_yaw/target_angle_error", chassis_big_yaw_target_angle_error_, NAN);
        register_input("/chassis/big_yaw/angle", chassis_big_yaw_angle_);
        register_output("/chassis/control_power_limit", chassis_control_power_limit_, 120.0);
        register_output("/chassis/control_velocity", chassis_control_velocity_);

        register_output("/move_speed_limit", speed_limit_, 3.0);
    }
    void update() override {
        using namespace rmcs_msgs;

        static bool initial_check_done_ = false;

        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;

        if (!initial_check_done_) {
            reset_motor();
            yaw_target_angle_ = *yaw_imu_angle_;
            yaw_control_mode_ = YawControlMode::IMU;
            if (switch_left == Switch::DOWN && switch_right == Switch::DOWN) {
                initial_check_done_ = true;
            }
            last_arm_mode_ = *arm_mode_;
            return;
        }

        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_motor();
            last_arm_mode_ = *arm_mode_;
            return;
        }

        mode_selection();

        static double angular_velocity{0.0};
        switch (chassis_mode_) {
        case rmcs_msgs::ChassisMode::Flow: {
            set_yaw_mode(YawControlMode::IMU);
            yaw_target_angle_ += joystick_right_->y() * 0.002;
            angular_velocity = std::clamp(
                following_velocity_controller_.update(*chassis_big_yaw_angle_),
                -angular_velocity_limit_, angular_velocity_limit_);
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
            if (is_stair_mode()) {
                const auto result = yaw_trajectory_controller_.trajectory();
                yaw_target_angle_ = !result.empty() ? result[0] : 0.0;
            } else if (*arm_mode_ == rmcs_msgs::ArmMode::Custome) {
                yaw_target_angle_ = *custom_big_yaw_;
            } else {
                yaw_target_angle_ += joystick_right_->y() * 0.002;
            }
            angular_velocity = 0.0;

            break;
        }
        default: break;
        }

        if (chassis_mode_ != rmcs_msgs::ChassisMode::None) {
            auto move = *joystick_left_;

            Eigen::Rotation2D<double> rotation(*chassis_big_yaw_angle_+*joint1_theta_ );
            move = rotation * (*joystick_left_);
            if (is_stair_mode()) {
                move.y() = 0.0;
            }
            chassis_control_velocity_->vector << (move * *speed_limit_), angular_velocity;
        } else {
            chassis_control_velocity_->vector << NAN, NAN, NAN;
        }

        *chassis_big_yaw_target_angle_error_ =
            normalize_angle(yaw_target_angle_ - get_yaw_feedback());

        calculate_virtual_energy();
        last_arm_mode_ = *arm_mode_;
    }

private:
    static double speed_gear_value(SpeedGear gear) {
        switch (gear) {
        case SpeedGear::Medium: return 2.0;
        case SpeedGear::Low: return 0.8;
        case SpeedGear::High:
        default: return 3.0;
        }
    }

    void mode_selection() {
        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto keyboard     = *keyboard_;
        using namespace rmcs_msgs;
        if (switch_left == Switch::MIDDLE
            && (switch_right == Switch::MIDDLE || switch_right == Switch::DOWN)) {
            chassis_mode_ = ChassisMode::Flow;
            set_speed_gear(SpeedGear::High);
        } else if (switch_left == Switch::MIDDLE && switch_right == Switch::UP) {
            chassis_mode_ = ChassisMode::SPIN;
            set_speed_gear(SpeedGear::High);
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
            if (keyboard.q) {
                if (!keyboard.shift && !keyboard.ctrl) {
                    chassis_mode_ = rmcs_msgs::ChassisMode::Flow;
                }
                if (keyboard.shift && !keyboard.ctrl) {
                    chassis_mode_ = rmcs_msgs::ChassisMode::Yaw_Free;
                }
            }
            if (keyboard.f) {
                set_speed_gear(SpeedGear::Low);
            }
            if (last_arm_mode_ != *arm_mode_) {
                switch (*arm_mode_) {
                case rmcs_msgs::ArmMode::Auto_Linear:
                case rmcs_msgs::ArmMode::Custome:
                    set_speed_gear(SpeedGear::Low);
                    chassis_mode_ = rmcs_msgs::ChassisMode::Yaw_Free;
                    break;
                case rmcs_msgs::ArmMode::Auto_Spin:
                    set_speed_gear(SpeedGear::High);
                    chassis_mode_ = rmcs_msgs::ChassisMode::SPIN;
                    break;
                case rmcs_msgs::ArmMode::Auto_Walk:
                    set_speed_gear(SpeedGear::High);
                    chassis_mode_ = rmcs_msgs::ChassisMode::Flow;
                    break;
                case rmcs_msgs::ArmMode::Auto_Up_One_Stairs:
                case rmcs_msgs::ArmMode::Auto_Up_Two_Stairs:
                case rmcs_msgs::ArmMode::Auto_Down_Stairs:
                    set_speed_gear(SpeedGear::Medium);
                    chassis_mode_ = rmcs_msgs::ChassisMode::Yaw_Free;
                    yaw_trajectory_controller_
                        .set_start_point(std::vector<double>{*chassis_big_yaw_angle_})
                        .set_total_step(600)
                        .set_end_point(std::vector<double>{0.0})
                        .reset();
                    break;
                default: set_speed_gear(SpeedGear::Medium); break;
                }
            }

        } else {
            chassis_mode_ = rmcs_msgs::ChassisMode::None;
        }
    }

    double get_yaw_feedback() {
        return (yaw_control_mode_ == YawControlMode::IMU) ? *yaw_imu_angle_
                                                          : *chassis_big_yaw_angle_;
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
        *chassis_big_yaw_target_angle_error_ = NAN;
        *chassis_control_velocity_           = {NAN, NAN, NAN};
        *chassis_control_power_limit_        = 0.0;
        virtual_buffer_energy_               = virtual_buffer_energy_limit_;
    }
    static double normalize_angle(double angle) {
        angle = std::fmod(angle + M_PI, 2 * M_PI);
        return angle < 0 ? angle + M_PI : angle - M_PI;
    }
    bool is_stair_mode() {
        return *arm_mode_ == rmcs_msgs::ArmMode::Auto_Up_One_Stairs
            || *arm_mode_ == rmcs_msgs::ArmMode::Auto_Up_Two_Stairs
            || *arm_mode_ == rmcs_msgs::ArmMode::Auto_Down_Stairs;
    }
    void calculate_virtual_energy() {
        // Virtual buffer energy power control
        constexpr double dt  = 0.001; // 1ms update cycle
        double chassis_power = *chassis_power_;
        virtual_buffer_energy_ += dt * (power_limit_ - chassis_power);
        virtual_buffer_energy_ =
            std::clamp(virtual_buffer_energy_, 0.0, virtual_buffer_energy_limit_);

        *chassis_control_power_limit_ =
            power_limit_ * (virtual_buffer_energy_ / virtual_buffer_energy_limit_);
    }
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    InputInterface<rmcs_msgs::ArmMode> arm_mode_;
    rmcs_msgs::ArmMode last_arm_mode_    = rmcs_msgs::ArmMode::None;
    rmcs_msgs::ChassisMode chassis_mode_ = rmcs_msgs::ChassisMode::None;

    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;
    InputInterface<double> chassis_power_;
    OutputInterface<double> chassis_control_power_limit_;
    OutputInterface<double> speed_limit_;

    static constexpr double power_limit_                 = 120.0;
    static constexpr double virtual_buffer_energy_limit_ = 1.0;
    double virtual_buffer_energy_                        = virtual_buffer_energy_limit_;
    double angular_velocity_limit_                       = 10.0;
    pid::PidCalculator following_velocity_controller_;
    InputInterface<double> joint1_theta_;
    InputInterface<double> custom_big_yaw_;

    YawControlMode yaw_control_mode_ = YawControlMode::IMU;
    InputInterface<double> yaw_imu_angle_;
    InputInterface<double> chassis_big_yaw_angle_;
    OutputInterface<double> chassis_big_yaw_target_angle_error_;
    double yaw_target_angle_ = NAN;
    rmcs_core::controller::arm::Trajectory<rmcs_core::controller::arm::TrajectoryType::JOINT>
        yaw_trajectory_controller_{1};
};
} // namespace rmcs_core::controller::chassis
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::ChassisController, rmcs_executor::Component)
