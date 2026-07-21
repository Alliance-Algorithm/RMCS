#include "controller/arm/trajectory.hpp"
#include "controller/pid/pid_calculator.hpp"
#include "rmcs_msgs/arm_mode.hpp"
#include "rmcs_utility/normalize_angle.hpp"
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

class ClimberChassisController
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    ClimberChassisController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , yaw_left_limit_(get_parameter("yaw_left_limit").as_double())
        , yaw_right_limit_(get_parameter("yaw_right_limit").as_double()) {
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/arm/joint_1/theta", joint1_theta_);
        register_input("/arm/mode", arm_mode_);
        register_input("/arm/custom_big_yaw", custom_big_yaw_);

        register_input("/steering/power_meter/power", chassis_power_);
        register_output(
            "/chassis/big_yaw/target_angle_error", chassis_big_yaw_target_angle_error_, NAN);
        register_input("/chassis/big_yaw/angle", chassis_big_yaw_angle_);
        register_output("/chassis/control_power_limit", chassis_control_power_limit_, 120.0);
        register_output("/chassis/control_velocity", chassis_control_velocity_);
        register_input("/chassis/climbing_forward_velocity", climbing_forward_velocity_, NAN);

        register_output("/move_speed_limit", speed_limit_, 3.0);
    }
    void update() override {
        using namespace rmcs_msgs;
        static bool initial_check_done_ = false;

        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;

        if (!initial_check_done_) {
            reset_motor();
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

        if (chassis_mode_ != rmcs_msgs::ChassisMode::None) {
            Eigen::Rotation2D<double> rotation(*chassis_big_yaw_angle_ + *joint1_theta_);

            auto move               = rotation * (*joystick_left_);
            double angular_velocity = angular_velocity_limit_ * joystick_right_->y();

            if (*arm_mode_ == rmcs_msgs::ArmMode::Auto_Up_One_Stairs
                || *arm_mode_ == rmcs_msgs::ArmMode::Auto_Up_Two_Stairs
                || *arm_mode_ == rmcs_msgs::ArmMode::Auto_Down_Stairs) {
                move.y()         = 0.0;
                angular_velocity = 0.0;
                if (!std::isnan(*climbing_forward_velocity_)) {
                    move.x() = *climbing_forward_velocity_;
                }
            }

            chassis_control_velocity_->vector << (move * *speed_limit_), angular_velocity;
        } else {
            chassis_control_velocity_->vector << NAN, NAN, NAN;
        }
        yaw_controller();
        calculate_virtual_energy();
        last_arm_mode_ = *arm_mode_;
    }

private:
    enum class SpeedGear : uint8_t { High, Medium, Low, Stairs };

    void mode_selection() {
        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto keyboard     = *keyboard_;
        using namespace rmcs_msgs;
        if (switch_left == Switch::MIDDLE
            && (switch_right == Switch::MIDDLE || switch_right == Switch::DOWN)) {
            chassis_mode_ = ChassisMode::Yaw_Free;
            set_speed_gear(SpeedGear::High);
        } else if (switch_left == Switch::DOWN && switch_right == Switch::UP) {
            chassis_mode_ = ChassisMode::Yaw_Free;
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
            if (last_arm_mode_ != *arm_mode_) {
                switch (*arm_mode_) {
                case rmcs_msgs::ArmMode::Auto_Extract_LB:
                case rmcs_msgs::ArmMode::Auto_Extract_RB:
                case rmcs_msgs::ArmMode::Auto_Storage_LB:
                case rmcs_msgs::ArmMode::Auto_Storage_RB:
                case rmcs_msgs::ArmMode::Custome: set_speed_gear(SpeedGear::Low); break;
                case rmcs_msgs::ArmMode::Auto_Spin:
                case rmcs_msgs::ArmMode::Auto_Walk: set_speed_gear(SpeedGear::High); break;
                case rmcs_msgs::ArmMode::Auto_Up_One_Stairs:
                case rmcs_msgs::ArmMode::Auto_Up_Two_Stairs:
                case rmcs_msgs::ArmMode::Auto_Down_Stairs: set_speed_gear(SpeedGear::Stairs); break;
                default: set_speed_gear(SpeedGear::Medium); break;
                }
            }

        } else {
            chassis_mode_ = rmcs_msgs::ChassisMode::None;
        }
    }
    void yaw_controller() {
        if (*arm_mode_ == rmcs_msgs::ArmMode::Custome) {
            yaw_target_angle_ = *custom_big_yaw_;
        } else {
            yaw_target_angle_ = 0.0;
        }
        yaw_target_angle_ = std::clamp(yaw_target_angle_, yaw_left_limit_, yaw_right_limit_);
        *chassis_big_yaw_target_angle_error_ =
            rmcs_utility::normalize_angle(yaw_target_angle_ - *chassis_big_yaw_angle_);
    }
    void set_speed_gear(SpeedGear gear) {
        switch (gear) {
        case SpeedGear::Medium: *speed_limit_ = 2.0; break;
        case SpeedGear::Low: *speed_limit_ = 0.8; break;
        case SpeedGear::Stairs: *speed_limit_ = 1.8; break;
        case SpeedGear::High:
        default: *speed_limit_ = 3.0; break;
        }
    }

    void reset_motor() {
        *chassis_big_yaw_target_angle_error_ = NAN;
        yaw_target_angle_                    = 0.0;

        *chassis_control_velocity_    = {NAN, NAN, NAN};
        *chassis_control_power_limit_ = 0.0;
        virtual_buffer_energy_        = virtual_buffer_energy_limit_;
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
    InputInterface<double> climbing_forward_velocity_;
    InputInterface<double> chassis_power_;
    OutputInterface<double> chassis_control_power_limit_;
    OutputInterface<double> speed_limit_;

    static constexpr double power_limit_                 = 120.0;
    static constexpr double virtual_buffer_energy_limit_ = 10.0;
    double virtual_buffer_energy_                        = virtual_buffer_energy_limit_;
    double angular_velocity_limit_                       = 10.0;

    InputInterface<double> joint1_theta_;
    InputInterface<double> custom_big_yaw_;

    InputInterface<double> chassis_big_yaw_angle_;
    OutputInterface<double> chassis_big_yaw_target_angle_error_;

    double yaw_target_angle_ = NAN;
    double yaw_left_limit_   = NAN;
    double yaw_right_limit_  = NAN;
};
} // namespace rmcs_core::controller::chassis
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::ClimberChassisController, rmcs_executor::Component)
