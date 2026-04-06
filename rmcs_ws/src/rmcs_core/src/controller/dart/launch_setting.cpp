#include <algorithm>
#include <cmath>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/dart_mechanism_command.hpp>

#include "controller/pid/matrix_pid_calculator.hpp"

namespace rmcs_core::controller::dart {

// DartLaunchSettingV2
//   同步带与扳机执行组件：
//   - 速度、限扭、保留力矩由上层 DartManagerV2 下发
//   - 本组件仅负责 belt PID 同步控制和扳机量值映射
//   - 升降机构与限位舵机由 DartFilling 独立处理
class DartLaunchSettingV2
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartLaunchSettingV2()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , belt_pid_{
              get_parameter("b_kp").as_double(), get_parameter("b_ki").as_double(),
              get_parameter("b_kd").as_double()} {

        belt_velocity_ = get_parameter("belt_velocity").as_double();
        sync_coefficient_ = get_parameter("sync_coefficient").as_double();
        max_control_torque_ = get_parameter("max_control_torque").as_double();
        trigger_free_value_ = get_parameter("trigger_free_value").as_double();
        trigger_lock_value_ = get_parameter("trigger_lock_value").as_double();
        default_belt_hold_torque_ = get_parameter("belt_hold_torque").as_double();

        register_input("/dart/manager/belt/command", belt_command_);
        register_input("/dart/manager/belt/target_velocity", belt_target_velocity_, false);
        register_input("/dart/manager/belt/torque_limit", belt_torque_limit_, false);
        register_input("/dart/manager/belt/hold_torque", belt_hold_torque_input_, false);
        register_input("/dart/manager/belt/wait_zero_velocity", belt_wait_zero_velocity_, false);
        register_input("/dart/manager/trigger/lock_enable", trigger_lock_enable_);
        register_input("/dart/drive_belt/left/velocity", left_belt_velocity_);
        register_input("/dart/drive_belt/right/velocity", right_belt_velocity_);
        register_input("/force_sensor/channel_1/weight", force_sensor_ch1_);
        register_input("/force_sensor/channel_2/weight", force_sensor_ch2_);

        register_output("/dart/drive_belt/left/control_torque", left_belt_torque_, 0.0);
        register_output("/dart/drive_belt/right/control_torque", right_belt_torque_, 0.0);
        register_output("/dart/trigger_servo/value", trigger_value_, trigger_lock_value_);
    }

    void update() override {
        const double requested_velocity =
            belt_target_velocity_.ready() ? std::abs(*belt_target_velocity_) : belt_velocity_;

        BeltControlMode control_mode = BeltControlMode::WAIT_HOLD;
        double control_velocity = 0.0;
        switch (*belt_command_) {
        case rmcs_msgs::DartMechanismCommand::DOWN:
            control_mode = BeltControlMode::MOVE_DOWN;
            control_velocity = +requested_velocity;
            prev_belt_cmd_ = rmcs_msgs::DartMechanismCommand::DOWN;
            break;
        case rmcs_msgs::DartMechanismCommand::UP:
            control_mode = BeltControlMode::MOVE_UP;
            control_velocity = -requested_velocity;
            prev_belt_cmd_ = rmcs_msgs::DartMechanismCommand::UP;
            break;
        default:
            control_mode = belt_wait_zero_velocity_.ready() && *belt_wait_zero_velocity_
                             ? BeltControlMode::WAIT_ZERO
                             : BeltControlMode::WAIT_HOLD;
            break;
        }

        if (control_mode != control_mode_) {
            belt_pid_.reset();
            control_mode_ = control_mode;
        }

        double torque_limit =
            belt_torque_limit_.ready() ? std::abs(*belt_torque_limit_) : max_control_torque_;
        torque_limit = std::min(torque_limit, max_control_torque_);

        if (control_mode == BeltControlMode::WAIT_HOLD) {
            apply_hold_torque();
        } else {
            drive_belt_sync_control(control_velocity, torque_limit);
        }

        *trigger_value_ = *trigger_lock_enable_ ? trigger_lock_value_ : trigger_free_value_;
    }

private:
    enum class BeltControlMode {
        MOVE_UP,
        MOVE_DOWN,
        WAIT_ZERO,
        WAIT_HOLD,
    };

    void apply_hold_torque() {
        double hold_torque = 0.0;
        if (belt_hold_torque_input_.ready()) {
            hold_torque = *belt_hold_torque_input_;
        } else if (prev_belt_cmd_ == rmcs_msgs::DartMechanismCommand::DOWN) {
            hold_torque = default_belt_hold_torque_;
        }

        *left_belt_torque_ = hold_torque;
        *right_belt_torque_ = hold_torque;
    }

    void drive_belt_sync_control(double set_velocity, double torque_limit) {
        Eigen::Vector2d setpoint_error{
            set_velocity - *left_belt_velocity_, set_velocity - *right_belt_velocity_};
        Eigen::Vector2d relative_velocity{
            *left_belt_velocity_ - *right_belt_velocity_,
            *right_belt_velocity_ - *left_belt_velocity_};

        Eigen::Vector2d control_torques =
            belt_pid_.update(setpoint_error) - sync_coefficient_ * relative_velocity;

        *left_belt_torque_ = std::clamp(control_torques[0], -torque_limit, torque_limit);
        *right_belt_torque_ = std::clamp(control_torques[1], -torque_limit, torque_limit);
    }

    pid::MatrixPidCalculator<2> belt_pid_;

    double belt_velocity_;
    double sync_coefficient_;
    double max_control_torque_;
    double trigger_free_value_;
    double trigger_lock_value_;
    double default_belt_hold_torque_{0.0};

    rmcs_msgs::DartMechanismCommand prev_belt_cmd_{rmcs_msgs::DartMechanismCommand::WAIT};
    BeltControlMode control_mode_{BeltControlMode::WAIT_HOLD};

    InputInterface<rmcs_msgs::DartMechanismCommand> belt_command_;
    InputInterface<double> belt_target_velocity_;
    InputInterface<double> belt_torque_limit_;
    InputInterface<double> belt_hold_torque_input_;
    InputInterface<bool> belt_wait_zero_velocity_;
    InputInterface<bool> trigger_lock_enable_;
    InputInterface<double> left_belt_velocity_;
    InputInterface<double> right_belt_velocity_;
    InputInterface<int> force_sensor_ch1_, force_sensor_ch2_;

    OutputInterface<double> left_belt_torque_;
    OutputInterface<double> right_belt_torque_;
    OutputInterface<double> trigger_value_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartLaunchSettingV2, rmcs_executor::Component)
