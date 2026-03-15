#include <algorithm>
#include <cmath>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/dart_slider_status.hpp>

#include "controller/pid/matrix_pid_calculator.hpp"

namespace rmcs_core::controller::dart {

// DartLaunchSettingV2 — DartLaunchSetting 的升级版，新增 LK 升降电机速度控制。
//
// 升降控制逻辑（仅做命令→速度映射，无堵转检测）：
//   UP   → left = +lifting_velocity, right = -lifting_velocity
//   DOWN → left = -lifting_velocity, right = +lifting_velocity
//   WAIT → left = right = 0（电机以小电流维持当前位置）
//
// 堵转检测由 LiftingLkAction 内部完成（直接读速度反馈），不在此处处理。
//
// yaml 新增参数：
//   lifting_velocity  (double, rad/s) — 升降速度，正值=左+右-（平台上升方向），需实测标定
class DartLaunchSettingV2
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartLaunchSettingV2()
        : Node{get_component_name(),
               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
        , belt_pid_{
              get_parameter("b_kp").as_double(),
              get_parameter("b_ki").as_double(),
              get_parameter("b_kd").as_double()} {

        belt_velocity_      = get_parameter("belt_velocity").as_double();
        sync_coefficient_   = get_parameter("sync_coefficient").as_double();
        max_control_torque_ = get_parameter("max_control_torque").as_double();
        trigger_free_value_ = get_parameter("trigger_free_value").as_double();
        trigger_lock_value_ = get_parameter("trigger_lock_value").as_double();
        lifting_velocity_   = get_parameter("lifting_velocity").as_double();
        belt_hold_torque_   = get_parameter("belt_hold_torque").as_double();

        register_input("/dart/manager/belt/command",        belt_command_);
        register_input("/dart/manager/trigger/lock_enable", trigger_lock_enable_);
        register_input("/dart/drive_belt/left/velocity",    left_belt_velocity_);
        register_input("/dart/drive_belt/right/velocity",   right_belt_velocity_);
        register_input("/force_sensor/channel_1/weight",    force_sensor_ch1_);
        register_input("/force_sensor/channel_2/weight",    force_sensor_ch2_);
        register_input("/dart/manager/lifting/command",     lifting_command_);
        // 第一发下降速度缩放因子（shot_count==1 时 0.5，其余 1.0）
        register_input("/dart/manager/belt/down_scale",     belt_down_scale_);

        register_output(
            "/dart/drive_belt/left/control_torque",  left_belt_torque_,  0.0);
        register_output(
            "/dart/drive_belt/right/control_torque", right_belt_torque_, 0.0);
        register_output("/dart/trigger_servo/value", trigger_value_, trigger_lock_value_);
        register_output("/dart/lifting_left/control_velocity",  lifting_left_vel_,  0.0);
        register_output("/dart/lifting_right/control_velocity", lifting_right_vel_, 0.0);
    }

    void update() override {
        double control_velocity = 0.0;
        switch (*belt_command_) {
        case rmcs_msgs::DartSliderStatus::DOWN:
            control_velocity = +belt_velocity_ * (*belt_down_scale_);
            prev_belt_cmd_   = rmcs_msgs::DartSliderStatus::DOWN;
            break;
        case rmcs_msgs::DartSliderStatus::UP:
            control_velocity = -belt_velocity_;
            prev_belt_cmd_   = rmcs_msgs::DartSliderStatus::UP;
            break;
        default:
            control_velocity = 0.0;
            break;
        }
        drive_belt_sync_control(control_velocity);

        *trigger_value_ = *trigger_lock_enable_ ? trigger_lock_value_ : trigger_free_value_;
        // RCLCPP_INFO(logger_,"ch1: %d | ch2: %d",*force_sensor_ch1_,*force_sensor_ch2_);
    }

private:
    void drive_belt_sync_control(double set_velocity) {
        // WAIT 状态：清除 PID 积分；若上一个方向是 DOWN，施加保持扭矩防止滑落
        if (set_velocity == 0.0) {
            belt_pid_.reset();
            if (prev_belt_cmd_ == rmcs_msgs::DartSliderStatus::DOWN) {
                *left_belt_torque_  = +belt_hold_torque_;
                *right_belt_torque_ = +belt_hold_torque_;
            } else {
                *left_belt_torque_  = 0.0;
                *right_belt_torque_ = 0.0;
            }
            return;
        }

        Eigen::Vector2d setpoint_error{
            set_velocity - *left_belt_velocity_,
            set_velocity - *right_belt_velocity_};
        Eigen::Vector2d relative_velocity{
            *left_belt_velocity_  - *right_belt_velocity_,
            *right_belt_velocity_ - *left_belt_velocity_};

        Eigen::Vector2d control_torques =
            belt_pid_.update(setpoint_error) - sync_coefficient_ * relative_velocity;

        *left_belt_torque_ =
            std::clamp(control_torques[0], -max_control_torque_, max_control_torque_);
        *right_belt_torque_ =
            std::clamp(control_torques[1], -max_control_torque_, max_control_torque_);
    }

    rclcpp::Logger logger_;

    pid::MatrixPidCalculator<2> belt_pid_;

    double belt_velocity_;
    double sync_coefficient_;
    double max_control_torque_;
    double trigger_free_value_;
    double trigger_lock_value_;
    double lifting_velocity_;
    double belt_hold_torque_{0.0};

    rmcs_msgs::DartSliderStatus prev_belt_cmd_{rmcs_msgs::DartSliderStatus::WAIT};

    InputInterface<rmcs_msgs::DartSliderStatus> belt_command_;
    InputInterface<bool>   trigger_lock_enable_;
    InputInterface<double> left_belt_velocity_;
    InputInterface<double> right_belt_velocity_;
    InputInterface<int>    force_sensor_ch1_, force_sensor_ch2_;

    InputInterface<rmcs_msgs::DartSliderStatus> lifting_command_;
    InputInterface<double> belt_down_scale_;

    OutputInterface<double> left_belt_torque_;
    OutputInterface<double> right_belt_torque_;
    OutputInterface<double> trigger_value_;
    OutputInterface<double> lifting_left_vel_;
    OutputInterface<double> lifting_right_vel_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::dart::DartLaunchSettingV2, rmcs_executor::Component)
