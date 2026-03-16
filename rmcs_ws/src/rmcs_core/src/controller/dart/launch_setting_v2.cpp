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
        // 第一发下降速度缩放因子（shot_count==1 时 0.8，其余 1.0）
        register_input("/dart/manager/belt/down_scale",     belt_down_scale_);
        // 归零模式：true 时将传送带扭矩上限限制到 10%，防止顶住机械限位过热
        register_input("/dart/manager/belt/homing",         belt_homing_mode_);

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
        double torque_limit = *belt_homing_mode_
            ? max_control_torque_ * 0.1
            : max_control_torque_;
        drive_belt_sync_control(control_velocity, torque_limit);

        *trigger_value_ = *trigger_lock_enable_ ? trigger_lock_value_ : trigger_free_value_;
        // RCLCPP_INFO(logger_,"ch1: %d | ch2: %d",*force_sensor_ch1_,*force_sensor_ch2_);

        // 升降电机速度控制（UP/DOWN/WAIT → 速度映射）
        switch (*lifting_command_) {
        case rmcs_msgs::DartSliderStatus::UP:
            *lifting_left_vel_  = +lifting_velocity_;
            *lifting_right_vel_ = -lifting_velocity_;
            break;
        case rmcs_msgs::DartSliderStatus::DOWN:
            *lifting_left_vel_  = -lifting_velocity_;
            *lifting_right_vel_ = +lifting_velocity_;
            break;
        default:
            *lifting_left_vel_  = 0.0;
            *lifting_right_vel_ = 0.0;
            break;
        }
    }

private:
    void drive_belt_sync_control(double set_velocity, double torque_limit) {
        // WAIT 状态：清除 PID 积分，避免残余扭矩
        // 上端（prev=UP）：完全清零，避免顶着限位持续出力
        // 下端（prev=DOWN）：施加保持扭矩，防止重力/弹力导致滑台回弹
        if (set_velocity == 0.0) {
            belt_pid_.reset();
            if (prev_belt_cmd_ == rmcs_msgs::DartSliderStatus::DOWN) {
                *left_belt_torque_  = +belt_hold_torque_;
                *right_belt_torque_ = +belt_hold_torque_;
                RCLCPP_INFO(logger_, "belt_combating");
            } else {
                *left_belt_torque_  = 0.0;
                *right_belt_torque_ = 0.0;
                RCLCPP_INFO(logger_, "belt_init done");
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

        *left_belt_torque_  = std::clamp(control_torques[0], -torque_limit, torque_limit);
        *right_belt_torque_ = std::clamp(control_torques[1], -torque_limit, torque_limit);
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
    InputInterface<bool>   belt_homing_mode_;

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