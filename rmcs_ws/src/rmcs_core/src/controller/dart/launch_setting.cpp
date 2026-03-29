#include <algorithm>
#include <cmath>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/dart_slider_status.hpp>

#include "controller/pid/matrix_pid_calculator.hpp"

namespace rmcs_core::controller::dart {

// DartLaunchSettingV2
//   传送带统一控制组件：
//   - 支持位置控制模式（级联控制：位置 → 速度 → 扭矩）
//   - 支持速度控制模式（UP/DOWN/WAIT 命令）
//   - 梯形速度规划
//   - 双电机同步控制（matrix PID + 相对速度阻尼）
//   - 零点校准功能
//   - 扳机控制
enum class BeltControlMode : uint8_t {
    IDLE = 0,             // 空闲，无控制
    POSITION_CONTROL = 1, // 位置控制模式
    VELOCITY_CONTROL = 2, // 速度控制模式（UP/DOWN）
    HOLD_POSITION = 3,    // 保持位置（高刚度）
    WAIT_ZERO = 4,        // 零速度闭环（低刚度）
    HOLD_TORQUE = 5,      // 保持扭矩模式
};

class DartLaunchSetting
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartLaunchSetting()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , velocity_pid_{
              get_parameter("b_kp").as_double(), get_parameter("b_ki").as_double(),
              get_parameter("b_kd").as_double()} {

        position_kp_ = get_parameter("position_kp").as_double();
        sync_coefficient_ = get_parameter("sync_coefficient").as_double();
        max_control_torque_ = get_parameter("max_control_torque").as_double();
        belt_acceleration_ = get_parameter("belt_acceleration").as_double();
        position_tolerance_ = get_parameter("position_tolerance").as_double();
        default_velocity_ = get_parameter("belt_velocity").as_double();
        trigger_free_value_ = get_parameter("trigger_free_value").as_double();
        trigger_lock_value_ = get_parameter("trigger_lock_value").as_double();
        default_belt_hold_torque_ = get_parameter("belt_hold_torque").as_double();

        register_input("/dart/manager/belt/control_mode", belt_control_mode_, true);
        register_input("/dart/manager/belt/command", belt_command_, true);
        register_input("/dart/manager/belt/target_position", belt_target_position_, true);
        register_input("/dart/manager/belt/target_velocity", belt_target_velocity_, false);
        register_input("/dart/manager/belt/max_velocity", belt_max_velocity_, true);
        register_input("/dart/manager/belt/torque_limit", belt_torque_limit_, true);
        register_input("/dart/manager/belt/hold_torque", belt_hold_torque_input_, true);
        register_input("/dart/manager/belt/wait_zero_velocity", belt_wait_zero_velocity_, false);
        register_input("/dart/manager/belt/zero_calibration", belt_zero_calibration_, false);
        register_input("/dart/manager/trigger/lock_enable", trigger_lock_enable_);

        register_input("/dart/drive_belt/left/angle", left_belt_angle_);
        register_input("/dart/drive_belt/right/angle", right_belt_angle_);
        register_input("/dart/drive_belt/left/velocity", left_belt_velocity_);
        register_input("/dart/drive_belt/right/velocity", right_belt_velocity_);
        register_input("/force_sensor/channel_1/weight", force_sensor_ch1_, false);
        register_input("/force_sensor/channel_2/weight", force_sensor_ch2_, false);

        register_output("/dart/drive_belt/left/control_torque", left_belt_torque_, 0.0);
        register_output("/dart/drive_belt/right/control_torque", right_belt_torque_, 0.0);
        register_output("/dart/trigger_servo/control_angle", trigger_value_, trigger_lock_value_);
        register_output("/dart/belt/position_reached", position_reached_, false);
    }

    void update() override {
        if (belt_zero_calibration_.ready() && *belt_zero_calibration_) {
            calibrate_zero_position();
        }

        BeltControlMode mode = BeltControlMode::IDLE;
        if (belt_control_mode_.ready()) {
            mode = static_cast<BeltControlMode>(*belt_control_mode_);
        }

        if (mode == BeltControlMode::VELOCITY_CONTROL && belt_command_.ready()) {
            mode = process_velocity_command();
        }

        if (mode != control_mode_) {
            velocity_pid_.reset();
            control_mode_ = mode;
            if (control_mode_ == BeltControlMode::HOLD_POSITION) {
                target_hold_position_ = get_avg_position();
            }
        }

        switch (mode) {
        case BeltControlMode::POSITION_CONTROL: position_control(); break;
        case BeltControlMode::VELOCITY_CONTROL: velocity_control(); break;
        case BeltControlMode::HOLD_POSITION: hold_position(); break;
        case BeltControlMode::WAIT_ZERO: wait_zero_velocity(); break;
        case BeltControlMode::HOLD_TORQUE: hold_torque_mode(); break;
        case BeltControlMode::IDLE:
        default: idle_mode(); break;
        }

        *trigger_value_ = *trigger_lock_enable_ ? trigger_lock_value_ : trigger_free_value_;
    }

private:
    // 处理速度控制命令（UP/DOWN/WAIT）
    BeltControlMode process_velocity_command() {
        switch (*belt_command_) {
        case rmcs_msgs::DartSliderStatus::DOWN:
            velocity_command_value_ = +std::abs(
                belt_target_velocity_.ready() ? *belt_target_velocity_ : default_velocity_);
            prev_belt_cmd_ = rmcs_msgs::DartSliderStatus::DOWN;
            return BeltControlMode::VELOCITY_CONTROL;
        case rmcs_msgs::DartSliderStatus::UP:
            velocity_command_value_ = -std::abs(
                belt_target_velocity_.ready() ? *belt_target_velocity_ : default_velocity_);
            prev_belt_cmd_ = rmcs_msgs::DartSliderStatus::UP;
            return BeltControlMode::VELOCITY_CONTROL;
        default:
            return belt_wait_zero_velocity_.ready() && *belt_wait_zero_velocity_
                     ? BeltControlMode::WAIT_ZERO
                     : BeltControlMode::HOLD_TORQUE;
        }
    }

    void calibrate_zero_position() {
        left_zero_offset_ = *left_belt_angle_;
        right_zero_offset_ = *right_belt_angle_;
    }

    double get_left_position() const { return *left_belt_angle_ - left_zero_offset_; }
    double get_right_position() const { return *right_belt_angle_ - right_zero_offset_; }
    double get_avg_position() const { return (get_left_position() + get_right_position()) / 2.0; }

    double compute_profile_velocity(double current_pos, double target_pos, double max_vel) const {
        if (max_vel <= 0.0 || belt_acceleration_ <= 0.0) {
            return 0.0;
        }

        double distance = target_pos - current_pos;
        if (std::abs(distance) <= 1e-9) {
            return 0.0;
        }

        double direction = (distance > 0) ? 1.0 : -1.0;
        double abs_distance = std::abs(distance);

        double decel_distance = (max_vel * max_vel) / (2.0 * belt_acceleration_);

        if (abs_distance > decel_distance) {
            return direction * max_vel;
        } else {
            return direction * std::sqrt(2.0 * belt_acceleration_ * abs_distance);
        }
    }

    double select_torque_limit() const {
        if (!belt_torque_limit_.ready()) {
            return max_control_torque_;
        }
        const double limit = std::abs(*belt_torque_limit_);
        return limit > 1e-9 ? limit : max_control_torque_;
    }

    // 位置控制模式
    void position_control() {
        if (!belt_target_position_.ready() || !belt_max_velocity_.ready()) {
            idle_mode();
            return;
        }

        double target_pos = *belt_target_position_;
        double max_vel = std::abs(*belt_max_velocity_);
        double torque_limit = select_torque_limit();

        // 使用平均位置做速度规划，避免多层位置环叠加导致速度突变
        double avg_pos = get_avg_position();

        // 速度规划（中环）
        double profile_velocity = compute_profile_velocity(avg_pos, target_pos, max_vel);
        double left_vel_setpoint = profile_velocity;
        double right_vel_setpoint = profile_velocity;

        // 调试日志（每100次打印一次）
        static int debug_counter = 0;
        if (++debug_counter >= 100) {
            debug_counter = 0;
            RCLCPP_INFO(
                get_logger(),
                "[position_control] avg_pos=%.4f, target=%.4f, error=%.4f, "
                "profile_vel=%.4f, max_vel=%.4f, left_angle=%.4f, right_angle=%.4f, "
                "left_offset=%.4f, right_offset=%.4f",
                avg_pos, target_pos, std::abs(target_pos - avg_pos), profile_velocity, max_vel,
                *left_belt_angle_, *right_belt_angle_, left_zero_offset_, right_zero_offset_);
        }

        // 同步控制 + 扭矩控制（内环）
        sync_velocity_control(left_vel_setpoint, right_vel_setpoint, torque_limit);

        // 判断是否到达目标位置
        double avg_pos_error = std::abs(target_pos - avg_pos);
        *position_reached_ = (avg_pos_error < position_tolerance_);
    }

    // 速度控制模式
    void velocity_control() {
        double torque_limit = select_torque_limit();
        sync_velocity_control(velocity_command_value_, velocity_command_value_, torque_limit);
        *position_reached_ = false;
    }

    // 保持位置模式（高刚度）
    void hold_position() {
        double left_pos = get_left_position();
        double right_pos = get_right_position();

        // 使用当前位置作为目标，高增益保持
        double left_vel_setpoint = position_kp_ * 2.0 * (target_hold_position_ - left_pos);
        double right_vel_setpoint = position_kp_ * 2.0 * (target_hold_position_ - right_pos);

        sync_velocity_control(left_vel_setpoint, right_vel_setpoint, max_control_torque_);
        *position_reached_ = true;
    }

    // 零速度闭环模式（低刚度）
    void wait_zero_velocity() {
        sync_velocity_control(0.0, 0.0, max_control_torque_ * 0.5);
        *position_reached_ = true;
    }

    // 保持扭矩模式
    void hold_torque_mode() {
        double hold_torque = 0.0;
        double torque_limit = select_torque_limit();
        if (belt_hold_torque_input_.ready()) {
            hold_torque = *belt_hold_torque_input_;
        } else if (prev_belt_cmd_ == rmcs_msgs::DartSliderStatus::DOWN) {
            hold_torque = default_belt_hold_torque_;
        }
        hold_torque = std::clamp(hold_torque, -torque_limit, torque_limit);

        *left_belt_torque_ = hold_torque;
        *right_belt_torque_ = hold_torque;
        *position_reached_ = true;
    }

    // 空闲模式
    void idle_mode() {
        *left_belt_torque_ = 0.0;
        *right_belt_torque_ = 0.0;
        *position_reached_ = false;
    }

    // 同步速度控制（带速度限制）
    void sync_velocity_control(
        double left_vel_setpoint, double right_vel_setpoint, double torque_limit) {

        // 速度限制：如果超速，将速度设定值限制为当前速度（停止加速）
        const double velocity_limit_factor = 1.2;
        double left_vel_limit = std::abs(left_vel_setpoint) * velocity_limit_factor;
        double right_vel_limit = std::abs(right_vel_setpoint) * velocity_limit_factor;

        // 检查是否超速，如果超速则限制速度设定值
        if (std::abs(*left_belt_velocity_) > left_vel_limit) {
            // 超速：将设定值限制为当前速度的符号 × 限制值
            left_vel_setpoint = std::copysign(left_vel_limit, left_vel_setpoint);

            static int left_overspeed_counter = 0;
            if (++left_overspeed_counter >= 100) {
                left_overspeed_counter = 0;
                RCLCPP_WARN(
                    get_logger(),
                    "[sync_velocity_control] LEFT OVERSPEED: vel=%.4f, limit=%.4f, clamped setpoint=%.4f",
                    *left_belt_velocity_, left_vel_limit, left_vel_setpoint);
            }
        }

        if (std::abs(*right_belt_velocity_) > right_vel_limit) {
            // 超速：将设定值限制为当前速度的符号 × 限制值
            right_vel_setpoint = std::copysign(right_vel_limit, right_vel_setpoint);

            static int right_overspeed_counter = 0;
            if (++right_overspeed_counter >= 100) {
                right_overspeed_counter = 0;
                RCLCPP_WARN(
                    get_logger(),
                    "[sync_velocity_control] RIGHT OVERSPEED: vel=%.4f, limit=%.4f, clamped setpoint=%.4f",
                    *right_belt_velocity_, right_vel_limit, right_vel_setpoint);
            }
        }

        // 正常 PID 控制（使用可能被限制后的速度设定值）
        Eigen::Vector2d vel_error{
            left_vel_setpoint - *left_belt_velocity_, right_vel_setpoint - *right_belt_velocity_};
        Eigen::Vector2d relative_velocity{
            *left_belt_velocity_ - *right_belt_velocity_,
            *right_belt_velocity_ - *left_belt_velocity_};

        Eigen::Vector2d control_torques =
            velocity_pid_.update(vel_error) - sync_coefficient_ * relative_velocity;

        *left_belt_torque_ = std::clamp(control_torques[0], -torque_limit, torque_limit);
        *right_belt_torque_ = std::clamp(control_torques[1], -torque_limit, torque_limit);
    }

    // 参数
    double position_kp_;              // 位置环比例增益
    double sync_coefficient_;         // 同步阻尼系数
    double max_control_torque_;       // N⋅m - 最大扭矩限制
    double belt_acceleration_;        // rad/s² - 加速度
    double position_tolerance_;       // rad - 位置误差阈值
    double default_velocity_;         // rad/s - 默认速度
    double trigger_free_value_;       // 扳机释放值
    double trigger_lock_value_;       // 扳机锁定值
    double default_belt_hold_torque_; // N⋅m - 默认保持扭矩

    // 状态变量
    double left_zero_offset_{0.0};       // 左电机零点偏移（rad）
    double right_zero_offset_{0.0};      // 右电机零点偏移（rad）
    double target_hold_position_{0.0};   // 保持模式的目标位置（rad）
    double velocity_command_value_{0.0}; // 速度控制命令值
    rmcs_msgs::DartSliderStatus prev_belt_cmd_{rmcs_msgs::DartSliderStatus::WAIT};
    BeltControlMode control_mode_{BeltControlMode::IDLE};

    // PID 控制器
    pid::MatrixPidCalculator<2> velocity_pid_;

    InputInterface<uint8_t> belt_control_mode_;
    InputInterface<rmcs_msgs::DartSliderStatus> belt_command_;
    InputInterface<double> belt_target_position_;
    InputInterface<double> belt_target_velocity_;
    InputInterface<double> belt_max_velocity_;
    InputInterface<double> belt_torque_limit_;
    InputInterface<double> belt_hold_torque_input_;
    InputInterface<bool> belt_wait_zero_velocity_;
    InputInterface<bool> belt_zero_calibration_;
    InputInterface<bool> trigger_lock_enable_;
    InputInterface<double> left_belt_angle_;
    InputInterface<double> right_belt_angle_;
    InputInterface<double> left_belt_velocity_;
    InputInterface<double> right_belt_velocity_;
    InputInterface<int> force_sensor_ch1_, force_sensor_ch2_;

    OutputInterface<double> left_belt_torque_;
    OutputInterface<double> right_belt_torque_;
    OutputInterface<double> trigger_value_;
    OutputInterface<bool> position_reached_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartLaunchSetting, rmcs_executor::Component)
