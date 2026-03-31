#include <algorithm>
#include <cmath>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/dart_slider_status.hpp>

#include "controller/pid/matrix_pid_calculator.hpp"

namespace rmcs_core::controller::dart {

// DartLaunchSetting
//   传送带速度控制组件：
//   - 速度控制模式（UP/DOWN/WAIT 命令）
//   - 双电机同步控制（matrix PID + 相对速度阻尼）
//   - 零点校准功能
//   - 扳机控制
enum class BeltControlMode : uint8_t {
    IDLE = 0,             // 空闲，无控制
    VELOCITY_CONTROL = 1, // 速度控制模式（UP/DOWN）
    WAIT_ZERO = 2,        // 零速度闭环（低刚度）
    HOLD_TORQUE = 3,      // 保持扭矩模式
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

        sync_coefficient_ = get_parameter("sync_coefficient").as_double();
        max_control_torque_ = get_parameter("max_control_torque").as_double();
        position_tolerance_ = get_parameter("position_tolerance").as_double();
        default_velocity_ = get_parameter("belt_velocity").as_double();
        trigger_free_value_ = get_parameter("trigger_free_value").as_double();
        trigger_lock_value_ = get_parameter("trigger_lock_value").as_double();
        default_belt_hold_torque_ = get_parameter("belt_hold_torque").as_double();

        register_input("/dart/manager/belt/command", belt_command_, true);
        register_input("/dart/manager/belt/target_position", belt_target_position_, false);
        register_input("/dart/manager/belt/target_velocity", belt_target_velocity_, false);
        register_input("/dart/manager/belt/torque_limit", belt_torque_limit_, true);
        register_input("/dart/manager/belt/hold_torque", belt_hold_torque_input_, true);
        register_input("/dart/manager/belt/torque_offset", belt_torque_offset_, false);
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
        if (belt_command_.ready()) {
            mode = process_velocity_command();
        }

        if (mode != control_mode_) {
            velocity_pid_.reset();
            control_mode_ = mode;
        }

        switch (mode) {
        case BeltControlMode::VELOCITY_CONTROL: velocity_control(); break;
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

    double select_torque_limit() const {
        if (!belt_torque_limit_.ready()) {
            return max_control_torque_;
        }
        const double limit = std::abs(*belt_torque_limit_);
        return limit > 1e-9 ? limit : max_control_torque_;
    }

    // 速度控制模式（读取角度判断是否到达目标）
    void velocity_control() {
        double torque_limit = select_torque_limit();
        sync_velocity_control(velocity_command_value_, velocity_command_value_, torque_limit);

        // 如果提供了目标位置，检查是否到达
        if (belt_target_position_.ready()) {
            double left_pos = get_left_position();
            double right_pos = get_right_position();
            double target_pos = *belt_target_position_;

            // 分别计算左右电机的位置误差（保留符号）
            double left_error = target_pos - left_pos;
            double right_error = target_pos - right_pos;

            // 左电机到达判断
            bool left_within_tolerance = std::abs(left_error) < position_tolerance_;
            bool left_overshot = false;
            if (prev_belt_cmd_ == rmcs_msgs::DartSliderStatus::DOWN) {
                left_overshot = (left_error < 0);  // 下行越界
            } else if (prev_belt_cmd_ == rmcs_msgs::DartSliderStatus::UP) {
                left_overshot = (left_error > 0);  // 上行越界
            }
            bool left_reached = left_within_tolerance || left_overshot;

            // 右电机到达判断
            bool right_within_tolerance = std::abs(right_error) < position_tolerance_;
            bool right_overshot = false;
            if (prev_belt_cmd_ == rmcs_msgs::DartSliderStatus::DOWN) {
                right_overshot = (right_error < 0);  // 下行越界
            } else if (prev_belt_cmd_ == rmcs_msgs::DartSliderStatus::UP) {
                right_overshot = (right_error > 0);  // 上行越界
            }
            bool right_reached = right_within_tolerance || right_overshot;

            // 只有当左右电机都到达目标位置时，才认为到达
            *position_reached_ = left_reached && right_reached;

            // 调试日志（每100次更新打印一次）
            static int log_counter = 0;
            if (++log_counter >= 100) {
                log_counter = 0;
                RCLCPP_DEBUG(
                    get_logger(),
                    "[velocity_control] L: pos=%.4f err=%.4f reach=%d | R: pos=%.4f err=%.4f "
                    "reach=%d | target=%.4f, both_reached=%d",
                    left_pos, left_error, left_reached, right_pos, right_error, right_reached,
                    target_pos, *position_reached_);
            }
        } else {
            *position_reached_ = false;
        }
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
        const double velocity_limit_factor = 1.3;  // 增大容差，从1.2提升到1.3
        const double min_velocity_for_overspeed_check = 1.0;  // rad/s - 低于此值不检测超速（从0.5提升到1.0）

        // 左电机超速检测（只在目标速度较大时检测，避免零附近误报）
        if (std::abs(left_vel_setpoint) > min_velocity_for_overspeed_check) {
            double left_vel_limit = std::abs(left_vel_setpoint) * velocity_limit_factor;

            // 检查是否超速，如果超速则限制速度设定值
            if (std::abs(*left_belt_velocity_) > left_vel_limit) {
                // 超速：将设定值限制为当前速度的符号 × 限制值
                left_vel_setpoint = std::copysign(left_vel_limit, left_vel_setpoint);

                static int left_overspeed_counter = 0;
                if (++left_overspeed_counter >= 100) {
                    left_overspeed_counter = 0;
                    RCLCPP_WARN(
                        get_logger(),
                        "[sync_velocity_control] LEFT OVERSPEED: vel=%.4f, limit=%.4f, clamped "
                        "setpoint=%.4f",
                        *left_belt_velocity_, left_vel_limit, left_vel_setpoint);
                }
            }
        }

        // 右电机超速检测（只在目标速度较大时检测，避免零附近误报）
        if (std::abs(right_vel_setpoint) > min_velocity_for_overspeed_check) {
            double right_vel_limit = std::abs(right_vel_setpoint) * velocity_limit_factor;

            // 检查是否超速，如果超速则限制速度设定值
            if (std::abs(*right_belt_velocity_) > right_vel_limit) {
                // 超速：将设定值限制为当前速度的符号 × 限制值
                right_vel_setpoint = std::copysign(right_vel_limit, right_vel_setpoint);

                static int right_overspeed_counter = 0;
                if (++right_overspeed_counter >= 100) {
                    right_overspeed_counter = 0;
                    RCLCPP_WARN(
                        get_logger(),
                        "[sync_velocity_control] RIGHT OVERSPEED: vel=%.4f, limit=%.4f, clamped "
                        "setpoint=%.4f",
                        *right_belt_velocity_, right_vel_limit, right_vel_setpoint);
                }
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

        // 添加常态力矩偏移（用于减速阶段补偿负载）
        double torque_offset = belt_torque_offset_.ready() ? *belt_torque_offset_ : 0.0;
        control_torques[0] += torque_offset;
        control_torques[1] += torque_offset;

        *left_belt_torque_ = std::clamp(control_torques[0], -torque_limit, torque_limit);
        *right_belt_torque_ = std::clamp(control_torques[1], -torque_limit, torque_limit);
    }

    // 参数
    double sync_coefficient_;         // 同步阻尼系数
    double max_control_torque_;       // N⋅m - 最大扭矩限制
    double position_tolerance_;       // rad - 位置误差阈值
    double default_velocity_;         // rad/s - 默认速度
    double trigger_free_value_;       // 扳机释放值
    double trigger_lock_value_;       // 扳机锁定值
    double default_belt_hold_torque_; // N⋅m - 默认保持扭矩

    // 状态变量
    double left_zero_offset_{0.0};       // 左电机零点偏移（rad）
    double right_zero_offset_{0.0};      // 右电机零点偏移（rad）
    double velocity_command_value_{0.0}; // 速度控制命令值
    rmcs_msgs::DartSliderStatus prev_belt_cmd_{rmcs_msgs::DartSliderStatus::WAIT};
    BeltControlMode control_mode_{BeltControlMode::IDLE};

    pid::MatrixPidCalculator<2> velocity_pid_;

    InputInterface<rmcs_msgs::DartSliderStatus> belt_command_;
    InputInterface<double> belt_target_position_;
    InputInterface<double> belt_target_velocity_;
    InputInterface<double> belt_torque_limit_;
    InputInterface<double> belt_hold_torque_input_;
    InputInterface<double> belt_torque_offset_;
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
