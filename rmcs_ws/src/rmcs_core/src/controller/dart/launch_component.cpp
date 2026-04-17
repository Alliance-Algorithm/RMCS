#include <algorithm>
#include <cmath>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/dart_mechanism_command.hpp>
#include <rmcs_msgs/dart_motor_exit_mode.hpp>
#include <rmcs_msgs/dart_servo_command.hpp>

#include "controller/pid/matrix_pid_calculator.hpp"

namespace rmcs_core::controller::dart {

// DartLaunchercontroller 负责将 manager 发布的扳机命令映射为舵机 PWM 值。
class DartTriggerController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartTriggerController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , trigger_free_value_(get_parameter("trigger_free_value").as_double())
        , trigger_lock_value_(get_parameter("trigger_lock_value").as_double()) {
        register_input("/dart_manager/trigger/command", trigger_command_);
        register_output("/dart/trigger_servo/value", trigger_value_, trigger_free_value_);
    }

    void update() override {
        switch (*trigger_command_) {
        case rmcs_msgs::DartServoCommand::FREE: *trigger_value_ = trigger_free_value_; break;
        case rmcs_msgs::DartServoCommand::LOCK: *trigger_value_ = trigger_lock_value_; break;
        case rmcs_msgs::DartServoCommand::WAIT: break;
        }
    }

private:
    double trigger_free_value_;
    double trigger_lock_value_;

    InputInterface<rmcs_msgs::DartServoCommand> trigger_command_;
    OutputInterface<double> trigger_value_;
};

// DartBeltController 负责同步带的闭环和同步控制，并对上游速度目标变化做梯形速度规划。
class DartBeltController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartBeltController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , belt_pid_{
              get_parameter("b_kp").as_double(), get_parameter("b_ki").as_double(),
              get_parameter("b_kd").as_double()}
        , belt_velocity_(get_parameter("belt_velocity").as_double())
        , belt_acceleration_(get_parameter("belt_acceleration").as_double())
        , sync_coefficient_(get_parameter("sync_coefficient").as_double())
        , max_control_torque_(get_parameter("max_control_torque").as_double())
        , default_belt_hold_torque_(get_parameter("belt_hold_torque").as_double()) {
        register_input("/dart_manager/belt/command", belt_command_, false);
        register_input("/dart_manager/belt/target_velocity", belt_target_velocity_, false);
        register_input("/dart_manager/belt/exit_mode", belt_exit_mode_, false);
        register_input("/dart/drive_belt/left/velocity", left_belt_velocity_);
        register_input("/dart/drive_belt/right/velocity", right_belt_velocity_);
        register_input("/predefined/update_rate", update_rate_, false);

        register_output("/dart/drive_belt/left/control_torque", left_belt_torque_, 0.0);
        register_output("/dart/drive_belt/right/control_torque", right_belt_torque_, 0.0);
    }

    void update() override {
        const double requested_velocity = requested_belt_velocity();
        const auto [control_mode, velocity_target] = resolve_control_mode(requested_velocity);

        if (control_mode != BeltControlMode::WAIT_HOLD) {
            retained_velocity_target_ = velocity_target;
        }

        if (control_mode != control_mode_) {
            belt_pid_.reset();
            if (control_mode == BeltControlMode::WAIT_HOLD) {
                reset_trapezoidal_plan();
            }
            control_mode_ = control_mode;
        }

        if (control_mode == BeltControlMode::WAIT_HOLD) {
            apply_hold_torque();
            return;
        }

        update_trapezoidal_target(velocity_target);
        update_planned_velocity();
        drive_belt_sync_control(planned_velocity_, max_control_torque_);
    }

private:
    enum class BeltControlMode {
        MOVE_UP,
        MOVE_DOWN,
        WAIT_ZERO,
        WAIT_HOLD,
    };

    rmcs_msgs::DartMechanismCommand active_belt_command() const {
        if (belt_command_.ready()) {
            return *belt_command_;
        }
        return rmcs_msgs::DartMechanismCommand::WAIT;
    }

    rmcs_msgs::ExitMode active_belt_exit_mode() const {
        if (belt_exit_mode_.ready()) {
            return *belt_exit_mode_;
        }
        return rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY;
    }

    double requested_belt_velocity() const {
        if (belt_target_velocity_.ready()) {
            return sanitize_velocity_magnitude(*belt_target_velocity_);
        }
        return sanitize_velocity_magnitude(belt_velocity_);
    }

    std::pair<BeltControlMode, double> resolve_control_mode(double requested_velocity) {
        switch (active_belt_command()) {
        case rmcs_msgs::DartMechanismCommand::DOWN:
            return {BeltControlMode::MOVE_DOWN, +requested_velocity};
        case rmcs_msgs::DartMechanismCommand::UP:
            return {BeltControlMode::MOVE_UP, -requested_velocity};
        default:
            switch (active_belt_exit_mode()) {
            case rmcs_msgs::ExitMode::WAIT_HOLD_TORQUE: return {BeltControlMode::WAIT_HOLD, 0.0};
            case rmcs_msgs::ExitMode::KEEP: return {control_mode_, retained_velocity_target_};
            case rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY:
            default: return {BeltControlMode::WAIT_ZERO, 0.0};
            }
        }
    }

    double control_dt() const {
        if (update_rate_.ready() && *update_rate_ > 1e-6) {
            return 1.0 / *update_rate_;
        }
        return 1.0 / 1000.0;
    }

    void update_trapezoidal_target(double velocity_target) {
        if (almost_equal(planned_velocity_target_, velocity_target)) {
            return;
        }

        // 上游目标变化时，从当前规划速度续接到新目标，避免速度指令跳变。
        planned_velocity_target_ = velocity_target;
    }

    void update_planned_velocity() {
        planned_velocity_ = approach_value(
            planned_velocity_, planned_velocity_target_, trapezoidal_velocity_delta_limit());
    }

    void reset_trapezoidal_plan() {
        planned_velocity_ = 0.0;
        planned_velocity_target_ = 0.0;
    }

    double trapezoidal_velocity_delta_limit() const {
        if (!std::isfinite(belt_acceleration_) || belt_acceleration_ <= 0.0) {
            return 0.0;
        }

        return belt_acceleration_ * control_dt();
    }

    static double sanitize_velocity_magnitude(double velocity) {
        if (!std::isfinite(velocity)) {
            return 0.0;
        }

        return std::abs(velocity);
    }

    static bool almost_equal(double lhs, double rhs) {
        constexpr double epsilon = 1e-9;
        return std::abs(lhs - rhs) <= epsilon;
    }

    static double approach_value(double current, double target, double max_delta) {
        if (max_delta <= 0.0) {
            return target;
        }

        if (current < target) {
            return std::min(current + max_delta, target);
        }

        if (current > target) {
            return std::max(current - max_delta, target);
        }

        return target;
    }

    void apply_hold_torque() {
        *left_belt_torque_ = default_belt_hold_torque_;
        *right_belt_torque_ = default_belt_hold_torque_;
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
    double belt_acceleration_;
    double sync_coefficient_;
    double max_control_torque_;
    double default_belt_hold_torque_{0.0};
    double planned_velocity_{0.0};
    double planned_velocity_target_{0.0};
    double retained_velocity_target_{0.0};

    BeltControlMode control_mode_{BeltControlMode::WAIT_ZERO};

    InputInterface<rmcs_msgs::DartMechanismCommand> belt_command_;
    InputInterface<double> belt_target_velocity_;
    InputInterface<rmcs_msgs::ExitMode> belt_exit_mode_;
    InputInterface<double> left_belt_velocity_;
    InputInterface<double> right_belt_velocity_;
    InputInterface<double> update_rate_;

    OutputInterface<double> left_belt_torque_;
    OutputInterface<double> right_belt_torque_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartTriggerController, rmcs_executor::Component)
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartBeltController, rmcs_executor::Component)
