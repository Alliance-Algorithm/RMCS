#include <algorithm>
#include <cmath>
#include <cstdint>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/dart_mechanism_command.hpp>
#include <rmcs_msgs/dart_motor_exit_mode.hpp>

#include "controller/pid/matrix_pid_calculator.hpp"

namespace rmcs_core::controller::dart {

// DartBeltController 负责同步带的闭环和同步控制，并对速度目标增加梯形加速规划。
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

        if (control_mode != control_mode_) {
            belt_pid_.reset();
            if (control_mode == BeltControlMode::WAIT_HOLD) {
                planned_velocity_ = 0.0;
            }
            control_mode_ = control_mode;
        }

        if (control_mode == BeltControlMode::WAIT_HOLD) {
            apply_hold_torque();
            return;
        }

        update_planned_velocity(velocity_target);
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

    double requested_belt_velocity() const {
        if (belt_target_velocity_.ready()) {
            return std::abs(*belt_target_velocity_);
        }
        return belt_velocity_;
    }

    std::pair<BeltControlMode, double> resolve_control_mode(double requested_velocity) {
        switch (active_belt_command()) {
        case rmcs_msgs::DartMechanismCommand::DOWN:
            return {BeltControlMode::MOVE_DOWN, +requested_velocity};
        case rmcs_msgs::DartMechanismCommand::UP:
            return {BeltControlMode::MOVE_UP, -requested_velocity};
        default:
            if (belt_exit_mode_.ready()) {
                return {
                    *belt_exit_mode_ == rmcs_msgs::ExitMode::WAIT_HOLD_TORQUE
                        ? BeltControlMode::WAIT_HOLD
                        : BeltControlMode::WAIT_ZERO,
                    0.0};
            }
            return {BeltControlMode::WAIT_ZERO, 0.0};
        }
    }

    double control_dt() const {
        if (update_rate_.ready() && *update_rate_ > 1e-6) {
            return 1.0 / *update_rate_;
        }
        return 1.0 / 1000.0;
    }

    void update_planned_velocity(double target_velocity) {
        const double max_delta = std::max(0.0, belt_acceleration_) * control_dt();
        planned_velocity_ = approach_value(planned_velocity_, target_velocity, max_delta);
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

// DartBeltStatus 负责同步带堵转检测，输出 manager 所需的 arrive_flag。
class DartBeltStatus
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartBeltStatus()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , stall_velocity_threshold_(get_parameter("stall_velocity_threshold").as_double())
        , stall_torque_threshold_(get_parameter("stall_torque_threshold").as_double())
        , stall_confirm_ticks_(static_cast<uint64_t>(get_parameter("stall_confirm_ticks").as_int()))
        , stall_min_run_ticks_(
              static_cast<uint64_t>(get_parameter("stall_min_run_ticks").as_int())) {
        register_input("/dart_manager/belt/command", belt_command_, false);
        register_input("/dart/drive_belt/left/velocity", belt_left_velocity_);
        register_input("/dart/drive_belt/left/torque", belt_left_torque_);
        register_input("/dart/drive_belt/right/velocity", belt_right_velocity_);
        register_input("/dart/drive_belt/right/torque", belt_right_torque_);

        register_output("/dart_manager/belt/arrive_flag", belt_arrive_flag_, false);
    }

    void update() override {
        const rmcs_msgs::DartMechanismCommand command = active_belt_command();

        if (command != prev_belt_command_) {
            running_ticks_ = 0;
            stall_counter_ = 0;
            set_arrive_flag(false);
            prev_belt_command_ = command;
        }

        const bool is_running_command = command == rmcs_msgs::DartMechanismCommand::UP
                                     || command == rmcs_msgs::DartMechanismCommand::DOWN;
        if (!is_running_command) {
            running_ticks_ = 0;
            stall_counter_ = 0;
            set_arrive_flag(false);
            return;
        }

        ++running_ticks_;
        if (running_ticks_ <= stall_min_run_ticks_) {
            set_arrive_flag(false);
            return;
        }

        const double avg_velocity =
            (std::abs(*belt_left_velocity_) + std::abs(*belt_right_velocity_)) / 2.0;
        const bool torque_active = std::abs(*belt_left_torque_) > stall_torque_threshold_
                                || std::abs(*belt_right_torque_) > stall_torque_threshold_;

        if (avg_velocity < stall_velocity_threshold_ && torque_active) {
            ++stall_counter_;
            set_arrive_flag(stall_counter_ >= stall_confirm_ticks_);
            return;
        }

        stall_counter_ = 0;
        set_arrive_flag(false);
    }

private:
    rmcs_msgs::DartMechanismCommand active_belt_command() const {
        if (belt_command_.ready()) {
            return *belt_command_;
        }
        return rmcs_msgs::DartMechanismCommand::WAIT;
    }

    void set_arrive_flag(bool flag) { *belt_arrive_flag_ = flag; }

    double stall_velocity_threshold_;
    double stall_torque_threshold_;
    uint64_t stall_confirm_ticks_;
    uint64_t stall_min_run_ticks_;

    rmcs_msgs::DartMechanismCommand prev_belt_command_{rmcs_msgs::DartMechanismCommand::WAIT};
    uint64_t running_ticks_{0};
    uint64_t stall_counter_{0};

    InputInterface<rmcs_msgs::DartMechanismCommand> belt_command_;
    InputInterface<double> belt_left_velocity_;
    InputInterface<double> belt_left_torque_;
    InputInterface<double> belt_right_velocity_;
    InputInterface<double> belt_right_torque_;

    OutputInterface<bool> belt_arrive_flag_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartBeltController, rmcs_executor::Component)
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartBeltStatus, rmcs_executor::Component)
