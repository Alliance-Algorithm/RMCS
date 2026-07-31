#include <algorithm>
#include <cmath>
#include <limits>

#include <eigen3/Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_dart_guidance/msg/mechanism_status.hpp>
#include <rmcs_dart_guidance/msg/yaw_command.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::dart {

class YawController
    : public rmcs_executor::Component
    , public rclcpp::Node {
    using YawCmd = rmcs_dart_guidance::msg::YawCommand;
    using MechStatus = rmcs_dart_guidance::msg::MechanismStatus;

public:
    YawController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/dart/yaw/command", command_, false);
        register_input("/dart/yaw/target-setpoint", target_setpoint_, false);
        register_input("/dart_guidance/camera/target_position", target_position_, false);
        register_input("/dart_guidance/tracker/tracking", tracking_, false);
        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);
        register_input("/remote/joystick/right", joystick_right_, false);

        register_output("/dart/yaw/status", status_, MechStatus::IDLE);
        register_output("/dart/yaw/motor/control_velocity", motor_control_velocity_, kNan);
        register_output("/dart/yaw/motor/control_torque_limit", motor_control_torque_limit_, kNan);

        load_parameters();
    }

    void before_updating() override {
        if (!command_.ready()) {
            command_.make_and_bind_directly(YawCmd::IDLE);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/dart/yaw/command\". Set to IDLE.");
        }
        if (!target_setpoint_.ready()) {
            target_setpoint_.make_and_bind_directly(kNan);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/dart/yaw/target-setpoint\". Set to NaN.");
        }
        if (!target_position_.ready()) {
            target_position_.make_and_bind_directly(cv::Point2i{-1, -1});
            RCLCPP_WARN(
                get_logger(),
                "Failed to fetch \"/dart_guidance/camera/target_position\". Set to invalid.");
        }
        if (!tracking_.ready()) {
            tracking_.make_and_bind_directly(false);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/dart_guidance/tracker/tracking\". Set to false.");
        }
        if (!switch_left_.ready()) {
            switch_left_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/remote/switch/left\". Set to UNKNOWN.");
        }
        if (!switch_right_.ready()) {
            switch_right_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/remote/switch/right\". Set to UNKNOWN.");
        }
        if (!joystick_right_.ready()) {
            joystick_right_.make_and_bind_directly(Eigen::Vector2d::Zero());
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/remote/joystick/right\". Set to zero.");
        }
    }

    void update() override {
        if (manual_mode()) {
            update_manual();
            return;
        }

        const auto cmd = command_.ready() ? *command_ : YawCmd::IDLE;
        const double setpoint = target_setpoint_.ready() ? *target_setpoint_ : kNan;

        MechStatus status = MechStatus::IDLE;
        double target_velocity = kNan;
        double torque_limit = kNan;

        const bool is_active_command = rmcs_dart_guidance::msg::is_active(cmd);
        const bool is_new_command = is_active_command && cmd != active_cmd_;
        update_active_ticks(cmd, is_new_command);

        switch (cmd) {
        case YawCmd::IDLE:
            active_cmd_ = YawCmd::IDLE;
            reset_command_state();
            status = MechStatus::IDLE;
            break;

        case YawCmd::ABORT:
            active_cmd_ = YawCmd::IDLE;
            reset_command_state();
            status = MechStatus::ABORTED;
            break;

        case YawCmd::VISION_AIM:
            torque_limit = yaw_torque_limit_;
            status = handle_vision_aim(is_new_command, setpoint, target_velocity);
            if (status == MechStatus::SUCCEEDED || status == MechStatus::FAILED) {
                target_velocity = kNan;
                torque_limit = kNan;
            }
            break;
        }

        *motor_control_velocity_ = target_velocity;
        *motor_control_torque_limit_ = torque_limit;
        *status_ = status;
    }

private:
    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

    void load_parameters() {
        vision_aim_tolerance_px_ = std::abs(get_parameter_or("vision_aim_tolerance_px", 5.0));

        int64_t settle_ticks = 50;
        get_parameter_or("vision_aim_settle_ticks", settle_ticks, int64_t{50});
        vision_aim_settle_ticks_ = static_cast<int>(std::max<int64_t>(settle_ticks, 1));

        aim_pid_.kp = get_parameter_or("vision_aim_pid_kp", 0.01);
        aim_pid_.ki = get_parameter_or("vision_aim_pid_ki", 0.0);
        aim_pid_.kd = get_parameter_or("vision_aim_pid_kd", 0.0);
        aim_pid_.integral_min =
            get_parameter_or("vision_aim_pid_integral_min", aim_pid_.integral_min);
        aim_pid_.integral_max =
            get_parameter_or("vision_aim_pid_integral_max", aim_pid_.integral_max);
        aim_pid_.output_min = get_parameter_or("vision_aim_pid_output_min", -5.0);
        aim_pid_.output_max = get_parameter_or("vision_aim_pid_output_max", 5.0);
        aim_pid_.reset();

        yaw_torque_limit_ = std::abs(get_parameter_or("yaw_torque_limit", 2.0));
        manual_yaw_velocity_sensitivity_ =
            get_parameter_or("manual_yaw_velocity_sensitivity", 5.0);
    }

    bool manual_mode() const {
        return switch_left_.ready() && *switch_left_ == rmcs_msgs::Switch::UP;
    }

    bool manual_yaw_mode() const {
        return manual_mode() && switch_right_.ready() && *switch_right_ == rmcs_msgs::Switch::DOWN;
    }

    void update_manual() {
        active_cmd_ = YawCmd::IDLE;
        reset_command_state();

        double target_velocity = 0.0;
        if (manual_yaw_mode() && joystick_right_.ready()) {
            target_velocity = manual_yaw_velocity_sensitivity_ * joystick_right_->y();
        }

        *motor_control_velocity_ = target_velocity;
        *motor_control_torque_limit_ = yaw_torque_limit_;
        *status_ = MechStatus::BUSY;
    }

    void update_active_ticks(YawCmd cmd, bool is_new_command) {
        if (!rmcs_dart_guidance::msg::is_active(cmd)) {
            active_ticks_ = 0;
            return;
        }
        if (is_new_command) {
            active_ticks_ = 1;
            return;
        }
        ++active_ticks_;
    }

    void reset_command_state() {
        active_ticks_ = 0;
        settle_ticks_ = 0;
        aim_pid_.reset();
    }

    MechStatus handle_vision_aim(
        bool is_new_command, double setpoint, double& target_velocity) {
        if (is_new_command) {
            active_cmd_ = YawCmd::VISION_AIM;
            settle_ticks_ = 0;
            aim_pid_.reset();
        }

        if (!std::isfinite(setpoint) || !tracking_.ready() || !*tracking_
            || !target_position_.ready() || target_position_->x < 0 || target_position_->y < 0) {
            return MechStatus::FAILED;
        }

        const double error = static_cast<double>(target_position_->x) - setpoint;
        target_velocity = aim_pid_.update(error);

        if (std::abs(error) <= vision_aim_tolerance_px_) {
            ++settle_ticks_;
            if (settle_ticks_ >= vision_aim_settle_ticks_) {
                return MechStatus::SUCCEEDED;
            }
        } else {
            settle_ticks_ = 0;
        }

        return MechStatus::BUSY;
    }

    InputInterface<YawCmd> command_;
    InputInterface<double> target_setpoint_;
    InputInterface<cv::Point2i> target_position_;
    InputInterface<bool> tracking_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<Eigen::Vector2d> joystick_right_;

    OutputInterface<MechStatus> status_;
    OutputInterface<double> motor_control_velocity_;
    OutputInterface<double> motor_control_torque_limit_;

    YawCmd active_cmd_{YawCmd::IDLE};
    int active_ticks_{0};
    int settle_ticks_{0};

    rmcs_core::controller::pid::PidCalculator aim_pid_{0.01, 0.0, 0.0};
    double vision_aim_tolerance_px_{5.0};
    int vision_aim_settle_ticks_{50};
    double yaw_torque_limit_{2.0};
    double manual_yaw_velocity_sensitivity_{5.0};
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::YawController, rmcs_executor::Component)
