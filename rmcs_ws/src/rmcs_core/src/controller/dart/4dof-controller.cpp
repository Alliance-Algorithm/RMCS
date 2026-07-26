#include <cmath>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_dart_guidance/msg/four_dof_command.hpp>
#include <rmcs_dart_guidance/msg/four_z_chassis_command.hpp>
#include <rmcs_dart_guidance/msg/mechanism_status.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::dart {

class FourDofController
    : public rmcs_executor::Component
    , public rclcpp::Node {
    using FourDofCommand = rmcs_dart_guidance::msg::FourDofCommand;
    using FourZCommand = rmcs_dart_guidance::msg::FourZChassisCommand;
    using MechStatus = rmcs_dart_guidance::msg::MechanismStatus;

public:
    FourDofController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/dart/4dof/command", command_, false);
        register_input("/dart/chassis/4z/status", four_z_status_, false);
        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);
        register_input("/remote/joystick/right", joystick_right_, false);

        register_output("/dart/chassis/4z/command", four_z_command_, FourZCommand::IDLE);
        register_output("/dart/yaw/motor/control_velocity", yaw_control_velocity_, NAN);
        register_output("/dart/4dof/status", status_, MechStatus::IDLE);

        get_parameter_or("manual_yaw_velocity_sensitivity", manual_yaw_velocity_sensitivity_, 0.0);
    }

    void before_updating() override {
        if (!command_.ready()) {
            command_.make_and_bind_directly(FourDofCommand::IDLE);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/dart/4dof/command\". Set to IDLE.");
        }
        if (!four_z_status_.ready()) {
            four_z_status_.make_and_bind_directly(MechStatus::IDLE);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/dart/chassis/4z/status\". Set to IDLE.");
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

        const auto command = command_.ready() ? *command_ : FourDofCommand::IDLE;

        FourZCommand chassis_command = FourZCommand::IDLE;
        double yaw_velocity = NAN;
        MechStatus status = MechStatus::IDLE;

        switch (command) {
        case FourDofCommand::IDLE:
            chassis_command = FourZCommand::IDLE;
            yaw_velocity = NAN;
            status = MechStatus::IDLE;
            break;
        case FourDofCommand::ABORT:
            chassis_command = FourZCommand::ABORT;
            yaw_velocity = NAN;
            status = MechStatus::ABORTED;
            break;
        case FourDofCommand::CALIBRATE_BOTTOM:
            chassis_command = FourZCommand::CALIBRATE_BOTTOM;
            yaw_velocity = 0.0;
            status = four_z_status_.ready() ? *four_z_status_ : MechStatus::BUSY;
            break;
        case FourDofCommand::LEVEL_ZERO:
            chassis_command = FourZCommand::LEVEL_ZERO;
            yaw_velocity = 0.0;
            status = four_z_status_.ready() ? *four_z_status_ : MechStatus::BUSY;
            break;
        case FourDofCommand::DOWN:
            chassis_command = FourZCommand::DOWN;
            yaw_velocity = 0.0;
            status = four_z_status_.ready() ? *four_z_status_ : MechStatus::BUSY;
            break;
        }

        *four_z_command_ = chassis_command;
        *yaw_control_velocity_ = yaw_velocity;
        *status_ = status;
    }

private:
    bool manual_mode() const {
        return switch_left_.ready() && *switch_left_ == rmcs_msgs::Switch::UP;
    }

    bool manual_yaw_mode() const {
        return manual_mode() && switch_right_.ready() && *switch_right_ == rmcs_msgs::Switch::UP;
    }

    void update_manual() {
        double yaw_velocity = 0.0;
        if (manual_yaw_mode() && joystick_right_.ready()) {
            yaw_velocity = manual_yaw_velocity_sensitivity_ * joystick_right_->y();
        }
        *four_z_command_ = FourZCommand::IDLE;
        *yaw_control_velocity_ = yaw_velocity;
        *status_ = MechStatus::BUSY;
    }

    InputInterface<FourDofCommand> command_;
    InputInterface<MechStatus> four_z_status_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<Eigen::Vector2d> joystick_right_;

    OutputInterface<FourZCommand> four_z_command_;
    OutputInterface<double> yaw_control_velocity_;
    OutputInterface<MechStatus> status_;

    double manual_yaw_velocity_sensitivity_ = 0.0;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::FourDofController,
                       rmcs_executor::Component)
