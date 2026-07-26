#include <cmath>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_dart_guidance/msg/four_dof_command.hpp>
#include <rmcs_dart_guidance/msg/four_z_chassis_command.hpp>
#include <rmcs_dart_guidance/msg/mechanism_status.hpp>
#include <rmcs_executor/component.hpp>

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

        register_output("/dart/chassis/4z/command", four_z_command_, FourZCommand::IDLE);
        register_output("/dart/yaw/motor/control_velocity", yaw_control_velocity_, NAN);
        register_output("/dart/4dof/status", status_, MechStatus::IDLE);
    }

    void update() override {
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
    InputInterface<FourDofCommand> command_;
    InputInterface<MechStatus> four_z_status_;

    OutputInterface<FourZCommand> four_z_command_;
    OutputInterface<double> yaw_control_velocity_;
    OutputInterface<MechStatus> status_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::FourDofController,
                       rmcs_executor::Component)
