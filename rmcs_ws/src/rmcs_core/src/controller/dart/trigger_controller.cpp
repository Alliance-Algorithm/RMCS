#include "controller/dart/mechanism_fsm.hpp"

#include <cstdint>
#include <limits>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_dart_guidance/msg/mechanism_status.hpp>
#include <rmcs_dart_guidance/msg/trigger_command.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::dart {

class TriggerController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    TriggerController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        using rmcs_dart_guidance::msg::MechanismStatus;
        using rmcs_dart_guidance::msg::TriggerCommand;

        register_input("/dart/trigger/command", command_, false);
        register_input("/dart/trigger/setpoint", setpoint_, false);
        register_output("/dart/trigger/status", status_, MechanismStatus::IDLE);

        int64_t stub_ticks = 50;
        get_parameter("stub_complete_ticks", stub_ticks);
        fsm_.set_stub_complete_ticks(static_cast<uint64_t>(stub_ticks));
    }

    void before_updating() override {
        if (!command_.ready()) {
            command_.make_and_bind_directly(rmcs_dart_guidance::msg::TriggerCommand::IDLE);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/dart/trigger/command\". Set to IDLE.");
        }
        if (!setpoint_.ready()) {
            setpoint_.make_and_bind_directly(std::numeric_limits<double>::quiet_NaN());
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/dart/trigger/setpoint\". Set to NaN.");
        }
    }

    void update() override {
        using rmcs_dart_guidance::msg::TriggerCommand;
        const auto cmd = command_.ready() ? *command_ : TriggerCommand::IDLE;
        *status_ = fsm_.update(cmd, &rmcs_dart_guidance::msg::is_active);
        (void)setpoint_;
    }

private:
    InputInterface<rmcs_dart_guidance::msg::TriggerCommand> command_;
    InputInterface<double> setpoint_;
    OutputInterface<rmcs_dart_guidance::msg::MechanismStatus> status_;
    MechanismFsm<rmcs_dart_guidance::msg::TriggerCommand> fsm_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::TriggerController, rmcs_executor::Component)
