#include "controller/dart/mechanism_fsm.hpp"

#include <cstdint>
#include <optional>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_dart_guidance/msg/example_command.hpp>
#include <rmcs_dart_guidance/msg/mechanism_status.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::dart {

class ExampleController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ExampleController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        using rmcs_dart_guidance::msg::ExampleCommand;
        using rmcs_dart_guidance::msg::MechanismStatus;

        register_input("/dart/example/command", command_, false);
        register_output("/dart/example/status", status_, MechanismStatus::IDLE);

        int64_t stub_ticks = 50;
        get_parameter("stub_complete_ticks", stub_ticks);
        fsm_.set_stub_complete_ticks(static_cast<uint64_t>(stub_ticks));
        RCLCPP_INFO(
            get_logger(), "[ExampleController] initialized (stub_complete_ticks=%ld)", stub_ticks);
    }

    void before_updating() override {
        if (!command_.ready()) {
            command_.make_and_bind_directly(rmcs_dart_guidance::msg::ExampleCommand::IDLE);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/dart/example/command\". Set to IDLE.");
        }
    }

    void update() override {
        using rmcs_dart_guidance::msg::ExampleCommand;
        using rmcs_dart_guidance::msg::MechanismStatus;
        using rmcs_dart_guidance::msg::to_string;

        const auto cmd = command_.ready() ? *command_ : ExampleCommand::IDLE;
        if (!last_cmd_.has_value() || *last_cmd_ != cmd) {
            RCLCPP_INFO(
                get_logger(), "[ExampleController] received command: %s", to_string(cmd));
            last_cmd_ = cmd;
        }

        const auto status = fsm_.update(cmd, &rmcs_dart_guidance::msg::is_active);
        if (!last_status_.has_value() || *last_status_ != status) {
            RCLCPP_INFO(
                get_logger(), "[ExampleController] status: %s (cmd=%s)", to_string(status),
                to_string(cmd));
            last_status_ = status;
        }
        *status_ = status;
    }

private:
    InputInterface<rmcs_dart_guidance::msg::ExampleCommand> command_;
    OutputInterface<rmcs_dart_guidance::msg::MechanismStatus> status_;
    MechanismFsm<rmcs_dart_guidance::msg::ExampleCommand> fsm_;
    std::optional<rmcs_dart_guidance::msg::ExampleCommand> last_cmd_;
    std::optional<rmcs_dart_guidance::msg::MechanismStatus> last_status_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::ExampleController, rmcs_executor::Component)
