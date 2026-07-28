#include <chrono>
#include <cstddef>
#include <memory>
#include <span>

#include <librmcs/board/rmcs_board_pro.hpp>
#include <librmcs/data/datas.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/vtm-link/ladar_package_transmit.hpp"

namespace rmcs_core::hardware {

class TestHardware
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::RmcsBoardPro::Callback {
private:
    class Command : public rmcs_executor::Component {
    public:
        explicit Command(TestHardware& hardware)
            : hardware_(hardware) {}

        void update() override { hardware_.command_update(); }

    private:
        TestHardware& hardware_;
    };

public:
    TestHardware()
        : rclcpp::Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
        , command_(create_partner_component<Command>(get_component_name() + "_command", *this))
        , ladar_transmit_(
              *command_, std::chrono::milliseconds{10},
              [this](const std::byte* buffer, size_t size) {
                  board_->start_transmit().uart_transmit(
                      Spec::kUarts.kUart0,
                      {.uart_data = std::span<const std::byte>{buffer, size}});
              },
              logger_) {
        board_ = std::make_unique<librmcs::board::RmcsBoardPro>(
            *this, get_parameter("board_serial").as_string());
        board_->start_transmit().uart_config(Spec::kUarts.kUart0, {.baudrate = 921600});
    }

    void update() override {}

    void command_update() { ladar_transmit_.command_update(); }

private:
    rclcpp::Logger logger_;
    std::shared_ptr<Command> command_;
    std::unique_ptr<librmcs::board::RmcsBoardPro> board_;
    vtm::LadarPackageTransmit ladar_transmit_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::TestHardware, rmcs_executor::Component)
