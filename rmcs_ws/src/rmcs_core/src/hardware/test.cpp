#include "hardware/device/tof.hpp"
#include <librmcs/client/cboard.hpp>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/int32.hpp>
namespace rmcs_core::hardware {

class Test
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    Test()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<TestCommand>(get_component_name() + "_command", *this)) {
        using namespace rmcs_description;
        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_component_, get_logger(),
            static_cast<int>(get_parameter("usb_pid_bottom_board").as_int()));
    }

    ~Test() override = default;

    void command_update() { bottom_board_->command_update(); }

    void before_updating() override { bottom_board_->before_updating(); }
    void update() override { bottom_board_->update(); }

private:
    class TestCommand : public rmcs_executor::Component {
    public:
        explicit TestCommand(Test& test)
            : test_(test) {}

        void update() override { test_.command_update(); }

        Test& test_;
    };

    std::shared_ptr<TestCommand> command_component_;

    class BottomBoard final : private librmcs::client::CBoard {
    public:
        friend class Test;
        explicit BottomBoard(
            Test& test, TestCommand& test_command, rclcpp::Logger logger, int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , logger_(std::move(logger))
            , tof_(test, "/tof")

            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {}

        ~BottomBoard() final {
            stop_handling_events();
            event_thread_.join();
        }
        void before_updating() {}
        void update() {
            //    RCLCPP_INFO(logger_, "Received UART1 data");

            tof_.update();
        }
        void command_update() {}

    private:
        void uart2_receive_callback(const std::byte* data, uint8_t length) override {

            RCLCPP_INFO(logger_, "Received UART2 length:%x",length);
            if (length < 6) {
                RCLCPP_INFO(logger_, "too short length: %x", length);
                return;
            }


            if (std::to_integer<uint8_t>(data[length - 1]) != 0x0A) {
                RCLCPP_INFO(
                    logger_, "Invalid back: 0x%02x", std::to_integer<uint8_t>(data[length - 1]));

                return;
            }

            RCLCPP_INFO(
                logger_, " 0x%02x  0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
                std::to_integer<uint8_t>(data[0]), std::to_integer<uint8_t>(data[1]),
                std::to_integer<uint8_t>(data[2]), std::to_integer<uint8_t>(data[3]),
                std::to_integer<uint8_t>(data[4]), std::to_integer<uint8_t>(data[5]),
                std::to_integer<uint8_t>(data[6]));

            tof_.store_status(data, length);
        }

        rclcpp::Logger logger_;

        rmcs_core::hardware::device::Tof tof_;
        librmcs::utility::RingBuffer<std::byte> ring_buff{16};
        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    };

    std::unique_ptr<BottomBoard> bottom_board_;
};

} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>
#include <utility>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Test, rmcs_executor::Component)