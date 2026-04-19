#include "hardware/device/tof.hpp"
#include "hardware/ring_buffer.hpp"
#include <librmcs/agent/c_board.hpp>
#include <librmcs/data/datas.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <string>

namespace rmcs_core::hardware {

class Test
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    Test()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<TestCommand>(get_component_name() + "_command", *this))
        , bottom_board_serial_filter_(
              get_optional_string_parameter(*this, "bottom_board_serial_filter")) {
        bottom_board_ = std::make_unique<BottomBoard>(*this, bottom_board_serial_filter_);
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
    std::string bottom_board_serial_filter_;

    class BottomBoard final : private librmcs::agent::CBoard {
    public:
        friend class Test;
        explicit BottomBoard(Test& test, const std::string& serial_filter = {})
            : librmcs::agent::CBoard(serial_filter)
            , tof_(test, "/tof") {}

        ~BottomBoard() final = default;

        void before_updating() {}

        void update() { tof_.update(); }

        void command_update() {}

    private:
        void uart2_receive_callback(const librmcs::data::UartDataView& data) override {
            auto* uart_data = data.uart_data.data();
            ring_buff.emplace_back_multi(
                [uart_data](std::byte* storage) mutable { *storage = *uart_data++; },
                data.uart_data.size());

            while (ring_buff.front() && ring_buff.readable() >= 16) {
                std::byte rx_data[16];
                std::byte* rx_ptr = rx_data;
                ring_buff.pop_front_multi([&rx_ptr](std::byte storage) { *rx_ptr++ = storage; }, 1);
                if (rx_data[0] != std::byte{0x57}) {
                    continue;
                }

                ring_buff.pop_front_multi(
                    [&rx_ptr](std::byte storage) { *rx_ptr++ = storage; }, 15);
                tof_.store_status(rx_data, 16);
            }
        }

        rmcs_core::hardware::device::Tof tof_;
        RingBuffer<std::byte> ring_buff{16};
    };

    std::unique_ptr<BottomBoard> bottom_board_;

    static std::string get_optional_string_parameter(rclcpp::Node& node, const char* name) {
        std::string value;
        node.get_parameter_or(name, value, std::string{});
        return value;
    }
};

} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>
#include <utility>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Test, rmcs_executor::Component)
