#include "hardware/device/force_sensor_runtime.hpp"
#include "librmcs/client/cboard.hpp"
#include <atomic>
#include <bit>
#include <cstdint>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware {

class ForceSensorTest
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::client::CBoard {

public:
    ForceSensorTest()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::client::CBoard{static_cast<int>(get_parameter("usb_pid").as_int())}
        , robot_command_(create_partner_component<RoboCommand>(get_component_name() + "_command", *this))
        , force_sensor_(*this)
        , transmit_buffer_(*this, 32)
        , event_thread_([this]() { handle_events(); }) {}

    ~ForceSensorTest() override {
        stop_handling_events();
        event_thread_.join();
    }

    void update() override { force_sensor_.update_status(); }

    void command_update() {
        // uint16_t can_commands[4];

        // can_commands[0] = 0;
        // can_commands[1] = 0;
        // can_commands[2] = 0;
        // can_commands[3] = 0;
        // transmit_buffer_.add_can1_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

        // can_commands[0] = 0;
        // can_commands[1] = 0;
        // can_commands[2] = 0;
        // can_commands[3] = 0;
        // transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        // can_commands[0] = 0;
        // can_commands[1] = 0;
        // can_commands[2] = 0;
        // can_commands[3] = 0;
        // transmit_buffer_.add_can2_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

        // can_commands[0] = 0;
        // can_commands[1] = 0;
        // can_commands[2] = 0;
        // can_commands[3] = 0;
        // transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        count_++;
        if (count_ == 100) {
            count_ = 0;
            transmit_buffer_.add_can1_transmission(0x301, std::bit_cast<uint64_t>(force_sensor_.generate_command()));
        }

        transmit_buffer_.trigger_transmission();
    }

    int count_ = 0;

protected:
    void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {

        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;

        if (can_id == 0x302) {
            force_sensor_.store_status(can_data);
            sensor_.store(std::bit_cast<sensor_data>(can_data), std::memory_order_relaxed);
            auto data = sensor_.load(std::memory_order_relaxed);

            RCLCPP_INFO(get_logger(), "ch1: 0x%2X 0x%2X 0x%2X 0x%2X", data.ch1_0, data.ch1_1, data.ch1_2, data.ch1_3);
            RCLCPP_INFO(get_logger(), "ch2: 0x%2X 0x%2X 0x%2X 0x%2X", data.ch2_0, data.ch2_1, data.ch2_2, data.ch2_3);
        }
    }

    struct sensor_data {
        uint8_t ch1_0;
        uint8_t ch1_1;
        uint8_t ch1_2;
        uint8_t ch1_3;
        uint8_t ch2_0;
        uint8_t ch2_1;
        uint8_t ch2_2;
        uint8_t ch2_3;
    };

    std::atomic<sensor_data> sensor_{};

    // void can2_receive_callback(
    //     uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
    //     uint8_t can_data_length) override;

    // void uart1_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override;

    // void uart2_receive_callback(const std::byte* data, uint8_t length) override;

    // void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override;

private:
    class RoboCommand : public rmcs_executor::Component {
    public:
        explicit RoboCommand(ForceSensorTest& robot)
            : robot_(robot) {}

        void update() override { robot_.command_update(); }

        ForceSensorTest& robot_;
    };
    std::shared_ptr<RoboCommand> robot_command_;

    device::ForceSensorRuntime force_sensor_;

    librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
    std::thread event_thread_;
};
} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::ForceSensorTest, rmcs_executor::Component)