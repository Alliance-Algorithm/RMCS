#include <cstdio>
#include <memory>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <thread>

#include <librmcs/client/cboard.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/tick_timer.hpp>
#include <std_msgs/msg/int32.hpp>

#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"

namespace rmcs_core::hardware {

class ShootingTest
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ShootingTest()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<ShootingTestCommand>(
                  get_component_name() + "_command", *this)) {
        using namespace rmcs_description;

        top_board_ = std::make_unique<TopBoard>(
            *this, *command_component_,
            static_cast<int>(get_parameter("usb_pid_top_board").as_int()));
    }

    ~ShootingTest() override = default;

    void update() override { top_board_->update(); }

    void command_update() { top_board_->command_update(); }

private:
    class ShootingTestCommand : public rmcs_executor::Component {
    public:
        explicit ShootingTestCommand(ShootingTest& shoot)
            : shoot_(shoot) {}

        void update() override { shoot_.command_update(); }

        ShootingTest& shoot_;
    };
    std::shared_ptr<ShootingTestCommand> command_component_;

    class TopBoard final : private librmcs::client::CBoard {
    public:
        friend class ShootingTest;
        explicit TopBoard(ShootingTest& shoot, ShootingTestCommand& shoot_command, int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , logger_(shoot.get_logger())
            , gimbal_friction_wheels_(
                  {shoot, shoot_command, "/gimbal/first_left_friction",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(1.)},
                  {shoot, shoot_command, "/gimbal/first_right_friction",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(1.)})
            , gimbal_bullet_feeder_(
                  shoot, shoot_command, "/gimbal/bullet_feeder",
                  device::LkMotor::Config{device::LkMotor::Type::MG5010E_I10}
                      .set_reversed()
                      .enable_multi_turn_angle())
            , dr16_(shoot)
            , transmit_buffer_(*this, 32)

            , event_thread_([this]() { handle_events(); }) {
            shoot.register_output("/referee/serial", referee_serial_);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_multi(
                    [&buffer](std::byte byte) { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                transmit_buffer_.add_uart1_transmission(buffer, size);
                return size;
            };
        }

        ~TopBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            dr16_.update_status();
            for (auto& motor : gimbal_friction_wheels_)
                motor.update_status();

            gimbal_bullet_feeder_.update_status();
        }

        void command_update() {
            uint16_t batch_commands[2]{};

            for (int i = 0; i < 2; i++)
                batch_commands[i] = gimbal_friction_wheels_[i].generate_command();
            transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint32_t>(batch_commands));

            frequency_control_flag_ = !frequency_control_flag_;
            if (frequency_control_flag_) {
                transmit_buffer_.add_can1_transmission(
                    0x1ff, std::bit_cast<uint32_t>(batch_commands));
            }

            transmit_buffer_.add_can2_transmission(
                0x141, gimbal_bullet_feeder_.generate_torque_command(
                           gimbal_bullet_feeder_.control_torque()));

            transmit_buffer_.trigger_transmission();
        }

    private:
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x201) {
                gimbal_friction_wheels_[0].store_status(can_data);
            } else if (can_id == 0x202) {
                gimbal_friction_wheels_[1].store_status(can_data);
            }
        }

        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x141) {
                gimbal_bullet_feeder_.store_status(can_data);
            }
        }

        void uart1_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            referee_ring_buffer_receive_.emplace_back_multi(
                [&uart_data](std::byte* storage) { *storage = *uart_data++; }, uart_data_length);
        }

        void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            dr16_.store_status(uart_data, uart_data_length);
        }
        rclcpp::Logger logger_;

        device::DjiMotor gimbal_friction_wheels_[2];
        device::LkMotor gimbal_bullet_feeder_;

        bool frequency_control_flag_{false};
        device::Dr16 dr16_;

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;

        librmcs::utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;
    };

    std::unique_ptr<TopBoard> top_board_;

    rmcs_utility::TickTimer temperature_logging_timer_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::ShootingTest, rmcs_executor::Component)