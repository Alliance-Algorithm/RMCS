#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/tof.hpp"
#include <cstdint>
#include <librmcs/client/cboard.hpp>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/int32.hpp>
namespace rmcs_core::hardware {

class LK_Robot
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    LK_Robot()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<LK_Robot_command>(
                  get_component_name() + "_command", *this)) {
        using namespace rmcs_description;
        bottom_board_ = std::make_unique<right_board>(
            *this, *command_component_, get_logger(),
            static_cast<int>(get_parameter("right_board_usb_pid").as_int()));
    }

    ~LK_Robot() override = default;

    void command() { bottom_board_->command(); }
    void update() override { bottom_board_->update(); }

private:
    class LK_Robot_command : public rmcs_executor::Component {
    public:
        explicit LK_Robot_command(LK_Robot& LK_Robot)
            : LK_Robot_(LK_Robot) {}

        void update() override { LK_Robot_.command(); }

        LK_Robot& LK_Robot_;
    };

    std::shared_ptr<LK_Robot_command> command_component_;

    class right_board final : private librmcs::client::CBoard {
    public:
        friend class LK_Robot;
        explicit right_board(
            LK_Robot& LK_Robot, LK_Robot_command& LK_Robot_command, rclcpp::Logger logger,
            int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , logger_(std::move(logger))

            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); })
            , dr16_(LK_Robot)
            , Steering_Motors(
                  {LK_Robot, LK_Robot_command, "/steering/steering/rf"},
                  {LK_Robot, LK_Robot_command, "/steering/steering/rb"})
            , Wheel_Motors(
                  {LK_Robot, LK_Robot_command, "/steering/wheel/rf"},
                  {LK_Robot, LK_Robot_command, "/steering/wheel/rb"}) {
            Wheel_Motors[0].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.set_reduction_ratio(11.0));
            Wheel_Motors[1].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.set_reduction_ratio(11.0));
            Steering_Motors[0].configure(
                device::LKMotorConfig{device::LKMotorType::MHF7015}
                    .set_gear_ratio(1.0)
                    .set_encoder_zero_point(
                        static_cast<int>(
                            LK_Robot.get_parameter("right_forward_zero_point").as_int()))
                    .enable_multi_turn_angle());
            Steering_Motors[1].configure(
                device::LKMotorConfig{device::LKMotorType::MHF7015}
                    .set_gear_ratio(1.0)
                    .set_encoder_zero_point(
                        static_cast<int>(LK_Robot.get_parameter("right_back_zero_point").as_int()))
                    .enable_multi_turn_angle());

            uint64_t command;
            command = device::LKMotor::lk_quest_command();
            transmit_buffer_.add_can1_transmission(0x141, std::bit_cast<uint64_t>(command));
            transmit_buffer_.add_can1_transmission(0x142, std::bit_cast<uint64_t>(command));
            transmit_buffer_.trigger_transmission();
        }

        ~right_board() final {
            uint16_t command[4] = {0, 0, 0, 0};

            transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(command));
            transmit_buffer_.add_can1_transmission(0x141, std::bit_cast<uint64_t>(command));
            transmit_buffer_.add_can1_transmission(0x142, std::bit_cast<uint64_t>(command));
            transmit_buffer_.trigger_transmission();
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            dr16_.update();
            Steering_Motors[0].update();
            Steering_Motors[1].update();
            Wheel_Motors[0].update();
            Wheel_Motors[1].update();
        }
        void command() {
            uint16_t command[4];
            uint64_t lk_command;
            command[0] = Wheel_Motors[0].generate_command();
            command[1] = Wheel_Motors[1].generate_command();
            command[2] = 0;
            command[3] = 0;
            transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(command));
            lk_command = Steering_Motors[0].generate_torque_command();
            transmit_buffer_.add_can1_transmission(0x141, std::bit_cast<uint64_t>(lk_command));
            lk_command = Steering_Motors[1].generate_torque_command();
            transmit_buffer_.add_can1_transmission(0x142, std::bit_cast<uint64_t>(lk_command));
            transmit_buffer_.trigger_transmission();
        }

    protected:
        void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            dr16_.store_status(uart_data, uart_data_length);
        }
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x141) {
                Steering_Motors[0].store_status(can_data);
            } else if (can_id == 0x142) {
                Steering_Motors[1].store_status(can_data);
            } else if (can_id == 0x201) {
                Wheel_Motors[0].store_status(can_data);
            } else if (can_id == 0x202) {
                Wheel_Motors[1].store_status(can_data);
            }
        }

    private:
        rclcpp::Logger logger_;

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
        device::Dr16 dr16_;
        device::LKMotor Steering_Motors[2];
        device::DjiMotor Wheel_Motors[2];
    };

    std::unique_ptr<right_board> bottom_board_;
};

} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>
#include <utility>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::LK_Robot, rmcs_executor::Component)