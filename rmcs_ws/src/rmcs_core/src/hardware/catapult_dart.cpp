#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/force_sensor.hpp"
#include "librmcs/client/cboard.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
namespace rmcs_core::hardware {
class CatapultDart
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::client::CBoard {
public:
    CatapultDart()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::client::CBoard{static_cast<int>(get_parameter("usb_pid").as_int())}
        , logger_(get_logger())
        , dart_command_(create_partner_component<DartCommand>(get_component_name() + "_command", *this))
        , dr16_(*this)
        , pitch_motor_(*this, *dart_command_, "/dart/pitch_motor")
        , yaw_motor_(*this, *dart_command_, "/dart/yaw_motor")
        , drive_belt_motor_(
              {*this, *dart_command_, "/dart/drive_belt/left"}, {*this, *dart_command_, "/dart/drive_belt/right"})
        , force_control_motor_(*this, *dart_command_, "/dart/force_control_motor")
        , force_sensor_(*this)
        , transmit_buffer_(*this, 32)
        , event_thread_([this]() { handle_events(); }) {}

    ~CatapultDart() override {
        stop_handling_events();
        event_thread_.join();
    }

    void update() override {}

    void command_update() {
        uint16_t can_commands[4];

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can1_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can2_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        transmit_buffer_.trigger_transmission();
    }

protected:
    // void can1_receive_callback(
    //     uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
    //     uint8_t can_data_length) override {

    //     if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
    //         return;
    // }

    // void can2_receive_callback(
    //     uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
    //     uint8_t can_data_length) override;

    // void uart1_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override;

    // void uart2_receive_callback(const std::byte* data, uint8_t length) override;

    // void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override;

private:
    rclcpp::Logger logger_;

    class DartCommand : public rmcs_executor::Component {
    public:
        explicit DartCommand(CatapultDart& robot)
            : dart(robot) {}

        void update() override { dart.command_update(); }

        CatapultDart& dart;
    };
    std::shared_ptr<DartCommand> dart_command_;

    device::Dr16 dr16_;

    device::DjiMotor pitch_motor_;
    device::DjiMotor yaw_motor_;
    device::DjiMotor drive_belt_motor_[2];
    device::DjiMotor force_control_motor_;

    device::ForceSensor force_sensor_;

    librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
    std::thread event_thread_;
};
} // namespace rmcs_core::hardware