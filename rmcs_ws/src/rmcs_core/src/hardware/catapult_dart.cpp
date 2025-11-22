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
        , pitch_motor_(
              *this, *dart_command_, "/dart/pitch_motor",
              device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(19.))
        , yaw_motor_(
              *this, *dart_command_, "/dart/yaw_motor",
              device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(19.))
        , force_control_motor_(
              *this, *dart_command_, "/dart/force_control_motor",
              device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(19.))
        , drive_belt_motor_(
              {*this, *dart_command_, "/dart/drive_belt/left",
               device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(19.)},
              {*this, *dart_command_, "/dart/drive_belt/right",
               device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(19.)})
        , force_sensor_(*this)
        , transmit_buffer_(*this, 32)
        , event_thread_([this]() { handle_events(); }) {}

    ~CatapultDart() override {
        stop_handling_events();
        event_thread_.join();
    }

    void update() override {
        dr16_.update_status();
        pitch_motor_.update_status();
        yaw_motor_.update_status();
        drive_belt_motor_[0].update_status();
        drive_belt_motor_[1].update_status();
        force_control_motor_.update_status();
    }

    void command_update() {
        uint16_t can_commands[4];

        can_commands[0] = pitch_motor_.generate_command();
        can_commands[1] = yaw_motor_.generate_command();
        can_commands[2] = force_control_motor_.generate_command();
        can_commands[3] = 0;
        transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = drive_belt_motor_[0].generate_command();
        can_commands[1] = drive_belt_motor_[1].generate_command();
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can2_transmission(0x1FF, std::bit_cast<uint64_t>(can_commands));

        transmit_buffer_.trigger_transmission();
    }

protected:
    // void can1_receive_callback(
    //     uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
    //     uint8_t can_data_length) override {

    //     if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
    //         return;
    // }

    void can2_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;

        if (can_id == 0x201) {
            pitch_motor_.store_status(can_data);
        } else if (can_id == 0x202) {
            yaw_motor_.store_status(can_data);
        } else if (can_id == 0x203) {
            force_control_motor_.store_status(can_data);
        } else if (can_id == 0x205) {
            drive_belt_motor_[0].generate_command();
        } else if (can_id == 0x206) {
            drive_belt_motor_[1].generate_command();
        }
    }

    // void uart1_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override;

    // void uart2_receive_callback(const std::byte* data, uint8_t length) override;

    void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
        dr16_.store_status(uart_data, uart_data_length);
    }

private:
    rclcpp::Logger logger_;

    class DartCommand : public rmcs_executor::Component {
    public:
        explicit DartCommand(CatapultDart& robot)
            : dart_(robot) {}

        void update() override { dart_.command_update(); }

        CatapultDart& dart_;
    };
    std::shared_ptr<DartCommand> dart_command_;

    device::Dr16 dr16_;

    device::DjiMotor pitch_motor_;
    device::DjiMotor yaw_motor_;
    device::DjiMotor force_control_motor_;
    device::DjiMotor drive_belt_motor_[2];

    device::ForceSensor force_sensor_;

    librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
    std::thread event_thread_;
};
} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::CatapultDart, rmcs_executor::Component)