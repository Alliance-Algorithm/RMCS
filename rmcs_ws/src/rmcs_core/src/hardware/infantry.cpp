#include <memory>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <serial/serial.h>

#include "hardware/device/dji_motor.hpp"
#include "hardware/forwarder/cboard.hpp"

namespace rmcs_core::hardware {

class Infantry
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private forwarder::CBoard {
public:
    Infantry()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , forwarder::CBoard{get_logger()}
        , infantry_command_(
              create_partner_component<InfantryCommand>(get_component_name() + "_command", *this))
        , transmit_buffer_(*this, 16) {

        gimbal_yaw_motor_.configure(
            device::DjiMotorConfig{device::DjiMotorType::GM6020}.set_encoder_zero_point(
                static_cast<int>(get_parameter("yaw_motor_zero_point").as_int())));

        register_output(
            "/gimbal/yaw/control_torque", yaw_motor_control_torque_,
            std::numeric_limits<double>::quiet_NaN());
    }

    void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        auto can_data_bytes = reinterpret_cast<uint8_t*>(&can_data);
        char hex_string[64 * 3 + 1];
        hex_string[0] = '\0';
        for (int i = 0; i < can_data_length; i++)
            sprintf(&hex_string[i * 3], "%02x ", static_cast<uint8_t>(can_data_bytes[i]));
        RCLCPP_INFO(
            get_logger(), "CAN1 receive: 0x%x(%d, %d), %s(%d)", can_id, is_extended_can_id,
            is_remote_transmission, hex_string, can_data_length);
    }

    void can2_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        auto can_data_bytes = reinterpret_cast<uint8_t*>(&can_data);
        char hex_string[64 * 3 + 1];
        hex_string[0] = '\0';
        for (int i = 0; i < can_data_length; i++)
            sprintf(&hex_string[i * 3], "%02x ", static_cast<uint8_t>(can_data_bytes[i]));
        RCLCPP_INFO(
            get_logger(), "CAN2 receive: 0x%x(%d, %d), %s(%d)", can_id, is_extended_can_id,
            is_remote_transmission, hex_string, can_data_length);
    }

    void update() override {
        transmit_buffer_.add_can2_transmission(0x200, 0x01'23'45'67'89'AB'CD'EF, true, true, 6);
        transmit_buffer_.add_can2_transmission(0x201, 0x01'23'45'67'89'AB'CD'EF, true, true, 0);
        transmit_buffer_.add_can2_transmission(0x202, 0x01'23'45'67'89'AB'CD'EF);
        transmit_buffer_.add_can2_transmission(0x203, 0x01'23'45'67'89'AB'CD'EF);
        transmit_buffer_.add_can2_transmission(0x204, 0x01'23'45'67'89'AB'CD'EF);
        transmit_buffer_.add_can2_transmission(0x205, 0x01'23'45'67'89'AB'CD'EF);
        transmit_buffer_.add_can2_transmission(0x206, 0x01'23'45'67'89'AB'CD'EF);
        transmit_buffer_.add_can2_transmission(0x207, 0x01'23'45'67'89'AB'CD'EF);
        transmit_buffer_.add_can2_transmission(0x208, 0x01'23'45'67'89'AB'CD'EF);
        transmit_buffer_.add_can2_transmission(0x209, 0x01'23'45'67'89'AB'CD'EF);
        transmit_buffer_.add_can2_transmission(0x20A, 0x01'23'45'67'89'AB'CD'EF);
        transmit_buffer_.add_can2_transmission(0x20B, 0x01'23'45'67'89'AB'CD'EF);
        transmit_buffer_.add_can2_transmission(0x20C, 0x01'23'45'67'89'AB'CD'EF);
        transmit_buffer_.add_can2_transmission(0x20D, 0x01'23'45'67'89'AB'CD'EF);
        transmit_buffer_.add_can2_transmission(0x20E, 0x01'23'45'67'89'AB'CD'EF);
        transmit_buffer_.add_can2_transmission(0x20F, 0x01'23'45'67'89'AB'CD'EF);
        transmit_buffer_.trigger_transmission();
    }

    void command_update() {}

private:
    class InfantryCommand : public rmcs_executor::Component {
    public:
        explicit InfantryCommand(Infantry& infantry)
            : infantry_(infantry) {}

        void update() override { infantry_.command_update(); }

        Infantry& infantry_;
    };
    std::shared_ptr<InfantryCommand> infantry_command_;

    device::DjiMotor gimbal_yaw_motor_{*this, *infantry_command_, "/gimbal/yaw"};

    OutputInterface<double> yaw_motor_control_torque_;

    forwarder::CBoard::TransmitBuffer transmit_buffer_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Infantry, rmcs_executor::Component)