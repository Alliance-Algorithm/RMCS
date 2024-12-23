#include <cstdint>
#include <memory>

#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/imu.hpp"
#include "hardware/forwarder/cboard.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware {

class DartLauncher
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private forwarder::CBoard {
public:
    DartLauncher()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , forwarder::CBoard(static_cast<uint16_t>(get_parameter("usb_pid").as_int()), get_logger())
        , logger_(get_logger())
        , dart_command_(
              create_partner_component<DartCommand>(get_component_name() + "_command", *this))
        , transmit_buffer_(*this, 16) {
        using namespace device;

        friction_motors_[0].configure(
            DjiMotorConfig{DjiMotorType::M3508}.reverse().set_reduction_ratio(1.));
        friction_motors_[1].configure(
            DjiMotorConfig{DjiMotorType::M3508}.reverse().set_reduction_ratio(1.));
        friction_motors_[2].configure(DjiMotorConfig{DjiMotorType::M3508}.set_reduction_ratio(1.));
        friction_motors_[3].configure(DjiMotorConfig{DjiMotorType::M3508}.set_reduction_ratio(1.));

        Conveyor_motor_.configure(DjiMotorConfig{DjiMotorType::M3508}.set_reduction_ratio(1.));

        yaw_motor_.configure(DjiMotorConfig{DjiMotorType::M2006}.enable_multi_turn_angle());
        pitch_left_motor.configure(DjiMotorConfig{DjiMotorType::M2006}.enable_multi_turn_angle());
        pitch_right_motor.configure(DjiMotorConfig{DjiMotorType::M2006}.enable_multi_turn_angle());
    }

    void update() override {
        update_motors();
        dr16_.update();
    }

    void command_update() {
        uint16_t can_commands[4];

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can1_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = pitch_left_motor.generate_command();
        can_commands[1] = pitch_right_motor.generate_command();
        can_commands[2] = yaw_motor_.generate_command();
        can_commands[3] = 0;
        transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = Conveyor_motor_.generate_command();
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can2_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = friction_motors_[0].generate_command();
        can_commands[1] = friction_motors_[1].generate_command();
        can_commands[2] = friction_motors_[2].generate_command();
        can_commands[3] = friction_motors_[3].generate_command();
        transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        transmit_buffer_.trigger_transmission();
    }

private:
    void update_motors() {
        using namespace rmcs_description;

        for (auto& motor : friction_motors_)
            motor.update();
        Conveyor_motor_.update();
        pitch_left_motor.update();
        pitch_right_motor.update();
        yaw_motor_.update();
        update_imu();
    }

    void update_imu() {}

protected:
    void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;

        if (can_id == 0x201) {
            auto& motor = pitch_left_motor;
            motor.store_status(can_data);
        } else if (can_id == 0x202) {
            auto& motor = pitch_right_motor;
            motor.store_status(can_data);
        } else if (can_id == 0x203) {
            auto& motor = yaw_motor_;
            motor.store_status(can_data);
        }
        // callback_update(1, can_id, 2, can_data);
    }

    void can2_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;

        if (can_id == 0x201) {
            auto& motor = friction_motors_[0];
            motor.store_status(can_data);
        } else if (can_id == 0x202) {
            auto& motor = friction_motors_[1];
            motor.store_status(can_data);
        } else if (can_id == 0x203) {
            auto& motor = friction_motors_[2];
            motor.store_status(can_data);
        } else if (can_id == 0x204) {
            auto& motor = friction_motors_[3];
            motor.store_status(can_data);
        } else if (can_id == 0x205) {
            auto& motor = Conveyor_motor_;
            motor.update();
        }
        callback_update(2, can_id, 3, can_data);
    }

    // void uart1_receive_callback();
    // void uart2_receive_callback();

    void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
        dr16_.store_status(uart_data, uart_data_length);
    }

    void callback_update(int can, uint32_t can_id, uint32_t motor_id, uint64_t can_data) {
        if (can_id != 0x200 + motor_id)
            return;
        can_callback callback_data = std::bit_cast<can_callback>(can_data);
        uint16_t angle             = (callback_data.angle_high << 8) | callback_data.angle_low;
        uint16_t speed             = (callback_data.speed_high << 8) | callback_data.speed_low;
        // double currrent            = (callback_data.current_high << 8) |
        // callback_data.current_low;
        int temperature = callback_data.temperature;

        RCLCPP_INFO(logger_, "can%d,angle:%4d,speed:%5d,temp:%2d", can, angle, speed, temperature);
    }

private:
    rclcpp::Logger logger_;

    class DartCommand : public rmcs_executor::Component {
    public:
        explicit DartCommand(DartLauncher& dart)
            : dart_(dart) {}

        void update() override { dart_.command_update(); }

        DartLauncher& dart_;
    };
    std::shared_ptr<DartCommand> dart_command_;

    device::DjiMotor friction_motors_[4]{
        {*this, *dart_command_, "/dart/friction_lf"},
        {*this, *dart_command_, "/dart/friction_lb"},
        {*this, *dart_command_, "/dart/friction_rb"},
        {*this, *dart_command_, "/dart/friction_rf"}
    };
    device::DjiMotor Conveyor_motor_{*this, *dart_command_, "/dart/conveyor"};

    device::DjiMotor yaw_motor_{*this, *dart_command_, "/dart/yaw"};
    device::DjiMotor pitch_left_motor{*this, *dart_command_, "/dart/pitch_left"};
    device::DjiMotor pitch_right_motor{*this, *dart_command_, "/dart/pitch_right"};

    device::Dr16 dr16_{*this};

    forwarder::CBoard::TransmitBuffer transmit_buffer_;

    struct can_callback {
        uint8_t angle_high;
        uint8_t angle_low;
        uint8_t speed_high;
        uint8_t speed_low;
        uint8_t current_high;
        uint8_t current_low;
        uint8_t temperature;
        uint8_t others;
    };

    device::Imu imu_;
};
} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DartLauncher, rmcs_executor::Component)