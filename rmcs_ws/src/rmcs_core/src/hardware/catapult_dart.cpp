#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/force_sensor_runtime.hpp"
#include "hardware/device/trigger_servo.hpp"
#include "librmcs/client/cboard.hpp"
#include <cstddef>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/int32.hpp>

namespace rmcs_core::hardware {
class CatapultDart
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::client::CBoard {
public:
    CatapultDart()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::client::CBoard{static_cast<int>(get_parameter("usb_pid").as_int())}
        , logger_(get_logger())
        , dart_command_(
              create_partner_component<DartCommand>(get_component_name() + "_command", *this))
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
               device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                   .set_reduction_ratio(19.)
                   .set_reversed()})
        , force_sensor_(*this)
        , trigger_servo_(*dart_command_, "/dart/trigger_servo")
        , transmit_buffer_(*this, 32)
        , event_thread_([this]() { handle_events(); }) {

        trigger_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/trigger/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                trigger_servo_calibrate_subscription_callback(std::move(msg));
            });
    }

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
        force_sensor_.update_status();
    }

    void command_update() {
        uint16_t can_commands[4];

        if (pub_time_count_++ > 100) {
            transmit_buffer_.add_can1_transmission(
                0x301, std::bit_cast<uint64_t>(force_sensor_.generate_command()));
            pub_time_count_ = 0;
        }

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

        if (!trigger_servo_.calibrate_mode()) {
            size_t uart_data_length;
            std::unique_ptr<std::byte[]> command_buffer =
                trigger_servo_.generate_runtime_command(uart_data_length);
            const auto trigger_servo_uart_data_ptr = command_buffer.get();
            transmit_buffer_.add_uart2_transmission(trigger_servo_uart_data_ptr, uart_data_length);

            // std::string hex_string = bytes_to_hex_string(command_buffer.get(), uart_data_length);
            // RCLCPP_INFO(
            //     this->get_logger(), "UART2(length: %zu): [ %s ]", uart_data_length,
            //     hex_string.c_str());
        }

        transmit_buffer_.trigger_transmission();
    }
    int pub_time_count_ = 0;

protected:
    void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {

        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;

        if (can_id == 0x302) {
            force_sensor_.store_status(can_data);
        }
    }

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
            drive_belt_motor_[0].store_status(can_data);
        } else if (can_id == 0x206) {
            drive_belt_motor_[1].store_status(can_data);
        }
    }

    // void uart1_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override;

    void uart2_receive_callback(const std::byte* data, uint8_t length) override {
        bool success = trigger_servo_.calibrate_current_angle(logger_, data, length);
        if (!success) {
            RCLCPP_INFO(logger_, "calibrate: uart2 data store failed");
        }

        // std::string hex_string = bytes_to_hex_string(data, length);
        // RCLCPP_INFO(this->get_logger(), "UART2(length: %hhu): [ %s ]", length,
        // hex_string.c_str());
    }

    void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
        dr16_.store_status(uart_data, uart_data_length);
    }

private:
    void trigger_servo_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr msg) {
        /*
        标定命令格式：
        ros2 topic pub --rate 2 --times 5 /trigger/calibrate std_msgs/msg/Int32 "{'data':0}"
        替换data值就行
        */
        trigger_servo_.set_calibrate_mode(msg->data);

        std::unique_ptr<std::byte[]> command_buffer;
        size_t command_length = 0;
        if (msg->data == 0) {
            command_buffer = trigger_servo_.generate_calibrate_command(
                device::CalibrateOperation::SWITCH_TO_SERVO_MODE, command_length);
        } else if (msg->data == 1) {
            command_buffer = trigger_servo_.generate_calibrate_command(
                device::CalibrateOperation::SWITCH_TO_MOTOR_MODE, command_length);
        } else if (msg->data == 2) {
            command_buffer = trigger_servo_.generate_calibrate_command(
                device::CalibrateOperation::MOTOR_FORWARD_MODE, command_length);
        } else if (msg->data == 3) {
            command_buffer = trigger_servo_.generate_calibrate_command(
                device::CalibrateOperation::MOTOR_REVERSE_MODE, command_length);
        } else if (msg->data == 4) {
            command_buffer = trigger_servo_.generate_calibrate_command(
                device::CalibrateOperation::MOTOR_RUNTIME_CONTROL, command_length);
        } else if (msg->data == 5) {
            command_buffer = trigger_servo_.generate_calibrate_command(
                device::CalibrateOperation::MOTOR_DISABLE_CONTROL, command_length);
        } else if (msg->data == 6) {
            command_buffer = trigger_servo_.generate_calibrate_command(
                device::CalibrateOperation::READ_CURRENT_ANGLE, command_length);
        }

        const auto trigger_servo_uart_data_ptr = command_buffer.get();
        transmit_buffer_.add_uart2_transmission(trigger_servo_uart_data_ptr, command_length);

        std::string hex_string = bytes_to_hex_string(command_buffer.get(), command_length);
        RCLCPP_INFO(
            this->get_logger(), "UART2 Pub: (length=%zu)[ %s ]", command_length,
            hex_string.c_str());
    }

    static std::string bytes_to_hex_string(const std::byte* data, size_t size) {
        if (!data || size == 0) {
            return "[]";
        }

        std::stringstream ss;
        ss << std::hex << std::uppercase << std::setfill('0');

        for (size_t i = 0; i < size; ++i) {
            ss << std::setw(2) << static_cast<int>(data[i]) << " ";
        }

        std::string result = ss.str();
        if (!result.empty() && result.back() == ' ') {
            result.pop_back();
        }
        return result;
    }

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

    device::ForceSensorRuntime force_sensor_;
    device::TriggerServo trigger_servo_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr trigger_calibrate_subscription_;

    librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
    std::thread event_thread_;
};
} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::CatapultDart, rmcs_executor::Component)