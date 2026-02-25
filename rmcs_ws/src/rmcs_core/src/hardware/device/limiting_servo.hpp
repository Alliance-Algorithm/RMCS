#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>
#include <string>

namespace rmcs_core::hardware::device {

class LimitingServo {
public:
    LimitingServo(rmcs_executor::Component& command_component, const std::string& name) {
        command_component.register_input(name + "/control_angle", control_angle_);
    }

    enum class CalibrateOperation {
        SWITCH_TO_SERVO_MODE,
        SWITCH_TO_MOTOR_MODE,
        MOTOR_FORWARD_MODE,
        MOTOR_REVERSE_MODE,
        MOTOR_RUNTIME_CONTROL,
        MOTOR_DISABLE_CONTROL,
        READ_CURRENT_ANGLE,
    };

    bool calibrate_mode() const { return is_calibrate_mode_; }

    void set_calibrate_mode(bool mode) { is_calibrate_mode_ = mode; }

    std::unique_ptr<std::byte[]> generate_runtime_command(size_t& output_length) {
        size_t size = sizeof(WrightControlAngleCommand);
        std::unique_ptr<std::byte[]> buffer = std::make_unique<std::byte[]>(size);
        WrightControlAngleCommand command;
        command.header[0] = 0xFF;
        command.header[1] = 0xFF;
        command.id = 0x02;
        command.data_length = 0x07;
        command.command_type = 0x03;
        command.command_address = 0x2A;
        command.control_angle[0] = static_cast<uint8_t>(*control_angle_ >> 8);
        command.control_angle[1] = static_cast<uint8_t>(*control_angle_ & 0x00FF);
        command.control_runtime[0] = 0x00;
        command.control_runtime[1] = 0x00;
        command.checksum = command.calculate_checksum();

        std::memcpy(buffer.get(), &command, size);
        output_length = size;

        return buffer;
    }

    std::unique_ptr<std::byte[]>
        generate_calibrate_command(CalibrateOperation calibrate_operation, size_t& output_length) {
        uint8_t* command = nullptr;
        size_t command_size = 0;

        if (calibrate_operation == CalibrateOperation::SWITCH_TO_SERVO_MODE) {
            command_size = sizeof(switch_servo_mode_);
            command = switch_servo_mode_;
        } else if (calibrate_operation == CalibrateOperation::SWITCH_TO_MOTOR_MODE) {
            command_size = sizeof(switch_motor_mode_);
            command = switch_motor_mode_;
        } else if (calibrate_operation == CalibrateOperation::MOTOR_FORWARD_MODE) {
            command_size = sizeof(switch_motor_forward_mode_);
            command = switch_motor_forward_mode_;
        } else if (calibrate_operation == CalibrateOperation::MOTOR_REVERSE_MODE) {
            command_size = sizeof(switch_motor_reverse_mode_);
            command = switch_motor_reverse_mode_;
        } else if (calibrate_operation == CalibrateOperation::MOTOR_RUNTIME_CONTROL) {
            command_size = sizeof(motor_mode_runtime_control_);
            command = motor_mode_runtime_control_;
        } else if (calibrate_operation == CalibrateOperation::MOTOR_DISABLE_CONTROL) {
            command_size = sizeof(motor_mode_disable_control_);
            command = motor_mode_disable_control_;
        } else if (calibrate_operation == CalibrateOperation::READ_CURRENT_ANGLE) {
            command_size = sizeof(read_angle_command_);
            command = read_angle_command_;
        }

        std::unique_ptr<std::byte[]> buffer = std::make_unique<std::byte[]>(command_size);
        std::memcpy(buffer.get(), command, command_size);
        output_length = command_size;

        return buffer;
    }

    static bool calibrate_current_angle(
        const rclcpp::Logger& logger, const std::byte* uart_data, size_t uart_data_length) {
        if (uart_data_length != sizeof(ReadAngleReceiveData)) {
            RCLCPP_INFO(logger, "UART data length not match");
            return false;
        }

        ReadAngleReceiveData package;
        uint8_t checksum;

        std::memcpy(&package, uart_data, sizeof(ReadAngleReceiveData));
        std::memcpy(&checksum, uart_data + sizeof(ReadAngleReceiveData) - 1, sizeof(uint8_t));

        if (package.header[0] != 0xFF || package.header[1] != 0xF5) {
            RCLCPP_INFO(logger, "UART data header not match");
            return false;
        }

        if (checksum != package.calculate_checksum()) {
            RCLCPP_INFO(
                logger, "Checksum error: receive:%x,calc:%x", checksum,
                package.calculate_checksum());
            return false;
        }

        uint16_t current_angle = static_cast<uint16_t>(package.current_angle[0]) << 8
                               | static_cast<uint16_t>(package.current_angle[1]);
        RCLCPP_INFO(logger, "Servo Current Angle: 0x%04X", current_angle);
        return true;
    }

private:
    struct __attribute__((packed)) WrightControlAngleCommand {
        uint8_t header[2];
        uint8_t id;
        uint8_t data_length;
        uint8_t command_type;
        uint8_t command_address;
        uint8_t control_angle[2];
        uint8_t control_runtime[2];
        uint8_t checksum;

        uint8_t calculate_checksum() const {
            uint8_t check = id + data_length + command_type + command_address + control_angle[0]
                          + control_angle[1] + control_runtime[0] + control_runtime[1];
            return ~check;
        }
    };

    struct __attribute__((packed)) ReadAngleReceiveData {
        uint8_t header[2];
        uint8_t id;
        uint8_t data_length;
        uint8_t servo_status;
        uint8_t current_angle[2];
        uint8_t checksum;

        uint8_t calculate_checksum() const {
            uint8_t check = id + data_length + servo_status + current_angle[0] + current_angle[1];
            return ~check;
        }
    };
    uint8_t read_angle_command_[8] = {0xFF, 0xFF, 0x02, 0x04, 0x02, 0x38, 0x02, 0xBD};

    uint8_t switch_servo_mode_[8] = {0xFF, 0xFF, 0x02, 0x04, 0x03, 0x1C, 0x01, 0xD9};
    uint8_t switch_motor_mode_[8] = {0xFF, 0xFF, 0x02, 0x04, 0x03, 0x1C, 0x00, 0xDA};

    uint8_t switch_motor_forward_mode_[8] = {0xFF, 0xFF, 0x02, 0x04, 0x03, 0x1D, 0x00, 0xD9};
    uint8_t switch_motor_reverse_mode_[8] = {0xFF, 0xFF, 0x02, 0x04, 0x03, 0x1D, 0x01, 0xD8};

    uint8_t motor_mode_runtime_control_[9] = {0xFF, 0xFF, 0x02, 0x05, 0x03, 0x41, 0x00, 0x14, 0xA0};
    uint8_t motor_mode_disable_control_[9] = {0xFF, 0xFF, 0x02, 0x05, 0x03, 0x41, 0x00, 0x00, 0xB4};

    rmcs_executor::Component::InputInterface<uint16_t> control_angle_; // 0-4095

    std::atomic<bool> is_calibrate_mode_ = false;
};
} 

// namespace rmcs_core::hardware::device
// #include <atomic>
// #include <cstddef>
// #include <cstdint>
// #include <cstring>
// #include <rclcpp/logger.hpp>
// #include <rclcpp/logging.hpp>
// #include <rmcs_executor/component.hpp>
// #include <string>

// namespace rmcs_core::hardware::device {

// enum class CalibrateOperation {
//     SWITCH_TO_SERVO_MODE,
//     SWITCH_TO_MOTOR_MODE,
//     MOTOR_FORWARD_MODE,
//     MOTOR_REVERSE_MODE,
//     MOTOR_RUNTIME_CONTROL,
//     MOTOR_DISABLE_CONTROL,
//     READ_CURRENT_ANGLE,
// };

// class TriggerServo {
// public:
//     TriggerServo(rmcs_executor::Component& command_component, const std::string& name, uint8_t device_id_ = 0x01) {
//         command_component.register_input(name + "/control_angle", control_angle_, device_id_);
//     }
//     bool calibrate_mode() const { return is_calibrate_mode_; }

//     void set_calibrate_mode(bool mode) { is_calibrate_mode_ = mode; }

//     std::unique_ptr<std::byte[]> generate_runtime_command(size_t& output_length) {
//         size_t size = sizeof(WrightControlAngleCommand);
//         std::unique_ptr<std::byte[]> buffer = std::make_unique<std::byte[]>(size);
//         WrightControlAngleCommand command;
//         command.header[0] = 0xFF;
//         command.header[1] = 0xFF;
//         command.id = device_id_;
//         command.data_length = 0x07;
//         command.command_type = 0x03;
//         command.command_address = 0x2A;
//         command.control_angle[0] = static_cast<uint8_t>(*control_angle_ >> 8);
//         command.control_angle[1] = static_cast<uint8_t>(*control_angle_ & 0x00FF);
//         command.control_runtime[0] = 0x00;
//         command.control_runtime[1] = 0x00;
//         command.checksum = command.calculate_checksum();

//         std::memcpy(buffer.get(), &command, size);
//         output_length = size;

//         // RCLCPP_INFO(
//         //     logger, "%x, %x, %x, %x", *control_angle_, *control_angle_ >> 8, static_cast<uint8_t>(*control_angle_ >>
//         //     8), static_cast<uint8_t>(*control_angle_ & 0x00FF));
//         return buffer;
//     }

//     std::unique_ptr<std::byte[]>
//         generate_calibrate_command(CalibrateOperation calibrate_operation, size_t& output_length) {
//         uint8_t* command = nullptr;
//         size_t command_size = 0;

//         if (calibrate_operation == CalibrateOperation::SWITCH_TO_SERVO_MODE) {
//             command_size = sizeof(switch_servo_mode_);
//             command = switch_servo_mode_;
//         } else if (calibrate_operation == CalibrateOperation::SWITCH_TO_MOTOR_MODE) {
//             command_size = sizeof(switch_motor_mode_);
//             command = switch_motor_mode_;
//         } else if (calibrate_operation == CalibrateOperation::MOTOR_FORWARD_MODE) {
//             command_size = sizeof(switch_motor_forward_mode_);
//             command = switch_motor_forward_mode_;
//         } else if (calibrate_operation == CalibrateOperation::MOTOR_REVERSE_MODE) {
//             command_size = sizeof(switch_motor_reverse_mode_);
//             command = switch_motor_reverse_mode_;
//         } else if (calibrate_operation == CalibrateOperation::MOTOR_RUNTIME_CONTROL) {
//             command_size = sizeof(motor_mode_runtime_control_);
//             command = motor_mode_runtime_control_;
//         } else if (calibrate_operation == CalibrateOperation::MOTOR_DISABLE_CONTROL) {
//             command_size = sizeof(motor_mode_disable_control_);
//             command = motor_mode_disable_control_;
//         } else if (calibrate_operation == CalibrateOperation::READ_CURRENT_ANGLE) {
//             command_size = sizeof(read_angle_command_);
//             command = read_angle_command_;
//         }

//         std::unique_ptr<std::byte[]> buffer = std::make_unique<std::byte[]>(command_size);
//         std::memcpy(buffer.get(), command, command_size);
//         output_length = command_size;

//         return buffer;
//     }

//     bool
//         calibrate_current_angle(const rclcpp::Logger& logger, const std::byte* uart_data, size_t uart_data_length) {
//         if (uart_data_length != sizeof(ReadAngleReceiveData)) {
//             RCLCPP_INFO(logger, "UART data length not match");
//             return false;
//         }

//         ReadAngleReceiveData package;
//         uint8_t checksum;

//         std::memcpy(&package, uart_data, sizeof(ReadAngleReceiveData));
//         std::memcpy(&checksum, uart_data + sizeof(ReadAngleReceiveData) - 1, sizeof(uint8_t));

//         if (package.header[0] != 0xFF || package.header[1] != 0xF5) {
//             RCLCPP_INFO(logger, "UART data header not match");
//             return false;
//         }

//         if (checksum != package.calculate_checksum()) {
//             RCLCPP_INFO(logger, "Checksum error: receive:%x,calc:%x", checksum, package.calculate_checksum());
//             return false;
//         }

//         current_angle_ =
//             static_cast<uint16_t>(package.current_angle[0]) << 8 | static_cast<uint16_t>(package.current_angle[1]);
//         RCLCPP_INFO(logger, "Servo Current Angle: 0x%04X", current_angle_);
//         return true;
//     }

//      uint16_t get_current_angle_() const { return current_angle_; }

// private:
//     uint8_t device_id_;
//     uint16_t current_angle_;

//     struct __attribute__((packed)) WrightControlAngleCommand {
//         uint8_t header[2];
//         uint8_t id;
//         uint8_t data_length;
//         uint8_t command_type;
//         uint8_t command_address;
//         uint8_t control_angle[2];
//         uint8_t control_runtime[2];
//         uint8_t checksum;

//         uint8_t calculate_checksum() const {
//             uint8_t check = id + data_length + command_type + command_address + control_angle[0] + control_angle[1]
//                           + control_runtime[0] + control_runtime[1];
//             return ~check;
//         }
//     };

//     struct __attribute__((packed)) ReadAngleReceiveData {
//         uint8_t header[2];
//         uint8_t id;
//         uint8_t data_length;
//         uint8_t servo_status;
//         uint8_t current_angle[2];
//         uint8_t checksum;

//         uint8_t calculate_checksum() const {
//             uint8_t check = id + data_length + servo_status + current_angle[0] + current_angle[1];
//             return ~check;
//         }
//     };
//     uint8_t read_angle_command_[8] = {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE};

//     uint8_t switch_servo_mode_[8] = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x1C, 0x01, 0xDA};
//     uint8_t switch_motor_mode_[8] = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x1C, 0x00, 0xDB};

//     uint8_t switch_motor_forward_mode_[8] = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x1D, 0x00, 0xDA};
//     uint8_t switch_motor_reverse_mode_[8] = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x1D, 0x01, 0xD9};

//     uint8_t motor_mode_runtime_control_[9] = {0xFF, 0xFF, 0x01, 0x05, 0x03, 0x41, 0x00, 0x14, 0xA1};
//     uint8_t motor_mode_disable_control_[9] = {0xFF, 0xFF, 0x01, 0x05, 0x03, 0x41, 0x00, 0x00, 0xB5};

//     rmcs_executor::Component::InputInterface<uint16_t> control_angle_; // 0-4095

//     std::atomic<bool> is_calibrate_mode_ = false;
// };
// } 
// namespace rmcs_core::hardware::device
// // #include "hardware/device/dji_motor.hpp"
// #include "hardware/device/dr16.hpp"
// #include "hardware/device/force_sensor.hpp"
// #include "hardware/device/trigger_servo.hpp"
// #include "librmcs/client/cboard.hpp"
// #include <cstddef>
// #include <rclcpp/logger.hpp>
// #include <rclcpp/logging.hpp>
// #include <rclcpp/node.hpp>
// #include <rmcs_executor/component.hpp>
// #include <rmcs_msgs/serial_interface.hpp>
// #include <std_msgs/msg/int32.hpp>

// namespace rmcs_core::hardware {
// class CatapultDart
//     : public rmcs_executor::Component
//     , public rclcpp::Node
//     , private librmcs::client::CBoard {
// public:
//     CatapultDart()
//         : Node{
//               get_component_name(),
//               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
//         , librmcs::client::CBoard{static_cast<int>(get_parameter("usb_pid").as_int())}
//         , logger_(get_logger())
//         , dart_command_(
//               create_partner_component<DartCommand>(get_component_name() + "_command", *this))
//         , dr16_(*this)
//         , pitch_motor_(
//               *this, *dart_command_, "/dart/pitch_motor",
//               device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(19.))
//         , yaw_motor_(
//               *this, *dart_command_, "/dart/yaw_motor",
//               device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(19.))
//         , force_control_motor_(
//               *this, *dart_command_, "/dart/force_screw_motor",
//               device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(19.))
//         , drive_belt_motor_(
//               {*this, *dart_command_, "/dart/drive_belt/left",
//                device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(19.)},
//               {*this, *dart_command_, "/dart/drive_belt/right",
//                device::DjiMotor::Config{device::DjiMotor::Type::M3508}
//                    .set_reduction_ratio(19.)
//                    .set_reversed()})
//         , force_sensor_(*this)
//         , trigger_servo_(*dart_command_, "/dart/trigger_servo")
//         , transmit_buffer_(*this, 32)
//         , event_thread_([this]() { handle_events(); }) {

//         force_sensor_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
//             "/force_sensor/calibrate", rclcpp::QoS{0},
//             [this](std_msgs::msg::Int32::UniquePtr&& msg) {
//                 force_sensor_calibrate_subscription_callback(std::move(msg));
//             });

//         trigger_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
//             "/trigger/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
//                 trigger_servo_calibrate_subscription_callback(std::move(msg));
//             });

//         register_output("/referee/serial", referee_serial_);
//         referee_serial_->read = [this](std::byte* buffer, size_t size) {
//             return referee_ring_buffer_receive_.pop_front_multi(
//                 [&buffer](std::byte byte) { *buffer++ = byte; }, size);
//         };
//         referee_serial_->write = [this](const std::byte* buffer, size_t size) {
//             transmit_buffer_.add_uart1_transmission(buffer, size);
//             return size;
//         };
//     }

//     ~CatapultDart() override {
//         stop_handling_events();
//         event_thread_.join();
//     }

//     void update() override {
//         dr16_.update_status();
//         pitch_motor_.update_status();
//         yaw_motor_.update_status();
//         drive_belt_motor_[0].update_status();
//         drive_belt_motor_[1].update_status();
//         force_control_motor_.update_status();
//         force_sensor_.update_status();
//     }

//     void command_update() {
//         uint16_t can_commands[4];

//         if (force_sensor_pub_time_count_++ > 100) {
//             force_sensor_pub_time_count_ = 0;

//             transmit_buffer_.add_can1_transmission(
//                 force_sensor_.get_can_id(device::ForceSensor::Mode::MEASUREMENT, 1),
//                 force_sensor_.generate_read_weight_command());
//         }

//         can_commands[0] = pitch_motor_.generate_command();
//         can_commands[1] = yaw_motor_.generate_command();
//         can_commands[2] = force_control_motor_.generate_command();
//         can_commands[3] = 0;
//         transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

//         can_commands[0] = drive_belt_motor_[0].generate_command();
//         can_commands[1] = drive_belt_motor_[1].generate_command();
//         can_commands[2] = 0;
//         can_commands[3] = 0;
//         transmit_buffer_.add_can2_transmission(0x1FF, std::bit_cast<uint64_t>(can_commands));

//         if (!trigger_servo_.calibrate_mode()) {
//             size_t uart_data_length;
//             std::unique_ptr<std::byte[]> command_buffer =
//                 trigger_servo_.generate_runtime_command(uart_data_length);
//             const auto trigger_servo_uart_data_ptr = command_buffer.get();
//             transmit_buffer_.add_uart2_transmission(trigger_servo_uart_data_ptr, uart_data_length);
//         }

//         transmit_buffer_.trigger_transmission();
//     }

// protected:
//     void can1_receive_callback(
//         uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
//         uint8_t can_data_length) override {

//         if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
//             return;

//         if (can_id == 0x302) {
//             force_sensor_.store_status(can_data);
//         }
//     }

//     void can2_receive_callback(
//         uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
//         uint8_t can_data_length) override {
//         if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
//             return;

//         if (can_id == 0x201) {
//             pitch_motor_.store_status(can_data);
//         } else if (can_id == 0x202) {
//             yaw_motor_.store_status(can_data);
//         } else if (can_id == 0x203) {
//             force_control_motor_.store_status(can_data);
//         } else if (can_id == 0x205) {
//             drive_belt_motor_[0].store_status(can_data);
//         } else if (can_id == 0x206) {
//             drive_belt_motor_[1].store_status(can_data);
//         }
//     }

//     void uart1_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
//         referee_ring_buffer_receive_.emplace_back_multi(
//             [&uart_data](std::byte* storage) { *storage = *uart_data++; }, uart_data_length);
//     }

//     // void uart2_receive_callback(const std::byte* data, uint8_t length) override {
//     //     bool success = trigger_servo_.calibrate_current_angle(logger_, data, length);
//     //     if (!success) {
//     //         RCLCPP_INFO(logger_, "calibrate: uart2 data store failed");
//     //     }

//     //     // std::string hex_string = bytes_to_hex_string(data, length);
//     //     // RCLCPP_INFO(this->get_logger(), "UART2(length: %hhu): [ %s ]", length,
//     //     // hex_string.c_str());
//     // }

//     void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
//         dr16_.store_status(uart_data, uart_data_length);
//     }

// private:
//     void force_sensor_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
//         transmit_buffer_.add_can1_transmission(
//             force_sensor_.get_can_id(device::ForceSensor::Mode::ZERO_CALIBRATE, 1),
//             force_sensor_.generate_zero_calibration_command());
//         RCLCPP_INFO(logger_, "call");
//     }

//     void trigger_servo_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr msg) {
//         /*
//         标定命令格式：
//         ros2 topic pub --rate 2 --times 5 /trigger/calibrate std_msgs/msg/Int32 "{'data':0}"
//         替换data值就行
//         */
//         trigger_servo_.set_calibrate_mode(msg->data);

//         std::unique_ptr<std::byte[]> command_buffer;
//         size_t command_length = 0;
//         if (msg->data == 0) {
//             command_buffer = trigger_servo_.generate_calibrate_command(
//                 device::TriggerServo::CalibrateOperation::SWITCH_TO_SERVO_MODE, command_length);
//         } else if (msg->data == 1) {
//             command_buffer = trigger_servo_.generate_calibrate_command(
//                 device::TriggerServo::CalibrateOperation::SWITCH_TO_MOTOR_MODE, command_length);
//         } else if (msg->data == 2) {
//             command_buffer = trigger_servo_.generate_calibrate_command(
//                 device::TriggerServo::CalibrateOperation::MOTOR_FORWARD_MODE, command_length);
//         } else if (msg->data == 3) {
//             command_buffer = trigger_servo_.generate_calibrate_command(
//                 device::TriggerServo::CalibrateOperation::MOTOR_REVERSE_MODE, command_length);
//         } else if (msg->data == 4) {
//             command_buffer = trigger_servo_.generate_calibrate_command(
//                 device::TriggerServo::CalibrateOperation::MOTOR_RUNTIME_CONTROL, command_length);
//         } else if (msg->data == 5) {
//             command_buffer = trigger_servo_.generate_calibrate_command(
//                 device::TriggerServo::CalibrateOperation::MOTOR_DISABLE_CONTROL, command_length);
//         } else if (msg->data == 6) {
//             command_buffer = trigger_servo_.generate_calibrate_command(
//                 device::TriggerServo::CalibrateOperation::READ_CURRENT_ANGLE, command_length);
//         }

//         const auto trigger_servo_uart_data_ptr = command_buffer.get();
//         transmit_buffer_.add_uart2_transmission(trigger_servo_uart_data_ptr, command_length);

//         std::string hex_string = bytes_to_hex_string(command_buffer.get(), command_length);
//         RCLCPP_INFO(
//             this->get_logger(), "UART2 Pub: (length=%zu)[ %s ]", command_length,
//             hex_string.c_str());
//     }

//     static std::string bytes_to_hex_string(const std::byte* data, size_t size) {
//         if (!data || size == 0) {
//             return "[]";
//         }

//         std::stringstream ss;
//         ss << std::hex << std::uppercase << std::setfill('0');

//         for (size_t i = 0; i < size; ++i) {
//             ss << std::setw(2) << static_cast<int>(data[i]) << " ";
//         }

//         std::string result = ss.str();
//         if (!result.empty() && result.back() == ' ') {
//             result.pop_back();
//         }
//         return result;
//     } // DEBUG TOOL

//     rclcpp::Logger logger_;

//     class DartCommand : public rmcs_executor::Component {
//     public:
//         explicit DartCommand(CatapultDart& robot)
//             : dart_(robot) {}

//         void update() override { dart_.command_update(); }

//         CatapultDart& dart_;
//     };
//     std::shared_ptr<DartCommand> dart_command_;

//     device::Dr16 dr16_;

//     device::DjiMotor pitch_motor_;
//     device::DjiMotor yaw_motor_;
//     device::DjiMotor force_control_motor_;
//     device::DjiMotor drive_belt_motor_[2];

//     device::ForceSensor force_sensor_;
//     int force_sensor_pub_time_count_ = 0; // ForceSensor仅支持10Hz

//     device::TriggerServo trigger_servo_;

//     rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr trigger_calibrate_subscription_;
//     rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr force_sensor_calibrate_subscription_;

//     librmcs::utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
//     OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

//     librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
//     std::thread event_thread_;
// };
// } // namespace rmcs_core::hardware

// #include <pluginlib/class_list_macros.hpp>

// PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::CatapultDart, rmcs_executor::Component)