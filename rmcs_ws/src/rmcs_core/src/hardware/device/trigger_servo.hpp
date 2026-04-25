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
class TriggerServo {
public:
    TriggerServo(rmcs_executor::Component& command_component, const std::string& name, uint8_t id)
        : id_(id) {
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
        READ_SERVO_ID,
    };

    bool calibrate_mode() const { return is_calibrate_mode_; }
    void set_calibrate_mode(bool mode) { is_calibrate_mode_ = mode; }
    uint16_t get_target_angle() const { return *control_angle_; }

    std::unique_ptr<std::byte[]> generate_runtime_command(size_t& output_length) {
        size_t size = sizeof(WrightControlAngleCommand);
        std::unique_ptr<std::byte[]> buffer = std::make_unique<std::byte[]>(size);
        WrightControlAngleCommand command;
        command.header[0] = 0xFF;
        command.header[1] = 0xFF;
        command.id = id_;
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

    static std::unique_ptr<std::byte[]> generate_sync_run_command(
        size_t& output_length, uint8_t cowork_id_one, uint8_t cowork_id_two,
        uint16_t control_angle_one, uint16_t control_angle_two, uint16_t runtime_one,
        uint16_t runtime_two) {
        size_t size = sizeof(WrightSyncControlAngleCommand);
        std::unique_ptr<std::byte[]> buffer = std::make_unique<std::byte[]>(size);
        WrightSyncControlAngleCommand command;
        command.header[0] = 0xFF;
        command.header[1] = 0xFF;
        command.id = 0xFE;
        command.data_length = 0x0E;
        command.command_type = 0x83;
        command.command_address = 0x2A;
        command.command_address_length = 0x04;
        command.cowork_id_one = cowork_id_one;
        command.control_angle_one[0] = static_cast<uint8_t>(control_angle_one >> 8);
        command.control_angle_one[1] = static_cast<uint8_t>(control_angle_one & 0x00FF);
        command.runtime_one[0] = static_cast<uint8_t>(runtime_one >> 8);
        command.runtime_one[1] = static_cast<uint8_t>(runtime_one & 0x00FF);
        command.cowork_id_two = cowork_id_two;
        command.control_angle_two[0] = static_cast<uint8_t>(control_angle_two >> 8);
        command.control_angle_two[1] = static_cast<uint8_t>(control_angle_two & 0x00FF);
        command.runtime_two[0] = static_cast<uint8_t>(runtime_two >> 8);
        command.runtime_two[1] = static_cast<uint8_t>(runtime_two & 0x00FF);
        command.checksum = command.calculate_checksum();

        std::memcpy(buffer.get(), &command, size);
        output_length = size;
        return buffer;
    }

    std::unique_ptr<std::byte[]>
        generate_calibrate_command(CalibrateOperation calibrate_operation, size_t& output_length) {
        switch (calibrate_operation) {
        case CalibrateOperation::SWITCH_TO_SERVO_MODE: {
            SwitchServoModeCommand command;
            command.id = id_;
            command.checksum = command.calculate_checksum();
            return copy_to_buffer(command, output_length);
        }
        case CalibrateOperation::SWITCH_TO_MOTOR_MODE: {
            SwitchMotorModeCommand command;
            command.id = id_;
            command.checksum = command.calculate_checksum();
            return copy_to_buffer(command, output_length);
        }
        case CalibrateOperation::MOTOR_FORWARD_MODE: {
            MotorForwardModeCommand command;
            command.id = id_;
            command.checksum = command.calculate_checksum();
            return copy_to_buffer(command, output_length);
        }
        case CalibrateOperation::MOTOR_REVERSE_MODE: {
            MotorReverseModeCommand command;
            command.id = id_;
            command.checksum = command.calculate_checksum();
            return copy_to_buffer(command, output_length);
        }
        case CalibrateOperation::MOTOR_RUNTIME_CONTROL: {
            MotorRuntimeControlCommand command;
            command.id = id_;
            command.checksum = command.calculate_checksum();
            return copy_to_buffer(command, output_length);
        }
        case CalibrateOperation::MOTOR_DISABLE_CONTROL: {
            MotorDisableControlCommand command;
            command.id = id_;
            command.checksum = command.calculate_checksum();
            return copy_to_buffer(command, output_length);
        }
        case CalibrateOperation::READ_CURRENT_ANGLE: {
            ReadAngleCommand command;
            command.id = id_;
            command.checksum = command.calculate_checksum();
            return copy_to_buffer(command, output_length);
        }
        case CalibrateOperation::READ_SERVO_ID: {
            ReadServoIDCommand command;
            command.checksum = command.calculate_checksum();
            return copy_to_buffer(command, output_length);
        }
        default: output_length = 0; return nullptr;
        }
    }

    std::pair<bool, uint16_t> calibrate_current_angle(
        const rclcpp::Logger& logger, const std::byte* uart_data, size_t uart_data_length) {
        if (uart_data_length != sizeof(ReadAngleReceiveData)) {
            RCLCPP_INFO(logger, "UART data length not match");
            return {false, 0x0000};
        }

        ReadAngleReceiveData package;
        uint8_t checksum;

        std::memcpy(&package, uart_data, sizeof(ReadAngleReceiveData));
        std::memcpy(&checksum, uart_data + sizeof(ReadAngleReceiveData) - 1, sizeof(uint8_t));

        if (package.header[0] != 0xFF || package.header[1] != 0xF5) {
            RCLCPP_INFO(logger, "UART data header not match");
            return {false, 0x0000};
        }

        if (package.id != id_) {
            RCLCPP_INFO(logger, "id not match");
            return {false, 0x0000};
        }
        if (checksum != package.calculate_checksum()) {
            RCLCPP_INFO(
                logger, "Checksum error: receive:%x,calc:%x", checksum,
                package.calculate_checksum());
            return {false, 0x0000};
        }

        uint16_t current_angle = static_cast<uint16_t>(package.current_angle[0]) << 8
                               | static_cast<uint16_t>(package.current_angle[1]);
        current_angle_.store(current_angle, std::memory_order_release);

        RCLCPP_INFO(logger, "%d Current Angle: 0x%04X", id_, current_angle);
        return {true, current_angle};
    }

    std::pair<bool, uint16_t> fixable_calibrate_current_angle(
        const rclcpp::Logger& logger, const std::byte* uart_data, size_t uart_data_length,
        uint16_t& out_angle) {
        if (uart_data_length < 7) {
            RCLCPP_WARN(
                logger, "UART data too short: %zu bytes (need at least 7)", uart_data_length);
            return {false, 0x0000};
        }

        if (static_cast<uint8_t>(uart_data[0]) != 0xFF
            || static_cast<uint8_t>(uart_data[1]) != 0xF5) {
            RCLCPP_DEBUG(
                logger, "UART header mismatch: %02X %02X", static_cast<uint8_t>(uart_data[0]),
                static_cast<uint8_t>(uart_data[1]));
            return {false, 0x0000};
        }

        uint8_t id = static_cast<uint8_t>(uart_data[2]);

        if (uart_data_length >= 8) {
            uint8_t data_length = static_cast<uint8_t>(uart_data[3]);
            uint8_t servo_status = static_cast<uint8_t>(uart_data[4]);
            uint8_t angle_high = static_cast<uint8_t>(uart_data[5]);
            uint8_t angle_low = static_cast<uint8_t>(uart_data[6]);
            uint8_t received_checksum = static_cast<uint8_t>(uart_data[7]);

            uint8_t calculated_checksum =
                ~(id + data_length + servo_status + angle_high + angle_low);
            if (received_checksum != calculated_checksum) {
                RCLCPP_WARN(
                    logger, "Checksum error: received 0x%02X, calculated 0x%02X", received_checksum,
                    calculated_checksum);
            }
        }

        uint16_t angle = static_cast<uint16_t>(static_cast<uint8_t>(uart_data[5])) << 8
                       | static_cast<uint16_t>(static_cast<uint8_t>(uart_data[6]));
        out_angle = angle;

        current_angle_.store(angle, std::memory_order_release);

        RCLCPP_DEBUG(logger, "Servo ID 0x%02X current angle: 0x%04X", id, angle);
        return {true, angle};
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

    struct __attribute__((packed)) WrightSyncControlAngleCommand {
        uint8_t header[2];
        uint8_t id;
        uint8_t data_length;
        uint8_t command_type;
        uint8_t command_address;
        uint8_t command_address_length;
        uint8_t cowork_id_one;
        uint8_t control_angle_one[2];
        uint8_t runtime_one[2];
        uint8_t cowork_id_two;
        uint8_t control_angle_two[2];
        uint8_t runtime_two[2];
        uint8_t checksum;

        uint8_t calculate_checksum() const {
            uint8_t check = id + data_length + command_type + command_address
                          + command_address_length + cowork_id_one + control_angle_one[0]
                          + control_angle_one[1] + runtime_one[0] + runtime_one[1] + cowork_id_two
                          + control_angle_two[0] + control_angle_two[1] + runtime_two[0]
                          + runtime_two[1];
            return ~check;
        }
    };
    struct __attribute__((packed)) SwitchServoModeCommand {
        uint8_t header[2] = {0xFF, 0xFF};
        uint8_t id;
        uint8_t data_length = 0x04;
        uint8_t command_type = 0x03;
        uint8_t command_address = 0x1C;
        uint8_t data = 0x01;
        uint8_t checksum;

        uint8_t calculate_checksum() const {
            return ~(id + data_length + command_type + command_address + data);
        }
    };

    struct __attribute__((packed)) SwitchMotorModeCommand {
        uint8_t header[2] = {0xFF, 0xFF};
        uint8_t id;
        uint8_t data_length = 0x04;
        uint8_t command_type = 0x03;
        uint8_t command_address = 0x1C;
        uint8_t data = 0x00;
        uint8_t checksum;

        uint8_t calculate_checksum() const {
            return ~(id + data_length + command_type + command_address + data);
        }
    };

    struct __attribute__((packed)) MotorForwardModeCommand {
        uint8_t header[2] = {0xFF, 0xFF};
        uint8_t id;
        uint8_t data_length = 0x04;
        uint8_t command_type = 0x03;
        uint8_t command_address = 0x1D;
        uint8_t data = 0x00;
        uint8_t checksum;

        uint8_t calculate_checksum() const {
            return ~(id + data_length + command_type + command_address + data);
        }
    };

    struct __attribute__((packed)) MotorReverseModeCommand {
        uint8_t header[2] = {0xFF, 0xFF};
        uint8_t id;
        uint8_t data_length = 0x04;
        uint8_t command_type = 0x03;
        uint8_t command_address = 0x1D;
        uint8_t data = 0x01;
        uint8_t checksum;

        uint8_t calculate_checksum() const {
            return ~(id + data_length + command_type + command_address + data);
        }
    };

    struct __attribute__((packed)) MotorRuntimeControlCommand {
        uint8_t header[2] = {0xFF, 0xFF};
        uint8_t id;
        uint8_t data_length = 0x05;
        uint8_t command_type = 0x03;
        uint8_t command_address = 0x41;
        uint8_t data[2] = {0x00, 0x14};
        uint8_t checksum;

        uint8_t calculate_checksum() const {
            return ~(id + data_length + command_type + command_address + data[0] + data[1]);
        }
    };

    struct __attribute__((packed)) MotorDisableControlCommand {
        uint8_t header[2] = {0xFF, 0xFF};
        uint8_t id;
        uint8_t data_length = 0x05;
        uint8_t command_type = 0x03;
        uint8_t command_address = 0x41;
        uint8_t data[2] = {0x00, 0x00};
        uint8_t checksum;

        uint8_t calculate_checksum() const {
            return ~(id + data_length + command_type + command_address + data[0] + data[1]);
        }
    };

    struct __attribute__((packed)) ReadAngleCommand {
        uint8_t header[2] = {0xFF, 0xFF};
        uint8_t id;
        uint8_t data_length = 0x04;
        uint8_t command_type = 0x02;
        uint8_t command_address = 0x38;
        uint8_t data = 0x02;
        uint8_t checksum;

        uint8_t calculate_checksum() const {
            return ~(id + data_length + command_type + command_address + data);
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

    struct __attribute__((packed)) ReadServoIDCommand {
        uint8_t header[2] = {0xFF, 0xFF};
        uint8_t id = 0xFE;
        uint8_t data_length = 0x04;
        uint8_t servo_status = 0x02;
        uint8_t current_angle[2] = {0x05, 0x01};
        uint8_t checksum;

        uint8_t calculate_checksum() const {
            uint8_t check = id + data_length + servo_status + current_angle[0] + current_angle[1];
            return ~check;
        }
    };

    template <typename T>
    std::unique_ptr<std::byte[]> copy_to_buffer(const T& cmd, size_t& out_len) {
        auto buffer = std::make_unique<std::byte[]>(sizeof(T));
        std::memcpy(buffer.get(), &cmd, sizeof(T));
        out_len = sizeof(T);
        return buffer;
    }

    rmcs_executor::Component::InputInterface<uint16_t> control_angle_;
    std::atomic<bool> is_calibrate_mode_ = false;
    std::atomic<uint16_t> current_angle_{0};
    uint8_t id_;
};

} // namespace rmcs_core::hardware::device