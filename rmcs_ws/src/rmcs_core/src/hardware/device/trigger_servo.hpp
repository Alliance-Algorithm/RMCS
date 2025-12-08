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
    TriggerServo(rmcs_executor::Component& command_component, const std::string& name) {
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
        command.id = 0x01;
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
    uint8_t read_angle_command_[8] = {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE};

    uint8_t switch_servo_mode_[8] = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x1C, 0x01, 0xDA};
    uint8_t switch_motor_mode_[8] = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x1C, 0x00, 0xDB};

    uint8_t switch_motor_forward_mode_[8] = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x1D, 0x00, 0xDA};
    uint8_t switch_motor_reverse_mode_[8] = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x1D, 0x01, 0xD9};

    uint8_t motor_mode_runtime_control_[9] = {0xFF, 0xFF, 0x01, 0x05, 0x03, 0x41, 0x00, 0x14, 0xA1};
    uint8_t motor_mode_disable_control_[9] = {0xFF, 0xFF, 0x01, 0x05, 0x03, 0x41, 0x00, 0x00, 0xB5};

    rmcs_executor::Component::InputInterface<uint16_t> control_angle_; // 0-4095

    std::atomic<bool> is_calibrate_mode_ = false;
};
} // namespace rmcs_core::hardware::device