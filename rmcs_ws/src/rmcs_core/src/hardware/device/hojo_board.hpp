#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>
#include <string>

namespace rmcs_core::hardware::device {

class HojoBoard {
public:
    HojoBoard(rmcs_executor::Component& command_component,
                 const std::string& name,
                 uint8_t id_first, uint8_t id_second)
        : id_first_(id_first), id_second_(id_second) {
        command_component.register_input(name + "_first/control_angle", control_angle_first_);
        command_component.register_input(name + "_second/control_angle", control_angle_second_);
        command_component.register_input(name + "_first/runtime", runtime_first_);
        command_component.register_input(name + "_second/runtime", runtime_second_);

    }

    std::unique_ptr<std::byte[]> generate_sync_run_command(size_t& output_length) {
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
        command.cowork_id_one = id_first_;
        command.control_angle_one[0] = static_cast<uint8_t>(*control_angle_first_ >> 8);
        command.control_angle_one[1] = static_cast<uint8_t>(*control_angle_first_ & 0x00FF);
        command.runtime_one[0] = static_cast<uint8_t>(*runtime_first_ >> 8);
        command.runtime_one[1] = static_cast<uint8_t>(*runtime_first_ & 0x00FF);
        command.cowork_id_two = id_second_;
        command.control_angle_two[0] = static_cast<uint8_t>(*control_angle_second_ >> 8);
        command.control_angle_two[1] = static_cast<uint8_t>(*control_angle_second_ & 0x00FF);
        command.runtime_two[0] = static_cast<uint8_t>(*runtime_second_ >> 8);
        command.runtime_two[1] = static_cast<uint8_t>(*runtime_second_ & 0x00FF);
        command.checksum = command.calculate_checksum();

        std::memcpy(buffer.get(), &command, size);
        output_length = size;
        return buffer;
    }

private:
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
            uint8_t check = id + data_length + command_type + command_address + command_address_length 
                          + cowork_id_one + control_angle_one[0] + control_angle_one[1] + runtime_one[0] + runtime_one[1] 
                          + cowork_id_two + control_angle_two[0] + control_angle_two[1] + runtime_two[0] + runtime_two[1];
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

    uint8_t id_first_;
    uint8_t id_second_;

    rmcs_executor::Component::InputInterface<uint16_t> control_angle_first_;
    rmcs_executor::Component::InputInterface<uint16_t> control_angle_second_;
    rmcs_executor::Component::InputInterface<uint16_t> runtime_first_;
    rmcs_executor::Component::InputInterface<uint16_t> runtime_second_;
};

} // namespace rmcs_core::hardware::device