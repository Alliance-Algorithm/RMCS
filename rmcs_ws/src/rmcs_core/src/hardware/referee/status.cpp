#include <iostream>

#include "serial_util/crc/dji_crc.hpp"
#include "serial_util/package_receive.hpp"

constexpr size_t frame_data_max_length = 1024;

struct __attribute__((packed)) FrameHeader {
    uint8_t start;
    uint16_t data_length;
    uint8_t sequence;
    uint8_t crc8;
};

struct __attribute__((packed)) FrameBody {
    uint16_t command_id;
    uint8_t data[frame_data_max_length];
};

struct __attribute__((packed)) Frame {
    FrameHeader header;
    FrameBody body;
};

struct __attribute__((packed)) RobotStatus {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output  : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
};

class Test {
public:
    void update() {
        if (cache_size_ >= sizeof(frame_.header)) {
            auto frame_size = sizeof(frame_.header) + sizeof(frame_.body.command_id)
                            + frame_.header.data_length + sizeof(uint16_t);
            cache_size_ += serial_.read(
                reinterpret_cast<uint8_t*>(&frame_) + cache_size_, frame_size - cache_size_);

            if (cache_size_ == frame_size) {
                cache_size_ = 0;
                if (serial_util::dji_crc::verify_crc16(&frame_, frame_size)) {
                    // std::cout << "success" << i++ << '\n';
                    if (frame_.body.command_id == 0x0201) {
                        auto& status = reinterpret_cast<RobotStatus&>(frame_.body.data);
                        std::cout << status.shooter_barrel_cooling_value << ' '
                                  << status.shooter_barrel_heat_limit << '\n';
                    }
                } else {
                    std::cout << "body crc16 invaild\n";
                }
            }
        } else {
            auto result = serial_util::receive_package(
                serial_, frame_.header, cache_size_, static_cast<uint8_t>(0xa5),
                [](const FrameHeader& header) {
                    return serial_util::dji_crc::verify_crc8(header);
                });
            if (result == serial_util::ReceiveResult::HEADER_INVAILD) {
                std::cout << "header start invaild\n";
            } else if (result == serial_util::ReceiveResult::VERIFY_INVAILD) {
                std::cout << "header crc8 invaild\n";
            }
        }
    }

private:
    serial::Serial serial_{"/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(0)};
    Frame frame_;
    size_t cache_size_ = 0;
};