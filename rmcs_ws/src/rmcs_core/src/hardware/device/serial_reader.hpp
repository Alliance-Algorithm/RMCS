#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

#include <serial/serial.h>

namespace rmcs_core::hardware::device {

class SerialReader {
public:
    SerialReader(const std::string& path, uint32_t baudrate)
        : serial_(path, baudrate, serial::Timeout::simpleTimeout(0)) {
        const std::string stty_command = "stty -F " + path + " raw";
        if (std::system(stty_command.c_str()) != 0)
            throw std::runtime_error{"Unable to call '" + stty_command + "'"};
    }

    [[nodiscard]] bool active() const { return serial_.isOpen(); }

    [[nodiscard]] size_t available() { return serial_.available(); }

    auto read_available() -> std::vector<std::byte> {
        const auto readable = serial_.available();
        std::vector<std::byte> buffer(readable);
        if (readable == 0)
            return buffer;

        const auto size = serial_.read(reinterpret_cast<uint8_t*>(buffer.data()), readable);
        buffer.resize(size);
        return buffer;
    }

private:
    serial::Serial serial_;
};

} // namespace rmcs_core::hardware::device
