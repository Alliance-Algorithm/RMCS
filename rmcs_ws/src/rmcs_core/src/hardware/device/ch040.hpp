#pragma once

#include <librmcs/device/ch040.hpp>
#include <rmcs_executor/component.hpp>
#include <serial/serial.h>

#include <eigen3/Eigen/Dense>

namespace rmcs_core::hardware::device {

class Ch040 : public librmcs::device::Ch040 {
public:
    explicit Ch040(const std::string& port = "/dev/ttyUSB0", uint32_t baud = 115200)
        : serial_(port, baud) {
        serial_.open();
    }

    void update_status() {
        if (!serial_.available())
            return;

        serial_.read(buffer_, sizeof(Package));
        store_status(buffer_, sizeof(Package));
    }

    Eigen::Quaterniond q() const { return Eigen::Quaterniond{w(), x(), y(), z()}; }

private:
    serial::Serial serial_;
    uint8_t buffer_[1024];
};

} // namespace rmcs_core::hardware::device
