#pragma once

#include <eigen3/Eigen/Dense>
#include <librmcs/device/ch040.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>
#include <serial/serial.h>

namespace rmcs_core::hardware::device {

class Ch040 : public librmcs::device::Ch040 {
public:
    explicit Ch040(const std::string& port = "/dev/ttyUSB0", uint32_t baud = 115200) {

        RCLCPP_INFO(logger_, "CH040 IMU Initializing");

        try {
            serial_ = std::make_unique<serial::Serial>(port, baud);
            serial_->open();
            available_ = serial_->isOpen();
        } catch (serial::IOException& e) {
            available_ = false;
            RCLCPP_WARN(
                rclcpp::get_logger("CH040"), "Error happened while opening port %s: %s",
                port.c_str(), e.what());
        } catch (const serial::SerialException& e) {
            available_ = false;
            RCLCPP_WARN(
                rclcpp::get_logger("CH040"), "Error happened while opening port %s: %s",
                port.c_str(), e.what());
        } catch (const std::invalid_argument& e) {
        } catch (...) {
            RCLCPP_WARN(logger_, "Unknown error happened");
        }
    }

    void update_status() {
        if (!available_)
            return;

        if (!serial_->available())
            return;

        auto length = serial_->available();
        if (!length)
            return;

        try {
            serial_->read(buffer_, length);
        } catch (const serial::PortNotOpenedException& e) {
            RCLCPP_WARN(rclcpp::get_logger("CH040"), "Error happened while reading: %s", e.what());
            available_ = false;
        } catch (const serial::SerialException& e) {
            RCLCPP_WARN(rclcpp::get_logger("CH040"), "Error happened while reading: %s", e.what());
            available_ = false;
        }

        constexpr uint8_t standard_header[] = {0x5a, 0xa5, 17};

        auto& index = check_index_;
        while (index + sizeof(Package) < buffer_.size()) {
            if (buffer_[index] == standard_header[0] && buffer_[index + 1] == standard_header[1]
                && buffer_[index + 2] == standard_header[2]) {
                store_status(buffer_.data() + index, sizeof(Package));
                index += sizeof(Package);
            }
            index++;
        }

        if (check_index_ > 1024) {
            buffer_.clear();
            check_index_ = 0;
        }
    }

    Eigen::Quaterniond q() const { return Eigen::Quaterniond{w(), x(), y(), z()}; }

    bool available() const { return available_; }

private:
    std::size_t check_index_{0};
    std::vector<uint8_t> buffer_;

    std::unique_ptr<serial::Serial> serial_;

    bool available_{false};

    rclcpp::Logger logger_{rclcpp::get_logger("CH040")};
};

} // namespace rmcs_core::hardware::device
