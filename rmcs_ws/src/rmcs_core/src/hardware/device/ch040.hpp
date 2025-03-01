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
        RCLCPP_INFO(logger_, "Ch040 IMU Initializing, try to open: %s", port.c_str());

        serial_.setPort(port);
        serial_.setBaudrate(baud);

        try {
            serial_.open();
            available_ = true;
        } catch (serial::IOException& e) {
            available_ = false;
            RCLCPP_WARN(
                logger_, "IOException happened while opening %s: %s", port.c_str(), e.what());
        } catch (const serial::SerialException& e) {
            available_ = false;
            RCLCPP_WARN(
                logger_, "SerialException happened while opening %s: %s", port.c_str(), e.what());
        } catch (const std::invalid_argument& e) {
        } catch (...) {
            available_ = false;
            RCLCPP_WARN(logger_, "Unknown error happened");
        }
    }

    auto quaternion() const {
        return initial_orientation_.inverse() * Eigen::Quaterniond{w(), x(), y(), z()};
    }
    auto available() const { return available_; }

    void update_status() {
        if (!serial_.available())
            return;

        try {
            auto length = serial_.read(buffer_ + index_head_, sizeof(buffer_) - index_tail_);
            index_tail_ += length;

            if (index_tail_ >= 1024 - sizeof(Package)) {
                index_head_ = 0;
                index_tail_ = 0;
            }

            while (index_head_ < index_tail_)
                if (store_status(buffer_ + index_head_++, sizeof(Package))) {
                    index_head_ += sizeof(Package);
                    if (!record_initial_orientation_) {
                        record_initial_orientation_ = true;
                        initial_orientation_        = {w(), x(), y(), z()};
                    }
                }

        } catch (const serial::PortNotOpenedException& e) {
            RCLCPP_WARN(logger_, "PortNotOpenedException happened while reading: %s", e.what());
            available_ = false;
        } catch (const serial::SerialException& e) {
            RCLCPP_WARN(logger_, "SerialException happened while reading: %s", e.what());
            available_ = false;
        }
    }

private:
    std::size_t index_head_{0};
    std::size_t index_tail_{0};
    uint8_t buffer_[1024];

    Eigen::Quaterniond initial_orientation_{Eigen::Quaterniond::Identity()};
    bool record_initial_orientation_{false};

    serial::Serial serial_{};
    rclcpp::Logger logger_{rclcpp::get_logger("ch040")};
    bool available_{false};
};

} // namespace rmcs_core::hardware::device
