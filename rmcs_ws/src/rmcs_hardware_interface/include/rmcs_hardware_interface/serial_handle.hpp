#pragma once

#include <algorithm>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stddef.h>
#include <string>

#include <serial/serial.h>

class SerialHandle {
public:
    SerialHandle() = default;
    SerialHandle(const std::string& serialport) { open(serialport); }
    virtual ~SerialHandle() { close(); }

    void open(const std::string& serialport) {
        try {
            serial_ = std::make_unique<serial::Serial>(
                serialport, 9600, serial::Timeout::simpleTimeout(0));
        } catch (...) {
            RCLCPP_ERROR(rclcpp::get_logger("SerialHandle"), "Error when open.");
            throw;
        }
        initialized_ = true;
        RCLCPP_INFO(rclcpp::get_logger("SerialHandle"), "Serialhandle opened successfully.");
    }

    void close() {
        if (!initialized_)
            return;
        serial_.reset(nullptr);
        initialized_ = false;
    }

    bool is_open() { return initialized_; }

    void send(const uint8_t* buf, const size_t size) {
        // RCLCPP_INFO(rclcpp::get_logger("SerialHandle"), "Sending");
        if (!initialized_) {
            RCLCPP_ERROR(
                rclcpp::get_logger("SerialHandle"), "Using send() of uninitialized SerialHandle!");
            return;
        }
        serial_->write(buf, size);
    }

    void recv(uint8_t* buf, size_t& size) {
        // RCLCPP_INFO(rclcpp::get_logger("SerialHandle"), "Recving");
        if (!initialized_) {
            RCLCPP_ERROR(
                rclcpp::get_logger("SerialHandle"), "Using recv() of uninitialized SerialHandle!");
            return;
        }
        size = serial_->read(buf, size);
    }

private:
    bool initialized_ = false;
    std::unique_ptr<serial::Serial> serial_;
};