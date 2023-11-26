#pragma once

#include <algorithm>
#include <rclcpp/logger.hpp>
#include <stddef.h>
#include <string>

extern "C" {
#include <errno.h>               // Error integer and strerror() function
#include <fcntl.h>               // Contains file controls like O_RDWR
#include <termios.h>             // Contains POSIX terminal control definitions
#include <unistd.h>              // write(), read(), close()
}

class SerialHandle {
public:
    SerialHandle() = default;
    SerialHandle(const std::string& serialport) { open(serialport); }
    virtual ~SerialHandle() { close(); }

    void open(const std::string& serialport) {
        RCLCPP_INFO(
            rclcpp::get_logger("SerialHandle"), "SerialHandle opening %s", serialport.c_str());
        this->close();

        serial_port_ = ::open(serialport.c_str(), O_RDWR | O_NOCTTY);
        if (serial_port_ < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("SerialHandle"), "Error when open.");
            return;
        }
        struct ::termios tty;
        if (::tcgetattr(serial_port_, &tty) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("SerialHandle"), "Error when tcgetattr.");
            ::close(serial_port_);
            return;
        }
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);
        tty.c_cflag &= ~PARENB;  // Clear parity bit, disabling parity
        tty.c_cflag &= ~CSTOPB;  // Clear stop field, only one stop bit used in communication
        tty.c_cflag &= ~CSIZE;   // Clear all bits that set the data size
        tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
        tty.c_cflag |= CS8 | CREAD | CLOCAL; // 8 bits per byte & Turn on READ & ignore ctrl lines

        tty.c_cc[VTIME] = 1;                 // Wait for up to 100ms
        tty.c_cc[VMIN]  = 0;                 // returning as soon as any data is received.
        tcflush(serial_port_, TCIOFLUSH);
        if (::tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("SerialHandle"), "Error when tcsetattr.");
            ::close(serial_port_);
            return;
        }

        initialized_ = true;
        RCLCPP_INFO(
            rclcpp::get_logger("SerialHandle"), "SerialHandle opened with fd %d", serial_port_);
    }

    void close() {
        if (initialized_) {
            ::close(serial_port_);
            initialized_ = false;
            RCLCPP_INFO(
                rclcpp::get_logger("SerialHandle"), "SerialHandle closed with fd %d", serial_port_);
        }
    }

    bool is_open() { return initialized_; }

    void send(const uint8_t* buf, const size_t size) {
        // RCLCPP_INFO(rclcpp::get_logger("SerialHandle"), "Sending");
        if (!initialized_) {
            RCLCPP_ERROR(
                rclcpp::get_logger("SerialHandle"), "Using send() of uninitialized SerialHandle!");
            return;
        }
        ::write(serial_port_, buf, size);
    }

    void recv(uint8_t* buf, size_t& size) {
        // RCLCPP_INFO(rclcpp::get_logger("SerialHandle"), "Recving");
        if (!initialized_) {
            RCLCPP_ERROR(
                rclcpp::get_logger("SerialHandle"), "Using recv() of uninitialized SerialHandle!");
            return;
        }
        size = ::read(serial_port_, buf, size);
    }

private:
    bool initialized_ = false;
    int serial_port_  = -1;
};