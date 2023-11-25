#pragma once

#include <algorithm>
#include <stddef.h>
#include <string>

extern "C" {
#include <errno.h>   // Error integer and strerror() function
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
}

class SerialHandle {
public:
    SerialHandle() = default;
    virtual ~SerialHandle() = default;

    void open(const std::string& serialport) {
        serial_port_ = ::open(serialport.c_str(), O_RDWR);
        if (serial_port_ < 0) {
            printf("Error when open.");
            return;
        }
        struct ::termios tty;
        if (::tcgetattr(serial_port_, &tty) != 0) {
            printf("Error when tcgetattr.");
        }
        tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity
        tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication
        tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
        tty.c_cflag |= CS8;            // 8 bits per byte
        tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;                   // Disable echo
        tty.c_lflag &= ~ECHOE;                  // Disable erasure
        tty.c_lflag &= ~ECHONL;                 // Disable new-line echo
        tty.c_lflag &= ~ISIG;                   // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        // Disable any special handling of received bytes

        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

        tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds)
        tty.c_cc[VMIN]  = 0;  // returning as soon as any data is received.
        if (::tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
            printf("Error when tcsetattr.");
        }

        initialized_ = true;
    }

    void close() {
        ::close(serial_port_);
        initialized_ = false;
    }

    bool is_open() {
        return initialized_ && serial_port_ >= 0;
    }

    void send(const uint8_t* buf, const size_t size) {
        printf("Send buf:");
        std::for_each(buf, buf + size, [](const uint8_t num) { printf(" %02X", num); });
        putchar('\n');
        ::write(serial_port_, buf, size);
    }

    void recv(uint8_t* buf, size_t& size) {
        puts("Try to recv.");
        size = ::read(serial_port_, buf, size);
    }

    void recv_exact(uint8_t* buf, const size_t& size) {
        printf("Try to recv exact %lu bytes.\n", size);
        size_t n = 0;
        do {
            n += ::read(serial_port_, buf, size - n);
        } while (n == size);
    }

private:
    bool initialized_ = false;
    int serial_port_ = -1;
};