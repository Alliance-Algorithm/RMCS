#pragma once

#include <stddef.h>
#include <string>

class SerialHandle {
public:
    SerialHandle()
        : initialized_(false) {}
    virtual ~SerialHandle() {}

    void open(const std::string& serialport) {
        // TODO(anyone)
        initialized_ = true;
    }

    void close() {
        // TODO(anyone)
        initialized_ = false;
    }

    bool is_open() {
        // TODO(anyone)
        return initialized_;
    }

    void send(const uint8_t* buf, const size_t size) {
        // TODO(anyone)
        printf("Send buf:");
        std::for_each(buf, buf + size, [](const uint8_t num) { printf(" %02X", num); });
        putchar('\n');
    }

    void recv(uint8_t* buf, size_t& size) {
        // TODO(anyone)
        puts("Try to recv.");
    }

    void recv_exact(uint8_t* buf, const size_t& size) {
        // TODO(anyone)
        printf("Try to recv exact %lu bytes.\n", size);
    }

private:
    // TODO(anyone)
    bool initialized_;
};