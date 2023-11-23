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

    void send(const char* buf, size_t size) {
        // TODO(anyone)
    }

    void recv(char* buf, size_t& size) {
        // TODO(anyone)
    }

private:
    // TODO(anyone)
    bool initialized_;
};