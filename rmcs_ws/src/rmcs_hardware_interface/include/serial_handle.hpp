#pragma once

#include <stddef.h>

class SerialHandle {
public:
    SerialHandle() {}
    virtual ~SerialHandle() {}

    void send(const char* buf, size_t size) {
        // TODO(anyone)
    }

    void recv(char* buf, size_t& size) {
        // TODO(anyone)
    }

private:
    // TODO(anyone)
};