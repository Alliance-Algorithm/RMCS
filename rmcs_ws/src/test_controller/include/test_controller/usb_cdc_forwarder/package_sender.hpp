#pragma once

#include <cassert>
#include <serial/serial.h>

#include "test_controller/usb_cdc_forwarder/package.hpp"

namespace usb_cdc_forwarder {

class PackageSender final {
public:
    explicit PackageSender(serial::Serial& serial)
        : serial_(serial) {}
    PackageSender(const PackageSender&)            = delete;
    PackageSender& operator=(const PackageSender&) = delete;

    void Send() {
        auto size = package.size();
        assert(size <= kPackageMaxSize);

        package.static_part().head = kPackageHead;
        package.verify_part() =
            calculate_verify_code(package.buffer, size - package.verify_part_size());

        serial_.write(package.buffer, size);
    }

    Package package;

private:
    serial::Serial& serial_;
};

} // namespace usb_cdc_forwarder