#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>

#include <serial/serial.h>

#include "rmcs_hardware_interface/serial_util.hpp"

namespace serial {

constexpr size_t package_size_max = 64;
constexpr size_t verify_bytes_size = 1;

class SerialPackage {
public:
    struct alignas(1) PackageHead {
        uint8_t type;
        uint8_t destnation;
        uint8_t index;
        uint8_t length;
    };
    static constexpr size_t data_size_max = package_size_max -verify_bytes_size- sizeof(PackageHead);

public:
    SerialPackage(uint8_t type, uint8_t destnation, size_t buf_size)
        : type_(type)
        , destnation_(destnation)
        , buf_(new uint8_t[buf_size]) {}
    virtual ~SerialPackage() { delete[] buf_; }

    const uint8_t* format(const uint8_t* data, const size_t data_size, size_t& formatted_size) {
        PackageHead pack_head = *reinterpret_cast<PackageHead*>(buf_);
        pack_head.type = type_;
        pack_head.destnation = destnation_;
        pack_head.index = formatted_size / data_size;
        pack_head.length = std::min(data_size - formatted_size, data_size_max);
        memcpy(buf_+sizeof(PackageHead), data, pack_head.length);
        formatted_size += pack_head.length;

        return buf_;
    }

protected:
    uint8_t type_, destnation_;
    uint8_t* buf_;
};

class SerialDeliver {
public:
    SerialDeliver()                                = default;
    SerialDeliver(const SerialDeliver&)            = delete;
    SerialDeliver& operator=(const SerialDeliver&) = delete;
    virtual ~SerialDeliver()                       = default;

    void Update() {}

private:
    std::unique_ptr<serial::Serial> serial_;
    std::unique_ptr<
        serial::SerialReceiver<SerialPackage::PackageHead, serial::Head<uint8_t, 0xAF>, serial::verify::CheckSum>>
        serial_receiver_;
};

} // namespace serial