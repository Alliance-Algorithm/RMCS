#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>

#include <rclcpp/logger.hpp>
#include <serial.h>

#include "serial_util.hpp"

namespace serial {

constexpr size_t package_size_max  = 64;
constexpr size_t verify_bytes_size = 1;

enum class PackageType : uint8_t {
    PKG_LOG  = 0x00,
    PKG_CAN  = 0x10,
    PKG_UART = 0x20,
    PKG_SPI  = 0x30,
    PKG_IIC  = 0x40,
};

class SerialPackage {
public:
    struct alignas(1) PackageHead {
        uint8_t type;
        uint8_t destnation;
        uint8_t index;
        uint8_t length;
    };
    static constexpr size_t data_size_max =
        package_size_max - verify_bytes_size - sizeof(PackageHead);

public:
    SerialPackage(uint8_t type, uint8_t destnation, size_t tx_buf_size, size_t rx_buf_size)
        : type_(type)
        , destnation_(destnation)
        , tx_buf_(new uint8_t[tx_buf_size])
        , rx_buf_(new uint8_t[rx_buf_size]) {}
    virtual ~SerialPackage() {
        delete[] tx_buf_;
        delete[] rx_buf_;
    }

    template <class VerifyCodeCalculator>
    // requires VerifyCodeCalculator::Append
    const uint8_t* split(const uint8_t* data, const size_t data_size, size_t& split_size) {
        PackageHead pack_head = *reinterpret_cast<PackageHead*>(tx_buf_);
        pack_head.type        = type_;
        pack_head.destnation  = destnation_;
        pack_head.index       = split_size / data_size;
        pack_head.length      = std::min(data_size - split_size, data_size_max);
        memcpy(tx_buf_ + sizeof(PackageHead), data, pack_head.length);
        VerifyCodeCalculator::Append(
            tx_buf_, sizeof(PackageHead) + pack_head.length,
            &tx_buf_[sizeof(PackageHead) + pack_head.length]);
        split_size += pack_head.length;
        return tx_buf_;
    }

    bool merge() {
        // TODO(anyone)
    }

    const uint8_t* read() { return tx_buf_; }

    template <class VerifyCodeCalculator>
    bool write(const uint8_t* data, size_t size) {
        if (VerifyCodeCalculator::Verify(data, size - 1, data[size - 1])) {}
        memcpy(tx_buf_, data, size);
    }

protected:
    uint8_t type_, destnation_;
    uint8_t *tx_buf_, *rx_buf;
};

class SerialDeliver {
public:
    SerialDeliver()                                = default;
    SerialDeliver(const SerialDeliver&)            = delete;
    SerialDeliver& operator=(const SerialDeliver&) = delete;
    virtual ~SerialDeliver()                       = default;

    void open(const std::string& serialport) {
        // TODO(anyone)
    }

    void send(uint8_t type, uint8_t dest, const uint8_t* data, const size_t size) {
        // TODO(anyone)
    }

    size_t recv(uint8_t type, uint8_t dest, uint8_t* data) {
        // TODO(anyone)
    }

private:
    std::unique_ptr<serial::Serial> serial_;
    std::unique_ptr<serial::SerialReceiver<
        SerialPackage::PackageHead, serial::Head<uint8_t, 0xAF>, serial::verify::CheckSum>>
        serial_receiver_;
};

} // namespace serial