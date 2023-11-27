#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>

#include <rclcpp/logger.hpp>
#include <serial/serial.h>

#include "verify.hpp"

namespace serial {

constexpr size_t package_buf_size = 1024;

class SerialPackage {
public:
    static constexpr size_t package_size_max   = 64;
    static constexpr uint8_t package_head_byte = 0xAF;

    enum class PackageType : uint8_t {
        USB_PKG_LOG         = 0x00,
        USB_PKG_LOG_INFO    = 0x00,
        USB_PKG_LOG_WARNING = 0x01,
        USB_PKG_LOG_ERROR   = 0x02,

        USB_PKG_CAN  = 0x10,
        USB_PKG_UART = 0x20,
        USB_PKG_SPI  = 0x30,
        USB_PKG_IIC  = 0x40,
    };
    static uint8_t TypeEncode(PackageType type, uint8_t destnation) {
        return static_cast<uint8_t>(type) | destnation;
    }
    static PackageType TypeDecode(uint8_t type_code) {
        return static_cast<PackageType>(type_code & 0xF0);
    }
    static uint8_t DestnationDecode(uint8_t type_code) { return type_code & 0x0F; }

    struct alignas(1) PackageHead {
        uint8_t head;
        uint8_t type_code;
        uint8_t index;
        uint8_t size;
        // 'Data[0]' ... 'Data[Size-1]'
        uint8_t data_crc;
    };
    static constexpr size_t data_size_max = package_size_max - sizeof(PackageHead);

public:
    SerialPackage(size_t tx_buf_size = package_buf_size, size_t rx_buf_size = package_buf_size)
        : tx_buf_(new uint8_t[tx_buf_size])
        , rx_buf_(new uint8_t[rx_buf_size]) {
        clean();
    }
    virtual ~SerialPackage() {
        delete[] tx_buf_;
        delete[] rx_buf_;
    }

    void clean() {
        auto& rx_package = *reinterpret_cast<SerialPackage::PackageHead*>(rx_buf_);
        rx_package.size  = 0;
        rx_package.index = 0xFF;
    }

    template <class VerifyCodeCalculator>
    size_t split(
        const uint8_t type_code, const uint8_t* data, const size_t data_size, size_t& split_size) {
        auto package      = *reinterpret_cast<PackageHead*>(tx_buf_);
        package.type_code = type_code;
        package.index     = split_size / data_size;
        package.size      = std::min(data_size - split_size, data_size_max);
        memcpy(&package.data_crc, data + split_size, package.size);
        VerifyCodeCalculator::Append(
            tx_buf_, sizeof(PackageHead) + package.size - sizeof(PackageHead::data_crc),
            tx_buf_[sizeof(PackageHead) + package.size - sizeof(PackageHead::data_crc)]);
        split_size += package.size;
        return sizeof(PackageHead) + package.size;
    }

    template <class VerifyCodeCalculator>
    bool merge(const uint8_t* data, const size_t data_size) {
        if (VerifyCodeCalculator::Verify(data, data_size - 1, data[data_size - 1]))
            return false;
        auto package     = *reinterpret_cast<const PackageHead*>(rx_buf_);
        auto new_package = *reinterpret_cast<const PackageHead*>(data);
        if (new_package.index != package.index + 1) {
            memcpy(rx_buf_, data, data_size);
            return true;
        } // else try to merge
        if (sizeof(PackageHead) + package.size + new_package.size > package_buf_size) {
            RCLCPP_WARN(
                rclcpp::get_logger("SerialDeliver"),
                "Buffer overflow, unable to merge package, type_code<%02X>", package.type_code);
            clean();
            return true;
        }
        memcpy(&package.data_crc + package.size, &new_package.data_crc, new_package.size);
        package.size += new_package.size;
        package.index = new_package.index;
        return false;
    }

    PackageHead& read() { return *reinterpret_cast<PackageHead*>(rx_buf_); }

    const uint8_t* send_buf() { return tx_buf_; }

protected:
    uint8_t *tx_buf_, *rx_buf_;
};

class SerialDeliver {
public:
    SerialDeliver()                                = default;
    SerialDeliver(const SerialDeliver&)            = delete;
    SerialDeliver(const SerialDeliver&&)           = delete;
    SerialDeliver& operator=(const SerialDeliver&) = delete;
    virtual ~SerialDeliver()                       = default;

    using VerifyCodeCalculator = verify::CheckSumCalculator;

    bool open(const std::string& serialport) {
        try {
            serial_ = std::make_unique<serial::Serial>(
                serialport, 9600U, serial::Timeout::simpleTimeout(0));
            serial_->open();
            if (!serial_->isOpen())
                throw;
        } catch (...) {
            RCLCPP_ERROR(
                rclcpp::get_logger("SerialDeliver"), "Failed to open serialport \"%s\"",
                serialport.c_str());
            serial_ = nullptr;
            return false;
        }
        return true;
    }

    void close() {
        if (serial_ == nullptr)
            return;
        serial_->close();
        serial_ = nullptr;
    }

    void update() {
        if (serial_ == nullptr || !serial_->available())
            return;

        while (serial_->available()) {
            serial_->read(
                buf_,
                sizeof(SerialPackage::PackageHead) - sizeof(SerialPackage::PackageHead::data_crc));
            auto package = *reinterpret_cast<SerialPackage::PackageHead*>(buf_);
            if (package.head != SerialPackage::package_head_byte) {
                do {
                    serial_->read(buf_, sizeof(SerialPackage::PackageHead::head));
                } while (package.head != SerialPackage::package_head_byte);
                serial_->read(
                    buf_ + sizeof(SerialPackage::PackageHead::head),
                    sizeof(SerialPackage::PackageHead) - sizeof(SerialPackage::PackageHead::head)
                        - sizeof(SerialPackage::PackageHead::data_crc));
            }

            auto bytes_read = serial_->read(
                &package.data_crc, package.size + sizeof(SerialPackage::PackageHead::data_crc));
            if (bytes_read != package.size + sizeof(SerialPackage::PackageHead::data_crc)) {
                RCLCPP_WARN(
                    rclcpp::get_logger("SerialDeliver"), "Lost package, type_code<%02X>",
                    package.type_code);
                continue;
            }

            packages_[package.type_code].merge<VerifyCodeCalculator>(
                buf_, package.size + sizeof(SerialPackage::PackageHead));
        }
    }

    void send(uint8_t type_code, const uint8_t* data, const size_t size) {
        if (serial_ == nullptr)
            return;
        auto& package     = packages_[type_code];
        size_t split_size = 0;
        while (split_size < size) {
            size_t send_buf_size =
                package.split<VerifyCodeCalculator>(type_code, data, size, split_size);
            serial_->write(package.send_buf(), send_buf_size);
        }
    }

    size_t get(uint8_t type_code, uint8_t* data) {
        if (!packages_.contains(type_code))
            return 0;
        auto& package = packages_.at(type_code).read();
        memcpy(data, &package.data_crc, package.size);
        return package.size;
    }

    size_t recv(uint8_t type_code, uint8_t* data) {
        if (serial_ == nullptr)
            return 0;
        update();
        return get(type_code, data);
    }

private:
    std::unique_ptr<serial::Serial> serial_;
    std::map<uint8_t, SerialPackage> packages_;

    uint8_t buf_[package_buf_size];
};

} // namespace serial