#pragma once

#include <cstddef>
#include <cstdint>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_utility/double_buffer.hpp>
#include <rmcs_utility/package_receive.hpp>

namespace rmcs_core::hardware::device {

class Hipnuc {
public:
    explicit Hipnuc() = default;

    bool store_status(const std::byte* uart_data, size_t uart_data_length) {
        struct {
            const std::byte* data;
            size_t data_length;

            size_t read(std::byte* buffer, size_t size) {
                if (size < data_length)
                    size = data_length;

                std::memcpy(buffer, data, size);
                data += size;
                data_length -= size;

                return size;
            };
        } stream{uart_data, uart_data_length};

        bool success = false;
        while (stream.data_length)
            success |= store_status<std::byte>(stream);
        return success;
    }

    template <rmcs_utility::is_byte ByteT>
    bool store_status(rmcs_utility::is_readable_stream<ByteT> auto& stream) {
        auto result = rmcs_utility::receive_package<ByteT>(
            stream, package_, cache_size_, uint16_t{0xA55A}, [](const auto& package) {
                uint16_t crc = 0;
                crc16_update(
                    &crc, reinterpret_cast<const std::byte*>(&package.header),
                    sizeof(package.header) + sizeof(package.length));
                crc16_update(
                    &crc, reinterpret_cast<const std::byte*>(&package.data_label),
                    sizeof(package.data_label) + sizeof(package.q));
                return crc == package.crc;
            });
        if (result == rmcs_utility::ReceiveResult::SUCCESS) {
            cache_size_ = 0;
            quaternion_buffer_.write(package_.q);
            return true;
        } else if (result == rmcs_utility::ReceiveResult::HEADER_INVALID)
            RCLCPP_ERROR(rclcpp::get_logger("ch040_imu"), "Header invalid");
        else if (result == rmcs_utility::ReceiveResult::VERIFY_INVALID)
            RCLCPP_ERROR(rclcpp::get_logger("ch040_imu"), "Data CRC invalid");
        return false;
    }

    void update_status() {
        QuaternionData quaternion;
        if (quaternion_buffer_.read(quaternion))
            quaternion_ =
                Eigen::Quaterniond{quaternion.w, quaternion.x, quaternion.y, quaternion.z};
    }

    const Eigen::Quaterniond& quaternion() const { return quaternion_; }

private:
    struct __attribute__((packed)) QuaternionData {
        float w;
        float x;
        float y;
        float z;
    };
    struct __attribute__((packed)) Package {
        uint16_t header;    // 0xA55A
        uint16_t length;    // 17 = sizeof(data_label) + sizeof(QuaternionData)
        uint16_t crc;       // Checksum of all fields except the CRC itself
        uint8_t data_label; // 0xD1 = Quaternion
        QuaternionData q;
    } package_;
    size_t cache_size_ = 0;

    rmcs_utility::DoubleBuffer<QuaternionData, true> quaternion_buffer_;
    Eigen::Quaterniond quaternion_ = Eigen::Quaterniond::Identity();

    static void crc16_update(uint16_t* crc_src, const std::byte* bytes, uint32_t len) {
        uint32_t crc = *crc_src;
        for (std::size_t byte_index = 0; byte_index < len; ++byte_index) {
            auto byte = static_cast<uint32_t>(bytes[byte_index]);
            crc ^= byte << 8;
            for (std::size_t crc_index = 0; crc_index < 8; ++crc_index) {
                uint32_t temp = crc << 1;
                if (crc & 0x8000)
                    temp ^= 0x1021;
                crc = temp;
            }
        }
        *crc_src = crc;
    }
};

} // namespace rmcs_core::hardware::device
