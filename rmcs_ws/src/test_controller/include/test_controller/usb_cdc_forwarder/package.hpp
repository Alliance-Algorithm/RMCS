#pragma once

#include <cstddef>
#include <cstdint>
#include <numeric>

#include "endian_promise.hpp"

namespace usb_cdc_forwarder {

constexpr size_t kPackageMaxSize = 64;

using package_head_t  = uint8_t;
using package_type_t  = uint8_t;
using package_index_t = uint8_t;
using package_size_t  = uint8_t;

using package_verify_code_t = uint8_t;
inline package_verify_code_t calculate_verify_code(const uint8_t* data, size_t size) {
    return std::accumulate(data, data + size, static_cast<package_verify_code_t>(0));
}

constexpr package_head_t kPackageHead = 0xAF;

struct __attribute__((packed)) PackageStaticPart final {
    package_head_t head;
    package_type_t type;
    package_index_t index;
    package_size_t data_size;
};

struct alignas(8) Package final {
    static constexpr size_t static_part_size() noexcept { return sizeof(PackageStaticPart); }

    PackageStaticPart& static_part() noexcept {
        return *reinterpret_cast<PackageStaticPart*>(buffer);
    }

    size_t dymatic_part_size() noexcept { return static_part().data_size; }

    template <typename T>
    T& dymatic_part() noexcept {
        return *reinterpret_cast<T*>(buffer + static_part_size());
    }

    static constexpr size_t verify_part_size() noexcept { return sizeof(package_verify_code_t); }

    package_verify_code_t& verify_part() noexcept {
        size_t length_without_verify = static_part_size() + dymatic_part_size();
        return *reinterpret_cast<package_verify_code_t*>(buffer + length_without_verify);
    }

    size_t size() noexcept { return static_part_size() + dymatic_part_size() + verify_part_size(); }

    uint8_t buffer[kPackageMaxSize];
};

using can_id_t = endian::le_uint32_t;

struct __attribute__((packed)) PackageC620FeedbackPart {
    can_id_t can_id;
    endian::be_int16_t angle;
    endian::be_int16_t velocity;
    endian::be_int16_t current;
    uint8_t temperature;
    uint8_t unused;
};

struct __attribute__((packed)) PackageC620ControlPart {
    can_id_t can_id;
    endian::be_int16_t current[4];
};

} // namespace usb_cdc_forwarder