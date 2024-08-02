#pragma once

#include <concepts>
#include <cstddef>
#include <cstring>

#include <rmcs_msgs/serial_interface.hpp>
#include <serial/serial.h>

namespace serial_util {

enum class ReceiveResult : uint8_t {
    SUCCESS        = 0,
    TIMEOUT        = 1,
    HEADER_INVALID = 2,
    VERIFY_INVALID = 3
};

template <typename F, typename PackageT>
concept verify_function = requires(F f) {
    { f(std::declval<const PackageT&>()) } -> std::convertible_to<bool>;
};

template <
    size_t header_size, typename ByteT, typename SerialT, typename PackageT,
    verify_function<PackageT> HeaderVerifyT, verify_function<PackageT> VerifyT>
inline ReceiveResult basic_receive_package(
    SerialT& serial, PackageT& buffer, size_t& cache_size, HeaderVerifyT header_verify,
    VerifyT verify) {

    if (cache_size == sizeof(PackageT))
        return ReceiveResult::SUCCESS;

    auto* buffer_pointer = reinterpret_cast<ByteT*>(&buffer);
    cache_size += serial.read(buffer_pointer + cache_size, sizeof(PackageT) - cache_size);

    if (cache_size == 0 || cache_size < header_size)
        return ReceiveResult::TIMEOUT;

    ReceiveResult result;
    if (header_verify(buffer)) {
        if (cache_size != sizeof(PackageT))
            return ReceiveResult::TIMEOUT;
        if (verify(buffer)) {
            return ReceiveResult::SUCCESS;
        }
        result = ReceiveResult::VERIFY_INVALID;
    } else {
        result = ReceiveResult::HEADER_INVALID;
    }

    while (true) {
        --cache_size;
        ++buffer_pointer;
        if (cache_size == 0 || cache_size < header_size
            || header_verify(reinterpret_cast<PackageT&>(*buffer_pointer))) {
            memmove(&buffer, buffer_pointer, cache_size);
            break;
        }
    }
    return result;
}

template <typename PackageT, std::integral HeaderT, verify_function<PackageT> VerifyT>
inline ReceiveResult receive_package(
    serial::Serial& serial, PackageT& buffer, size_t& cache_size, HeaderT header, VerifyT verify) {
    return basic_receive_package<sizeof(header), uint8_t>(
        serial, buffer, cache_size,
        [header](const PackageT& package) {
            return reinterpret_cast<const HeaderT&>(package) == header;
        },
        verify);
}

template <typename PackageT, std::integral HeaderT, verify_function<PackageT> VerifyT>
inline ReceiveResult receive_package(
    rmcs_msgs::SerialInterface& serial, PackageT& buffer, size_t& cache_size, HeaderT header,
    VerifyT verify) {
    return basic_receive_package<sizeof(header), std::byte>(
        serial, buffer, cache_size,
        [header](const PackageT& package) {
            return reinterpret_cast<const HeaderT&>(package) == header;
        },
        verify);
}

} // namespace serial_util