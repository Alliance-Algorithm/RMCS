#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <concepts>
#include <new>

namespace rmcs_utility {

enum class ReceiveResult : uint8_t {
    SUCCESS        = 0,
    TIMEOUT        = 1,
    HEADER_INVALID = 2,
    VERIFY_INVALID = 3
};

template <typename T>
concept is_byte =
    std::is_same_v<T, char> || std::is_same_v<T, unsigned char> || std::is_same_v<T, std::byte>;

template <typename SerialT, typename ByteT>
concept is_readable_stream = requires(SerialT& serial, ByteT* pointer, size_t size) {
    { serial.read(pointer, size) } -> std::convertible_to<size_t>;
};

template <typename F, typename PackageT>
concept is_verify_function = requires(const F& f, const PackageT& package) {
    { f(package) } -> std::convertible_to<bool>;
};

template <size_t header_size, is_byte ByteT, typename PackageT>
requires std::is_trivially_copyable_v<PackageT> inline auto receive_package(
    is_readable_stream<ByteT> auto& stream, PackageT& buffer, size_t& cache_size,
    const is_verify_function<PackageT> auto& header_verify,
    const is_verify_function<PackageT> auto& verify) -> ReceiveResult {
    if (cache_size == sizeof(PackageT))
        return ReceiveResult::SUCCESS;

    auto* buffer_pointer = reinterpret_cast<ByteT*>(&buffer);
    cache_size += stream.read(buffer_pointer + cache_size, sizeof(PackageT) - cache_size);

    if (cache_size == 0 || cache_size < header_size)
        return ReceiveResult::TIMEOUT;

    ReceiveResult result;
    if (header_size == 0 || header_verify(buffer)) {
        if (cache_size != sizeof(PackageT))
            return ReceiveResult::TIMEOUT;
        if (verify(buffer))
            return ReceiveResult::SUCCESS;
        else
            result = ReceiveResult::VERIFY_INVALID;
    } else {
        result = ReceiveResult::HEADER_INVALID;
    }

    while (true) {
        memmove(buffer_pointer, buffer_pointer + 1, --cache_size);
        if (cache_size == 0 || cache_size < header_size || header_size == 0
            || header_verify(*std::launder(reinterpret_cast<PackageT*>(buffer_pointer)))) {
            break;
        }
    }
    return result;
}

template <is_byte ByteT, typename PackageT, std::integral HeaderT>
requires std::is_trivially_copyable_v<PackageT> inline auto receive_package(
    is_readable_stream<ByteT> auto& stream, PackageT& buffer, size_t& cache_size, HeaderT header,
    const is_verify_function<PackageT> auto& verify) -> ReceiveResult {
    return receive_package<sizeof(HeaderT), ByteT>(
        stream, buffer, cache_size,
        [header](const PackageT& package) {
            HeaderT actual_header;
            std::memcpy(&actual_header, &package, sizeof(HeaderT));
            return actual_header == header;
        },
        verify);
}

template <is_byte ByteT, typename PackageT>
requires std::is_trivially_copyable_v<PackageT> inline auto receive_package(
    is_readable_stream<ByteT> auto& stream, PackageT& buffer, size_t& cache_size,
    const is_verify_function<PackageT> auto& verify) -> ReceiveResult {
    return receive_package<0, std::byte>(
        stream, buffer, cache_size, [](const PackageT&) { return true; }, verify);
}

} // namespace rmcs_utility