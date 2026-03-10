#pragma once

#include <array>
#include <atomic>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <stdexcept>
#include <type_traits>

namespace rmcs_core::hardware::device {

template <typename T, size_t align = alignof(T)>
requires(std::is_trivial_v<T>) struct ByteConvertible {
    alignas(align) T data;

    ByteConvertible() = default;

    constexpr explicit ByteConvertible(const T& data)
        : data(data) {}

    constexpr explicit ByteConvertible(std::span<const std::byte, sizeof(data)> bytes) noexcept {
        std::memcpy(&data, bytes.data(), sizeof(data));
    }

    constexpr explicit ByteConvertible(std::span<const std::byte> bytes) {
        if (bytes.size() != sizeof(data)) [[unlikely]]
            throw std::invalid_argument("Illegal span size");

        std::memcpy(&data, bytes.data(), sizeof(data));
    }

    constexpr std::span<const std::byte, sizeof(data)> as_bytes() noexcept {
        return std::span<const std::byte, sizeof(data)>{
            reinterpret_cast<const std::byte*>(&data), sizeof(data)};
    }

    constexpr std::span<std::byte, sizeof(data)> as_writable_bytes() noexcept {
        return std::span<std::byte, sizeof(data)>{
            reinterpret_cast<std::byte*>(&data), sizeof(data)};
    }
};

struct CanPacket8 : ByteConvertible<std::array<uint16_t, 4>, alignof(uint64_t)> {
    struct Quarter : ByteConvertible<uint16_t> {
        using ByteConvertible::ByteConvertible;
    };

    struct PaddingQuarter : Quarter {
        PaddingQuarter()
            : Quarter(0) {}
    };

    using ByteConvertible::ByteConvertible;

    explicit CanPacket8(uint64_t data)
        : ByteConvertible(std::bit_cast<ByteConvertible>(data)) {}

    CanPacket8(Quarter q0, Quarter q1, Quarter q2, Quarter q3)
        : ByteConvertible({q0.data, q1.data, q2.data, q3.data}) {}
};
static_assert(std::atomic<CanPacket8>::is_always_lock_free);
static_assert(std::atomic<CanPacket8::Quarter>::is_always_lock_free);

} // namespace rmcs_core::hardware::device