#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <type_traits>

namespace rmcs_utility {

template <typename T>
requires(std::is_integral_v<T> || std::is_floating_point_v<T>)
[[nodiscard]] inline T swap_endian(const T& value) noexcept {
    static_assert(
        sizeof(T) == 2 || sizeof(T) == 4 || sizeof(T) == 8,
        "Endian swap is only defined for 2, 4, and 8-byte types");

    T result;

    const auto* value_bytes = reinterpret_cast<const std::byte*>(&value);
    auto* result_bytes = reinterpret_cast<std::byte*>(&result);

    if constexpr (sizeof(T) == 8) {        // 64-bit
        result_bytes[0] = value_bytes[7];
        result_bytes[1] = value_bytes[6];
        result_bytes[2] = value_bytes[5];
        result_bytes[3] = value_bytes[4];
        result_bytes[4] = value_bytes[3];
        result_bytes[5] = value_bytes[2];
        result_bytes[6] = value_bytes[1];
        result_bytes[7] = value_bytes[0];
    } else if constexpr (sizeof(T) == 4) { // 32-bit
        result_bytes[0] = value_bytes[3];
        result_bytes[1] = value_bytes[2];
        result_bytes[2] = value_bytes[1];
        result_bytes[3] = value_bytes[0];
    } else if constexpr (sizeof(T) == 2) { // 16-bit
        result_bytes[0] = value_bytes[1];
        result_bytes[1] = value_bytes[0];
    } else {
        return 0;
    }

    return result;
}

template <typename T, std::endian target_endian>
requires(std::is_integral_v<T> || std::is_floating_point_v<T>)
struct [[gnu::packed]] EndianContainer final {
    T value_buffer;

    [[nodiscard]] static T transform(const T& value) noexcept {
        if constexpr (std::endian::native == target_endian) {
            return value;
        } else {
            return swap_endian(value);
        }
    }

    EndianContainer() = default;

    // Storage in
    EndianContainer(const T& value) noexcept // NOLINT(google-explicit-constructor)
        : value_buffer(transform(value)) {}

    template <typename U>
    explicit EndianContainer(U const& value) noexcept
        : value_buffer(transform(T(value))) {}

    // Storage out
    template <typename U>
    operator U() const noexcept { // NOLINT(google-explicit-constructor)
        return U(transform(value_buffer));
    }

    operator T() const noexcept { // NOLINT(google-explicit-constructor)
        return transform(value_buffer);
    }

    template <typename U>
    bool operator==(U const& o) const noexcept {
        return U(*this) == o;
    }
    template <typename U>
    bool operator!=(U const& o) const noexcept {
        return U(*this) != o;
    }

    // Arithmetic assignment operators
    EndianContainer& operator++() noexcept /* prefix */ {
        *this = T(*this) + T(1);
        return *this;
    }
    EndianContainer operator++(int) noexcept /* suffix */ {
        EndianContainer t(*this);
        *this = T(*this) + T(1);
        return t;
    }
    EndianContainer& operator--() noexcept /* prefix */ {
        *this = T(*this) - T(1);
        return *this;
    }
    EndianContainer operator--(int) noexcept /* suffix */ {
        EndianContainer t(*this);
        *this = T(*this) - T(1);
        return t;
    }

    // Compound assignment operators
    EndianContainer& operator+=(const T& value) noexcept {
        *this = T(*this) + value;
        return *this;
    }
    EndianContainer& operator-=(const T& value) noexcept {
        *this = T(*this) - value;
        return *this;
    }
    EndianContainer& operator*=(const T& value) noexcept {
        *this = T(*this) * value;
        return *this;
    }
    EndianContainer& operator/=(const T& value) noexcept {
        *this = T(*this) / value;
        return *this;
    }
    EndianContainer& operator%=(const T& value) noexcept {
        *this = T(*this) % value;
        return *this;
    }
    EndianContainer& operator&=(const T& value) noexcept {
        *this = T(*this) & value;
        return *this;
    }
    EndianContainer& operator|=(const T& value) noexcept {
        *this = T(*this) | value;
        return *this;
    }
    EndianContainer& operator^=(const T& value) noexcept {
        *this = T(*this) ^ value;
        return *this;
    }
    EndianContainer& operator<<=(const T& value) noexcept {
        *this = T(T(*this) << value);
        return *this;
    }
    EndianContainer& operator>>=(const T& value) noexcept {
        *this = T(T(*this) >> value);
        return *this;
    }
    friend std::ostream& operator<<(std::ostream& out, const EndianContainer value) {
        out << T(value);
        return out;
    }
    friend std::istream& operator>>(std::istream& in, EndianContainer& value) {
        T val;
        in >> val;
        value = val;
        return in;
    }
};

template <typename T>
requires(std::is_integral_v<T> || std::is_floating_point_v<T>)
// NOLINTNEXTLINE(readability-identifier-naming)
using little_endian_t = EndianContainer<T, std::endian::little>;

template <typename T>
requires(std::is_integral_v<T> || std::is_floating_point_v<T>)
// NOLINTNEXTLINE(readability-identifier-naming)
using big_endian_t = EndianContainer<T, std::endian::big>;

using le_int16_t = little_endian_t<int16_t>;   // NOLINT(readability-identifier-naming)
using le_int32_t = little_endian_t<int32_t>;   // NOLINT(readability-identifier-naming)
using le_int64_t = little_endian_t<int64_t>;   // NOLINT(readability-identifier-naming)

using le_uint16_t = little_endian_t<uint16_t>; // NOLINT(readability-identifier-naming)
using le_uint32_t = little_endian_t<uint32_t>; // NOLINT(readability-identifier-naming)
using le_uint64_t = little_endian_t<uint64_t>; // NOLINT(readability-identifier-naming)

using le_float32_t = little_endian_t<float>;   // NOLINT(readability-identifier-naming)
using le_float64_t = little_endian_t<double>;  // NOLINT(readability-identifier-naming)

using be_int16_t = big_endian_t<int16_t>;      // NOLINT(readability-identifier-naming)
using be_int32_t = big_endian_t<int32_t>;      // NOLINT(readability-identifier-naming)
using be_int64_t = big_endian_t<int64_t>;      // NOLINT(readability-identifier-naming)

using be_uint16_t = big_endian_t<uint16_t>;    // NOLINT(readability-identifier-naming)
using be_uint32_t = big_endian_t<uint32_t>;    // NOLINT(readability-identifier-naming)
using be_uint64_t = big_endian_t<uint64_t>;    // NOLINT(readability-identifier-naming)

using be_float32_t = big_endian_t<float>;      // NOLINT(readability-identifier-naming)
using be_float64_t = big_endian_t<double>;     // NOLINT(readability-identifier-naming)

} // namespace rmcs_utility
