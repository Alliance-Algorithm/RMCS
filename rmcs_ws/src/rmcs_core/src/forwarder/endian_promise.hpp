#pragma once

#include <bit>
#include <cstdint>
#include <iostream>
#include <type_traits>

namespace rmcs_core::forwarder {

template <typename T>
requires(std::is_integral_v<T> || std::is_floating_point_v<T>)
[[nodiscard]] inline T swap_endian(const T& value) noexcept {
    T result;

    if constexpr (sizeof(T) == 8) {        // 64-bit
        ((uint8_t*)&result)[0] = ((uint8_t*)&value)[7];
        ((uint8_t*)&result)[1] = ((uint8_t*)&value)[6];
        ((uint8_t*)&result)[2] = ((uint8_t*)&value)[5];
        ((uint8_t*)&result)[3] = ((uint8_t*)&value)[4];
        ((uint8_t*)&result)[4] = ((uint8_t*)&value)[3];
        ((uint8_t*)&result)[5] = ((uint8_t*)&value)[2];
        ((uint8_t*)&result)[6] = ((uint8_t*)&value)[1];
        ((uint8_t*)&result)[7] = ((uint8_t*)&value)[0];
    } else if constexpr (sizeof(T) == 4) { // 32-bit
        ((uint8_t*)&result)[0] = ((uint8_t*)&value)[3];
        ((uint8_t*)&result)[1] = ((uint8_t*)&value)[2];
        ((uint8_t*)&result)[2] = ((uint8_t*)&value)[1];
        ((uint8_t*)&result)[3] = ((uint8_t*)&value)[0];
    } else if constexpr (sizeof(T) == 2) { // 16-bit
        ((uint8_t*)&result)[0] = ((uint8_t*)&value)[1];
        ((uint8_t*)&result)[1] = ((uint8_t*)&value)[0];
    } else {
        return 0;                          // Endian swap is only defined for 2, 4, and 8-byte types
    }

    return result;
}

template <typename T, std::endian target_endian>
requires(std::is_integral_v<T> || std::is_floating_point_v<T>)
struct __attribute__((packed)) endian_t final {
    T value_buffer;

    [[nodiscard]] static T transform(const T& value) noexcept {
        if constexpr (std::endian::native == target_endian) {
            return value;
        } else {
            return swap_endian(value);
        }
    }

    endian_t() = default;

    // Storage in
    endian_t(const T& value) noexcept
        : value_buffer(transform(value)) {}

    template <typename U>
    explicit endian_t(U const& value) noexcept
        : value_buffer(transform(T(value))) {}

    // Storage out
    template <typename U>
    operator U() const noexcept {
        return U(transform(value_buffer));
    }

    operator T() const noexcept { return transform(value_buffer); }

    template <typename U>
    bool operator==(U const& o) const noexcept {
        return U(*this) == o;
    }
    template <typename U>
    bool operator!=(U const& o) const noexcept {
        return U(*this) != o;
    }

    // Arithmetic assignment operators
    endian_t& operator++() noexcept /* prefix */ {
        *this = T(*this) + T(1);
        return *this;
    }
    endian_t operator++(int) noexcept /* suffix */ {
        endian_t t(*this);
        *this = T(*this) + T(1);
        return t;
    }
    endian_t& operator--() noexcept /* prefix */ {
        *this = T(*this) - T(1);
        return *this;
    }
    endian_t operator--(int) noexcept /* suffix */ {
        endian_t t(*this);
        *this = T(*this) - T(1);
        return t;
    }

    // Compound assignment operators
    endian_t& operator+=(const T& value) noexcept {
        *this = T(*this) + value;
        return *this;
    }
    endian_t& operator-=(const T& value) noexcept {
        *this = T(*this) - value;
        return *this;
    }
    endian_t& operator*=(const T& value) noexcept {
        *this = T(*this) * value;
        return *this;
    }
    endian_t& operator/=(const T& value) noexcept {
        *this = T(*this) / value;
        return *this;
    }
    endian_t& operator%=(const T& value) noexcept {
        *this = T(*this) % value;
        return *this;
    }
    endian_t& operator&=(const T& value) noexcept {
        *this = T(*this) & value;
        return *this;
    }
    endian_t& operator|=(const T& value) noexcept {
        *this = T(*this) | value;
        return *this;
    }
    endian_t& operator^=(const T& value) noexcept {
        *this = T(*this) ^ value;
        return *this;
    }
    endian_t& operator<<=(const T& value) noexcept {
        *this = T(T(*this) << value);
        return *this;
    }
    endian_t& operator>>=(const T& value) noexcept {
        *this = T(T(*this) >> value);
        return *this;
    }
    friend std::ostream& operator<<(std::ostream& out, const endian_t value) {
        out << T(value);
        return out;
    }
    friend std::istream& operator>>(std::istream& in, endian_t& value) {
        T val;
        in >> val;
        value = val;
        return in;
    }
};

template <typename T>
requires(std::is_integral_v<T> || std::is_floating_point_v<T>)
using little_endian_t = endian_t<T, std::endian::little>;

template <typename T>
requires(std::is_integral_v<T> || std::is_floating_point_v<T>)
using big_endian_t = endian_t<T, std::endian::big>;

using le_int16_t = little_endian_t<int16_t>;
using le_int32_t = little_endian_t<int32_t>;
using le_int64_t = little_endian_t<int64_t>;

using le_uint16_t = little_endian_t<uint16_t>;
using le_uint32_t = little_endian_t<uint32_t>;
using le_uint64_t = little_endian_t<uint64_t>;

using le_float32_t = little_endian_t<float>;
using le_float64_t = little_endian_t<double>;

using be_int16_t = big_endian_t<int16_t>;
using be_int32_t = big_endian_t<int32_t>;
using be_int64_t = big_endian_t<int64_t>;

using be_uint16_t = big_endian_t<uint16_t>;
using be_uint32_t = big_endian_t<uint32_t>;
using be_uint64_t = big_endian_t<uint64_t>;

using be_float32_t = big_endian_t<float>;
using be_float64_t = big_endian_t<double>;

} // namespace endian