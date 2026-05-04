#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <new>
#include <type_traits>
#include <utility>

namespace rmcs_core::referee::command {

class Field {
public:
    constexpr Field() { write_ = &do_nothing; }

    template <typename F>
    requires requires(const F& f, std::byte* buffer) {
        { f(buffer) } -> std::convertible_to<size_t>;
    } constexpr explicit Field(F functor) {
        static_assert(sizeof(functor) <= sizeof(intptr_t));
        static_assert(std::is_trivially_destructible_v<F>);

        ::new (&storage_) F(std::move(functor));
        write_ = [](const intptr_t& storage, std::byte* buffer) {
            return (*std::launder(reinterpret_cast<const F*>(&storage)))(buffer);
        };
    }

    [[nodiscard]] constexpr explicit operator bool() const { return write_ != &do_nothing; }
    [[nodiscard]] constexpr bool empty() const { return write_ == &do_nothing; }

    size_t write(std::byte* buffer) const { return write_(storage_, buffer); };

private:
    static constexpr size_t do_nothing(const intptr_t&, std::byte*) { return 0; };

    intptr_t storage_;
    size_t (*write_)(const intptr_t&, std::byte*);
};

inline size_t write_field(std::byte*) { return 0; }

template <typename... Ts>
inline size_t write_field(std::byte*, const Field&, const Ts&...);
template <typename T, typename... Ts>
inline size_t write_field(std::byte*, const T&, const Ts&...);

template <typename... Ts>
inline size_t write_field(std::byte* buffer, const Field& package, const Ts&... other_data) {
    auto written = package.write(buffer);
    return written + write_field(buffer + written, other_data...);
}
template <typename T, typename... Ts>
inline size_t write_field(std::byte* buffer, const T& data, const Ts&... other_data) {
    memcpy(buffer, &data, sizeof(data));
    return sizeof(data) + write_field(buffer + sizeof(data), other_data...);
}

#define MAKE_FIELD(...)                                                        \
    ::rmcs_core::referee::command::Field {                                     \
        [this](std::byte* buffer) { return write_field(buffer, __VA_ARGS__); } \
    }

} // namespace rmcs_core::referee::command