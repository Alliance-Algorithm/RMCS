#pragma once
#include <cstddef>
#include <string_view>

namespace rmcs_util {

template <std::size_t N>
struct StaticString {
    char data[N];

    // NOLINTBEGIN(google-explicit-constructor)
    constexpr StaticString(const char (&s)[N]) noexcept {
        for (std::size_t i = 0; i < N; ++i)
            data[i] = s[i];
    }
    // NOLINTEND(google-explicit-constructor)

    constexpr explicit operator std::string_view() const noexcept { return data; }

    constexpr auto view() const noexcept { return std::string_view{data}; }

    constexpr bool operator==(const std::string_view& o) const noexcept { return o == view(); }

    template <std::size_t M>
    constexpr bool operator==(const StaticString<M>& o) const noexcept {
        if constexpr (N != M) {
            return false;
        } else {
            return view() == o.view();
        }
    }
};
template <std::size_t N>
StaticString(const char (&)[N]) -> StaticString<N>;

} // namespace rmcs_util
