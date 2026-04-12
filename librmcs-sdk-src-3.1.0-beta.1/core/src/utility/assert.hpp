#pragma once

#include <source_location>
#include <type_traits>
#include <utility>

#ifndef NDEBUG
# include <functional>
#endif

namespace librmcs::core::utility {

[[noreturn]] void assert_func(const std::source_location& location);

[[noreturn]] constexpr void
    assert_failed_always(const std::source_location& location = std::source_location::current()) {
    assert_func(location);
}

[[noreturn]] inline void
    assert_failed_debug(const std::source_location& location = std::source_location::current()) {
#ifdef NDEBUG
    (void)location;
    std::unreachable();
#else
    assert_func(location);
#endif
}

constexpr void assert_always(
    bool condition, const std::source_location& location = std::source_location::current()) {
    if (!condition) [[unlikely]]
        assert_func(location);
}

constexpr void assert_debug(
    bool condition, const std::source_location& location = std::source_location::current()) {
#ifdef NDEBUG
    [[assume(condition)]];
    (void)location;
#else
    assert_always(condition, location);
#endif
}

// Debug-only lazy assertion: The predicate is evaluated only in debug builds.
template <typename Condition>
requires std::is_nothrow_invocable_r_v<bool, Condition&&> inline void assert_debug_lazy(
    Condition&& condition, const std::source_location& location = std::source_location::current()) {
#ifdef NDEBUG
    (void)condition;
    (void)location;
#else
    assert_always(static_cast<bool>(std::invoke(std::forward<Condition>(condition))), location);
#endif
}

} // namespace librmcs::core::utility
