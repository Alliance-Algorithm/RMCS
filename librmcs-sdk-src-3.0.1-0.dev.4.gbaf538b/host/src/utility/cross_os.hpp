#pragma once

namespace librmcs::host::utility {

#ifdef _MSC_VER
# define PACKED_STRUCT(...) __pragma(pack(push, 1)) struct __VA_ARGS__ __pragma(pack(pop))
#elif defined(__GNUC__)
# define PACKED_STRUCT(...) struct __attribute__((packed)) __VA_ARGS__
#endif

constexpr static bool is_linux() {
#ifdef __linux__
    return true;
#else
    return false;
#endif
}

constexpr static bool is_windows() {
#if defined(_WIN32) || defined(WIN32)
    return true;
#else
    return false;
#endif
}

#if defined(_MSC_VER)
# define ALWAYS_INLINE __forceinline
#elif defined(__GNUC__) || defined(__clang__)
# define ALWAYS_INLINE __attribute__((always_inline)) inline
#else
# define ALWAYS_INLINE inline
#endif

} // namespace librmcs::host::utility
