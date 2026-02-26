#pragma once

namespace librmcs::utility {

#ifdef _MSC_VER
# define PACKED_STRUCT(...) __pragma(pack(push, 1)) struct __VA_ARGS__ __pragma(pack(pop))
#elif defined(__GNUC__)
# define PACKED_STRUCT(...) struct __attribute__((packed)) __VA_ARGS__
#endif

constexpr static inline bool is_linux() {
#ifdef __linux__
    return true;
#else
    return false;
#endif
}

constexpr static inline bool is_windows() {
#if defined(_WIN32) || defined(WIN32)
    return true;
#else
    return false;
#endif
}

} // namespace librmcs::utility
