#pragma once
#include <memory>

namespace internal {
inline auto use_memory_header() {
    // Remove warning from unused include
    std::ignore = std::unique_ptr<int>();
}
} // namespace internal

#define RMCS_PIMPL_DEFINITION(CLASS)         \
public:                                      \
    explicit CLASS() noexcept;               \
    ~CLASS() noexcept;                       \
    CLASS(const CLASS&) = delete;            \
    CLASS& operator=(const CLASS&) = delete; \
                                             \
private:                                     \
    struct Impl;                             \
    std::unique_ptr<Impl> pimpl;
