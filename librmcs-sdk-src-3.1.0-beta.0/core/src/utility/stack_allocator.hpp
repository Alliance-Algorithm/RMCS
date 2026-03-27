#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "core/src/utility/assert.hpp"

namespace librmcs::core::utility {

template <bool debug>
class BasicStackAllocator;

template <>
class BasicStackAllocator<false> {
public:
    constexpr explicit BasicStackAllocator(std::span<std::byte> buffer)
        : top_(buffer.data())
        , end_(buffer.data() + buffer.size()) {}

    BasicStackAllocator(const BasicStackAllocator&) = delete;
    BasicStackAllocator& operator=(const BasicStackAllocator&) = delete;
    BasicStackAllocator(BasicStackAllocator&&) = delete;
    BasicStackAllocator& operator=(BasicStackAllocator&&) = delete;
    ~BasicStackAllocator() = default;

    constexpr void* allocate(std::size_t n) noexcept {
        constexpr std::size_t align = alignof(std::max_align_t);
        n = (n + align - 1) & ~(align - 1);

        const std::size_t remaining = static_cast<std::size_t>(end_ - top_);
        if (remaining < n) [[unlikely]]
            return nullptr;

        void* p = top_;
        top_ += n;
        return p;
    }

    constexpr void deallocate(void* p, size_t n) noexcept {
        (void)n;
        top_ = static_cast<std::byte*>(p);
    }

protected:
    std::byte *top_, *end_;
};

template <>
class BasicStackAllocator<true> : private BasicStackAllocator<false> {
    using Impl = BasicStackAllocator<false>;

public:
    constexpr explicit BasicStackAllocator(std::span<std::byte> buffer)
        : Impl(buffer) {}

    void* allocate(std::size_t n) noexcept {
        assert_always(n > 0);
        void* p = Impl::allocate(n);
        if (!p)
            return nullptr;
        assert_always(reinterpret_cast<std::uintptr_t>(p) % alignof(std::max_align_t) == 0);

        assert_always(lifo_check_depth_ < kMaxAllocs);
        lifo_check_stack_[lifo_check_depth_++] = p;

        return p;
    }

    void deallocate(void* p, std::size_t n) noexcept {
        assert_always(p);
        assert_always(lifo_check_depth_ > 0);
        assert_always(p == lifo_check_stack_[lifo_check_depth_ - 1]);

        constexpr std::size_t align = alignof(std::max_align_t);
        n = (n + align - 1) & ~(align - 1);
        assert_always(p == Impl::top_ - n);

        Impl::deallocate(p, n);
        --lifo_check_depth_;
    }

private:
    static constexpr std::size_t kMaxAllocs = 16;
    void* lifo_check_stack_[kMaxAllocs]{};
    std::size_t lifo_check_depth_ = 0;
};

#ifdef NDEBUG
using StackAllocator = BasicStackAllocator<false>;
#else
using StackAllocator = BasicStackAllocator<true>;
#endif

} // namespace librmcs::core::utility
