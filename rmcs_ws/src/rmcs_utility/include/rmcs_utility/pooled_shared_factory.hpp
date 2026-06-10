#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <new>
#include <type_traits>
#include <utility>

#include "rmcs_utility/memory_pool.hpp"

namespace rmcs_utility {

template <typename T>
requires std::is_nothrow_destructible_v<T> class PooledSharedFactory {
public:
    explicit PooledSharedFactory(size_t capacity)
        : pool_(std::make_shared<PoolImpl>(capacity)) {}

    template <typename... Args>
    [[nodiscard]] std::shared_ptr<T> make(Args&&... args) {
        auto ptr = try_make(std::forward<Args>(args)...);
        if (!ptr)
            throw std::bad_alloc{};
        return ptr;
    }

    template <typename... Args>
    [[nodiscard]] std::shared_ptr<T> try_make(Args&&... args) {
        void* storage = nullptr;
        {
            auto guard = std::scoped_lock{pool_->mutex};
            storage = pool_->allocate();
        }

        if (!storage)
            return nullptr;

        try {
            return std::shared_ptr<T>(
                std::construct_at(static_cast<T*>(storage), std::forward<Args>(args)...),
                [pool = pool_](T* ptr) {
                    std::destroy_at(ptr);
                    auto guard = std::scoped_lock{pool->mutex};
                    pool->free(ptr);
                });
        } catch (...) {
            auto guard = std::scoped_lock{pool_->mutex};
            pool_->free(storage);
            throw;
        }
    }

    [[nodiscard]] size_t max_size() const noexcept { return pool_->max_size(); }

private:
    class PoolImpl final : public MemoryPool<sizeof(T), alignof(T)> {
    public:
        using MemoryPool<sizeof(T), alignof(T)>::MemoryPool;

        mutable std::mutex mutex;
    };

    std::shared_ptr<PoolImpl> pool_;
};

} // namespace rmcs_utility
