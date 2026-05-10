#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <memory>
#include <new>
#include <type_traits>
#include <utility>

namespace rmcs_utility {

template <typename T>
requires(
    !std::is_reference_v<T> && !std::is_const_v<T> && !std::is_volatile_v<T>
    && !std::is_unbounded_array_v<T> && std::is_nothrow_destructible_v<T>) class MemoryPool {
public:
    /*!
     * @brief Unique ownership wrapper for pool-allocated objects
     * @note All returned smart pointers must be destroyed before the owning
     *       MemoryPool is destroyed.
     */
    struct ReturnToPool {
        MemoryPool* pool{nullptr};

        void operator()(T* pointer) const noexcept {
            if (pool != nullptr && pointer != nullptr)
                pool->free(pointer);
        }
    };

    using UniquePtr = std::unique_ptr<T, ReturnToPool>;

    /*!
     * @brief Construct a fixed-capacity memory pool
     * @param capacity Number of object slots to allocate
     * @note This container is not thread-safe.
     */
    explicit MemoryPool(size_t capacity)
        : capacity_(capacity)
        , storage_(new Storage[capacity])
        , free_stack_(new size_t[capacity])
        , occupied_(new bool[capacity]{}) {
        reset_free_stack();
    }

    MemoryPool(const MemoryPool&) = delete;
    MemoryPool& operator=(const MemoryPool&) = delete;
    MemoryPool(MemoryPool&&) = delete;
    MemoryPool& operator=(MemoryPool&&) = delete;

    /*!
     * @brief Destructor
     * @note All objects must be returned to the pool before destruction.
     *       Violating this lifetime contract terminates the program.
     */
    ~MemoryPool() noexcept {
        if (!empty()) {
            std::fprintf(
                stderr,
                "rmcs_utility::MemoryPool %p destroyed with outstanding objects: size=%zu, "
                "capacity=%zu\n",
                static_cast<const void*>(this), size_, capacity_);
            std::fflush(stderr);
            std::terminate();
        }
    }

    /*!
     * @brief Capacity of the memory pool
     * @return Total number of object slots
     */
    [[nodiscard]] size_t max_size() const noexcept { return capacity_; }

    /*!
     * @brief Number of currently allocated objects
     */
    [[nodiscard]] size_t size() const noexcept { return size_; }

    /*!
     * @brief Number of free slots remaining
     */
    [[nodiscard]] size_t available() const noexcept { return capacity_ - size_; }

    /*!
     * @brief Check whether the pool contains no live object
     */
    [[nodiscard]] bool empty() const noexcept { return size_ == 0; }

    /*!
     * @brief Check whether the pool has no free slot
     */
    [[nodiscard]] bool full() const noexcept { return size_ == capacity_; }

    /*!
     * @brief Construct one object in-place inside the pool
     * @return Pointer to the constructed object, or nullptr if the pool is full
     */
    template <typename... Args>
    [[nodiscard]] T* emplace(Args&&... args) noexcept(noexcept(T{std::forward<Args>(args)...})) {
        if (full())
            return nullptr;

        const auto index = free_stack_[top_ - 1];
        auto* pointer = new (storage_[index].data) T{std::forward<Args>(args)...};

        occupied_[index] = true;
        --top_;
        ++size_;

        return pointer;
    }

    /*!
     * @brief Default-construct one object in-place inside the pool
     * @return Pointer to the constructed object, or nullptr if the pool is full
     */
    [[nodiscard]] T* allocate() noexcept(noexcept(T{})) requires std::default_initializable<T> {
        return emplace();
    }

    /*!
     * @brief Construct one object and return unique ownership bound to the pool
     * @return Smart pointer owning the object, or a null smart pointer if the pool is full
     */
    template <typename... Args>
    [[nodiscard]] UniquePtr
        make_unique(Args&&... args) noexcept(noexcept(T{std::forward<Args>(args)...})) {
        if (auto* pointer = emplace(std::forward<Args>(args)...); pointer != nullptr)
            return UniquePtr(pointer, ReturnToPool{this});
        return UniquePtr(nullptr, ReturnToPool{this});
    }

    /*!
     * @brief Default-construct one object and return unique ownership bound to the pool
     * @return Smart pointer owning the object, or a null smart pointer if the pool is full
     */
    [[nodiscard]] UniquePtr allocate_unique() noexcept(noexcept(T{}))
        requires std::default_initializable<T> {
        return make_unique();
    }

    /*!
     * @brief Release an object back to the pool
     * @param pointer Pointer previously returned by this pool
     * @return true if the object was released, false if the pointer was invalid
     */
    bool free(T* pointer) noexcept {
        const auto index = pointer_to_index(pointer);

        if (index == capacity_ || !occupied_[index])
            return false;

        std::destroy_at(std::launder(pointer));
        occupied_[index] = false;
        free_stack_[top_] = index;
        ++top_;
        --size_;

        return true;
    }

    /*!
     * @brief Check whether a pointer refers to a live object owned by this pool
     */
    [[nodiscard]] bool contains(const T* pointer) const noexcept {
        const auto index = pointer_to_index(pointer);
        return index != capacity_ && occupied_[index];
    }

    /*!
     * @brief Destroy every live object and reset the pool
     * @return Number of objects that were destroyed
     */
    size_t clear() noexcept {
        size_t count = 0;

        for (size_t i = 0; i < capacity_; i++) {
            if (occupied_[i]) {
                std::destroy_at(slot_pointer(i));
                occupied_[i] = false;
                ++count;
            }
        }

        size_ = 0;
        reset_free_stack();

        return count;
    }

private:
    struct Storage {
        alignas(T) std::byte data[sizeof(T)];
    };

    [[nodiscard]] T* slot_pointer(size_t index) noexcept {
        return std::launder(reinterpret_cast<T*>(storage_[index].data));
    }

    [[nodiscard]] const T* slot_pointer(size_t index) const noexcept {
        return std::launder(reinterpret_cast<const T*>(storage_[index].data));
    }

    [[nodiscard]] size_t pointer_to_index(const T* pointer) const noexcept {
        if (pointer == nullptr)
            return capacity_;

        const auto begin = reinterpret_cast<uintptr_t>(storage_.get());
        const auto end = begin + sizeof(Storage) * capacity_;
        const auto target = reinterpret_cast<uintptr_t>(pointer);

        if (target < begin || target >= end)
            return capacity_;

        const auto offset = target - begin;
        if (offset % sizeof(Storage) != 0)
            return capacity_;

        return static_cast<size_t>(offset / sizeof(Storage));
    }

    void reset_free_stack() noexcept {
        for (size_t i = 0; i < capacity_; i++)
            free_stack_[capacity_ - 1 - i] = i;
        top_ = capacity_;
    }

    size_t capacity_;
    std::unique_ptr<Storage[]> storage_;
    std::unique_ptr<size_t[]> free_stack_;
    std::unique_ptr<bool[]> occupied_;

    size_t top_{0};
    size_t size_{0};
};

} // namespace rmcs_utility
