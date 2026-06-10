#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>

namespace rmcs_utility {

template <size_t Size, size_t Align>
class MemoryPool {
    static_assert(Size > 0, "MemoryPool slot size must be greater than zero");
    static_assert(Align > 0, "MemoryPool slot alignment must be greater than zero");
    static_assert((Align & (Align - 1)) == 0, "MemoryPool slot alignment must be a power of two");

public:
    /*!
     * @brief Construct a fixed-capacity raw storage pool
     * @param capacity Number of storage slots to allocate
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
     * @note All allocated slots must be returned to the pool before destruction.
     *       Violating this lifetime contract terminates the program.
     */
    ~MemoryPool() noexcept {
        if (!empty()) {
            std::fprintf(
                stderr,
                "rmcs_utility::MemoryPool %p destroyed with outstanding slots: size=%zu, "
                "capacity=%zu\n",
                static_cast<const void*>(this), size_, capacity_);
            std::fflush(stderr);
            std::terminate();
        }
    }

    /*!
     * @brief Payload size of each storage slot in bytes
     */
    [[nodiscard]] static constexpr size_t slot_size() noexcept { return Size; }

    /*!
     * @brief Alignment guarantee of each storage slot in bytes
     */
    [[nodiscard]] static constexpr size_t slot_align() noexcept { return Align; }

    /*!
     * @brief Capacity of the memory pool
     * @return Total number of storage slots
     */
    [[nodiscard]] size_t max_size() const noexcept { return capacity_; }

    /*!
     * @brief Number of currently allocated storage slots
     */
    [[nodiscard]] size_t size() const noexcept { return size_; }

    /*!
     * @brief Number of free storage slots remaining
     */
    [[nodiscard]] size_t available() const noexcept { return capacity_ - size_; }

    /*!
     * @brief Check whether the pool contains no allocated slot
     */
    [[nodiscard]] bool empty() const noexcept { return size_ == 0; }

    /*!
     * @brief Check whether the pool has no free slot
     */
    [[nodiscard]] bool full() const noexcept { return size_ == capacity_; }

    /*!
     * @brief Allocate one raw storage slot from the pool
     * @return Pointer to the slot's raw storage, or nullptr if the pool is full
     */
    [[nodiscard]] void* allocate() noexcept {
        if (full())
            return nullptr;

        const auto index = free_stack_[top_ - 1];
        occupied_[index] = true;
        --top_;
        ++size_;

        return static_cast<void*>(slot_pointer(index));
    }

    /*!
     * @brief Release a raw storage slot back to the pool
     * @param pointer Pointer previously returned by this pool
     * @return true if the slot was released, false if the pointer was invalid
     * @note The pointer must refer to the start address of a live slot.
     */
    bool free(void* pointer) noexcept {
        const auto index = pointer_to_index(pointer);

        if (index == capacity_ || !occupied_[index])
            return false;

        occupied_[index] = false;
        free_stack_[top_] = index;
        ++top_;
        --size_;

        return true;
    }

    /*!
     * @brief Check whether a pointer refers to a live slot owned by this pool
     */
    [[nodiscard]] bool contains(const void* pointer) const noexcept {
        const auto index = pointer_to_index(pointer);
        return index != capacity_ && occupied_[index];
    }

    /*!
     * @brief Release every live slot and reset the pool
     * @return Number of slots that were released
     */
    size_t clear() noexcept {
        size_t count = 0;

        for (size_t i = 0; i < capacity_; i++) {
            if (occupied_[i]) {
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
        alignas(Align) std::byte data[Size];
    };

    [[nodiscard]] std::byte* slot_pointer(size_t index) noexcept { return storage_[index].data; }

    [[nodiscard]] const std::byte* slot_pointer(size_t index) const noexcept {
        return storage_[index].data;
    }

    [[nodiscard]] size_t pointer_to_index(const void* pointer) const noexcept {
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
