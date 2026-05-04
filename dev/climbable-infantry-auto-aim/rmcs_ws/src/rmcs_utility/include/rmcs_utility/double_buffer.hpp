#pragma once

#include <cstdint>
#include <cstring>

#include <algorithm>
#include <atomic>
#include <type_traits>

namespace rmcs_utility {

/// @brief Implements a reliable, lock-free, arbitrary-length cross-thread buffer.
/// @tparam T The type of data to be stored in the buffer.
/// @tparam i_am_very_sure_this_is_trivially_copyable_and_destructible A flag to bypass type checks.
///         Set to `true` only if you are certain that `T` is trivially copyable and destructible.
template <typename T, bool i_am_very_sure_this_is_trivially_copyable_and_destructible = false>
requires(std::is_trivially_copyable_v<T> && std::is_trivially_destructible_v<T>)
     || i_am_very_sure_this_is_trivially_copyable_and_destructible class DoubleBuffer {
public:
    /// @brief Writes data to the buffer in a thread-safe manner.
    /// @param data The data to be written to the buffer.
    void write(const T& data) noexcept {
        const uint64_t current = current_.load(std::memory_order_relaxed);
        const uint64_t next    = current + 1;
        std::memcpy(&buffers_[next & 1], &data, sizeof(T));
        current_.store(next, std::memory_order_release);
    }

    /// @brief Reads data from the buffer in a thread-safe manner.
    /// @param data The variable where the read data will be stored.
    /// @return Returns `true` if the read was successful, `false` otherwise.
    /// @note Under high write pressure, consumers may occasionally read incorrect data.
    ///       The function explicitly returns `false` to signal failure. Readers can choose to retry
    ///       (e.g., using a `while` loop) or cancel the operation.
    /// @note If the function returns `false`, the variable `data` will be filled with invalid data.
    bool read(T& data) noexcept {
        const uint64_t current = current_.load(std::memory_order_acquire);
        std::memcpy(&data, &buffers_[current & 1], sizeof(T));
        return current_.load(std::memory_order_acquire) == current;
    }

private:
    /// @brief Internal buffer structure to hold the data.
    struct alignas(T) Buffer {
        std::byte data[sizeof(T)];
    };

    Buffer buffers_[2];
    std::atomic<uint64_t> current_{0};
};

} // namespace rmcs_utility