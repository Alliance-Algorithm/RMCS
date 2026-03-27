#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <limits>
#include <memory>
#include <new>
#include <utility>

namespace librmcs::host::utility {

// Lock-free Single-Producer/Single-Consumer (SPSC) ring buffer
// Inspired by Linux kfifo.
template <typename T>
class RingBuffer {
public:
    /*!
     * @brief Construct an SPSC ring buffer
     * @param size Minimum capacity requested. Actual capacity is rounded up
     *        to the next power of two and clamped to at least 2.
     * @note This data structure is single-producer/single-consumer. Only one
     *       thread may push, and only one thread may pop, at a time.
     */
    explicit RingBuffer(size_t size) {
        if (size <= 2)
            size = 2;
        else
            size = round_up_to_next_power_of_2(size);
        mask_ = size - 1;
        storage_ = new Storage[size];
    }

    RingBuffer(const RingBuffer&) = delete;
    RingBuffer& operator=(const RingBuffer&) = delete;
    RingBuffer(RingBuffer&&) = delete;
    RingBuffer& operator=(RingBuffer&&) = delete;

    /*!
     * @brief Destructor
     * Destroys all elements remaining in the buffer and frees storage.
     */
    ~RingBuffer() {
        clear();
        delete[] storage_;
    }

    /*!
     * @brief Capacity of the ring buffer
     * @return Total number of slots (power of two)
     */
    size_t max_size() const { return mask_ + 1; }

    /*!
     * @brief Number of elements currently readable
     * @return Count of elements available to the consumer
     * @note Uses acquire on producer index and relaxed on consumer index to
     *       ensure visibility of constructed elements to the consumer.
     */
    size_t readable() const {
        const auto in = in_.load(std::memory_order::acquire);
        const auto out = out_.load(std::memory_order::relaxed);
        return in - out;
    }

    /*!
     * @brief Number of free slots for producer
     * @return Count of slots available to write
     * @note Uses relaxed on producer index and acquire on consumer index to
     *       avoid overrun while allowing the producer to run without contention.
     */
    size_t writable() const {
        const auto in = in_.load(std::memory_order::relaxed);
        const auto out = out_.load(std::memory_order::acquire);
        return max_size() - (in - out);
    }

    /*!
     * @brief Peek the first element (consumer side)
     * @return Pointer to the first element, or nullptr if empty
     * @warning Do not call from producer thread. The pointer remains valid
     *          until the element is popped or overwritten.
     */
    T* peek_front() {
        const auto out = out_.load(std::memory_order::relaxed);

        if (out == in_.load(std::memory_order::acquire))
            return nullptr;

        return std::launder(reinterpret_cast<T*>(storage_[out & mask_].data));
    }

    /*!
     * @brief Peek the last produced element (consumer side)
     * @return Pointer to the last element, or nullptr if empty
     * @warning Do not call from producer thread. The pointer remains valid
     *          until the element is popped or overwritten.
     */
    T* peek_back() {
        const auto in = in_.load(std::memory_order::acquire);

        if (in == out_.load(std::memory_order::relaxed))
            return nullptr;

        return std::launder(reinterpret_cast<T*>(storage_[(in - 1) & mask_].data));
    }

    /*!
     * @brief Batch-construct elements at the tail (producer)
     * @tparam F Functor with signature `void(std::byte* storage)` that constructs
     *         a `T` in-place via placement-new.
     * @param count Maximum number of elements to construct (defaults to as many as fit)
     * @return Number of elements actually constructed
     * @note Producer-only. Publishes with release semantics.
     */
    template <typename F>
    requires requires(F& f, std::byte* storage) {
        { f(storage) } noexcept;
    }
    size_t emplace_back_n(F construct_functor, size_t count = std::numeric_limits<size_t>::max()) {
        const auto in = in_.load(std::memory_order::relaxed);
        const auto out = out_.load(std::memory_order::acquire);

        const auto writable = max_size() - (in - out);

        if (count > writable)
            count = writable;
        if (!count)
            return 0;

        const auto offset = in & mask_;
        const auto slice = std::min(count, max_size() - offset);

        for (size_t i = 0; i < slice; i++)
            construct_functor(storage_[offset + i].data);
        for (size_t i = 0; i < count - slice; i++)
            construct_functor(storage_[i].data);

        in_.store(in + count, std::memory_order::release);

        return count;
    }

    /*!
     * @brief Construct one element in-place at the tail (producer)
     * @return true if pushed, false if buffer is full
     */
    template <typename... Args>
    bool emplace_back(Args&&... args) {
        return emplace_back_n(
            [&](std::byte* storage) noexcept(noexcept(T{std::forward<Args>(args)...})) {
                new (storage) T{std::forward<Args>(args)...};
            },
            1);
    }

    /*!
     * @brief Batch-push using a generator (producer)
     * @tparam F Functor returning a `T` to be stored
     * @param count Maximum number to generate/push
     * @return Number of elements actually pushed
     */
    template <typename F>
    requires requires(F& f) {
        { f() } noexcept;
        { T{f()} } noexcept;
    } size_t push_back_n(F generator, size_t count = std::numeric_limits<size_t>::max()) {
        return emplace_back_n(
            [&](std::byte* storage) noexcept(noexcept(T{generator()})) {
                new (storage) T{generator()};
            },
            count);
    }

    /*!
     * @brief Push a copy of value (producer)
     * @return true if pushed, false if buffer is full
     */
    bool push_back(const T& value) {
        return emplace_back_n(
            [&](std::byte* storage) noexcept(noexcept(T{value})) { new (storage) T{value}; }, 1);
    }
    /*!
     * @brief Push by moving value (producer)
     * @return true if pushed, false if buffer is full
     */
    bool push_back(T&& value) {
        return emplace_back_n(
            [&](std::byte* storage) noexcept(noexcept(T{std::move(value)})) {
                new (storage) T{std::move(value)};
            },
            1);
    }

    /*!
     * @brief Batch-pop elements from the head (consumer)
     * @tparam F Functor with signature `void(T)` receiving moved-out elements
     * @param count Maximum number of elements to pop (defaults to all available)
     * @return Number of elements actually popped
     * @note Consumer-only. Consumes with release on `out_` and destroys elements.
     */
    template <typename F>
    requires requires(F& f, T& t) {
        { f(std::move(t)) } noexcept;
    } size_t pop_front_n(F callback_functor, size_t count = std::numeric_limits<size_t>::max()) {
        const auto in = in_.load(std::memory_order::acquire);
        const auto out = out_.load(std::memory_order::relaxed);

        const auto readable = in - out;
        count = std::min(count, readable);
        if (!count)
            return 0;

        const auto offset = out & mask_;
        const auto slice = std::min(count, max_size() - offset);

        auto process = [&callback_functor](std::byte* storage) {
            auto& element = *std::launder(reinterpret_cast<T*>(storage));
            callback_functor(std::move(element));
            std::destroy_at(&element);
        };
        for (size_t i = 0; i < slice; i++)
            process(storage_[offset + i].data);
        for (size_t i = 0; i < count - slice; i++)
            process(storage_[i].data);

        out_.store(out + count, std::memory_order::release);

        return count;
    }

    /*!
     * @brief Pop one element (consumer)
     * @return true if an element was popped, false if empty
     */
    template <typename F>
    requires requires(F& f, T& t) {
        { f(std::move(t)) } noexcept;
    } bool pop_front(F&& callback_functor) {
        return pop_front_n(std::forward<F>(callback_functor), 1);
    }

    /*!
     * @brief Clear the buffer by consuming all elements
     * @return Number of elements that were erased
     */
    size_t clear() {
        return pop_front_n([](const T&) noexcept {});
    }

private:
    /*!
     * @brief Round up to next power of two
     * @note Assumes n > 0. Handles 32/64-bit size_t.
     */
    constexpr static size_t round_up_to_next_power_of_2(size_t n) {
        n--;
        n |= n >> 1;
        n |= n >> 2;
        n |= n >> 4;
        n |= n >> 8;
        n |= n >> 16;
        if constexpr (sizeof(size_t) > 4)
            n |= n >> 32;
        n++;
        return n;
    }

    size_t mask_;
    struct Storage {
        alignas(T) std::byte data[sizeof(T)];
    }* storage_;

    std::atomic<size_t> in_{0}, out_{0};
};

} // namespace librmcs::host::utility
