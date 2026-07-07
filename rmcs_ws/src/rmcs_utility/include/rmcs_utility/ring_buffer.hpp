#pragma once

#include <algorithm>
#include <atomic>
#include <compare>
#include <cstddef>
#include <iterator>
#include <limits>
#include <memory>
#include <new>
#include <type_traits>
#include <utility>

namespace rmcs_utility {

// Lock-free Single-Producer/Single-Consumer (SPSC) ring buffer
// Inspired by Linux kfifo.
template <typename T>
class RingBuffer {
public:
    template <bool is_const>
    class ReadableView;

    template <bool is_const>
    class BasicIterator {
        using Buffer = std::conditional_t<is_const, const RingBuffer, RingBuffer>;

    public:
        using iterator_category = std::random_access_iterator_tag;
        using iterator_concept = std::random_access_iterator_tag;
        using value_type = T;
        using difference_type = std::ptrdiff_t;
        using reference = std::conditional_t<is_const, const T&, T&>;
        using pointer = std::conditional_t<is_const, const T*, T*>;

        BasicIterator() = default;
        BasicIterator(const BasicIterator&) = default;
        BasicIterator(BasicIterator&&) = default;
        BasicIterator& operator=(const BasicIterator&) = default;
        BasicIterator& operator=(BasicIterator&&) = default;

        // NOLINTNEXTLINE(google-explicit-constructor)
        BasicIterator(const BasicIterator<false>& other) requires is_const
            : buffer_(other.buffer_)
            , origin_(other.origin_)
            , offset_(other.offset_) {}

        reference operator*() const { return *ptr(); }
        pointer operator->() const { return ptr(); }
        reference operator[](difference_type n) const { return *(*this + n); }

        BasicIterator& operator++() {
            offset_++;
            return *this;
        }

        BasicIterator operator++(int) {
            auto old = *this;
            ++*this;
            return old;
        }

        BasicIterator& operator--() {
            offset_--;
            return *this;
        }

        BasicIterator operator--(int) {
            auto old = *this;
            --*this;
            return old;
        }

        BasicIterator& operator+=(difference_type n) {
            if (n >= 0)
                offset_ += static_cast<size_t>(n);
            else
                offset_ -= static_cast<size_t>(-(n + 1)) + 1;
            return *this;
        }

        BasicIterator& operator-=(difference_type n) {
            if (n >= 0)
                offset_ -= static_cast<size_t>(n);
            else
                offset_ += static_cast<size_t>(-(n + 1)) + 1;
            return *this;
        }

        friend BasicIterator operator+(BasicIterator it, difference_type n) {
            it += n;
            return it;
        }

        friend BasicIterator operator+(difference_type n, BasicIterator it) {
            it += n;
            return it;
        }

        friend BasicIterator operator-(BasicIterator it, difference_type n) {
            it -= n;
            return it;
        }

        friend difference_type operator-(BasicIterator lhs, BasicIterator rhs) {
            if (lhs.offset_ >= rhs.offset_)
                return static_cast<difference_type>(lhs.offset_ - rhs.offset_);
            return -static_cast<difference_type>(rhs.offset_ - lhs.offset_);
        }

        friend bool operator==(BasicIterator lhs, BasicIterator rhs) {
            return lhs.buffer_ == rhs.buffer_ && lhs.origin_ == rhs.origin_
                && lhs.offset_ == rhs.offset_;
        }

        friend std::strong_ordering operator<=>(BasicIterator lhs, BasicIterator rhs) {
            return lhs.offset_ <=> rhs.offset_;
        }

    private:
        friend class RingBuffer;
        template <bool>
        friend class BasicIterator;
        template <bool>
        friend class ReadableView;

        BasicIterator(Buffer* buffer, size_t origin, size_t offset)
            : buffer_(buffer)
            , origin_(origin)
            , offset_(offset) {}

        pointer ptr() const {
            return std::launder(
                reinterpret_cast<pointer>(
                    buffer_->storage_[(origin_ + offset_) & buffer_->mask_].data));
        }

        Buffer* buffer_ = nullptr;
        size_t origin_ = 0;
        size_t offset_ = 0;
    };

    using iterator = BasicIterator<false>;
    using const_iterator = BasicIterator<true>;

    template <bool is_const>
    class ReadableView {
        using Buffer = std::conditional_t<is_const, const RingBuffer, RingBuffer>;

    public:
        using iterator = BasicIterator<is_const>;
        using value_type = T;
        using difference_type = std::ptrdiff_t;
        using size_type = size_t;
        using reference = std::conditional_t<is_const, const T&, T&>;

        ReadableView() = default;
        ReadableView(const ReadableView&) = default;
        ReadableView(ReadableView&&) = default;
        ReadableView& operator=(const ReadableView&) = default;
        ReadableView& operator=(ReadableView&&) = default;

        // NOLINTNEXTLINE(google-explicit-constructor)
        ReadableView(const ReadableView<false>& other) requires is_const
            : buffer_(other.buffer_)
            , origin_(other.origin_)
            , size_(other.size_) {}

        iterator begin() const { return iterator{buffer_, origin_, 0}; }
        iterator end() const { return iterator{buffer_, origin_, size_}; }

        [[nodiscard]] bool empty() const { return size_ == 0; }
        [[nodiscard]] size_type size() const { return size_; }

        reference front() const { return *begin(); }
        reference back() const { return *(end() - 1); }
        reference operator[](size_type index) const { return begin()[index]; }

    private:
        friend class RingBuffer;
        template <bool>
        friend class ReadableView;

        ReadableView(Buffer* buffer, size_t origin, size_t size)
            : buffer_(buffer)
            , origin_(origin)
            , size_(size) {}

        Buffer* buffer_ = nullptr;
        size_t origin_ = 0;
        size_t size_ = 0;
    };

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
     * @brief Snapshot view of elements currently readable by the consumer
     * @note Captures [out, in) once. Producer pushes do not extend the returned view.
     *       Consumer pops invalidate iterators to erased elements.
     */
    ReadableView<false> readable_view() noexcept {
        const auto in = in_.load(std::memory_order::acquire);
        const auto out = out_.load(std::memory_order::relaxed);
        return {this, out, in - out};
    }

    /*!
     * @brief Const snapshot view of elements currently readable by the consumer
     */
    ReadableView<true> readable_view() const noexcept {
        const auto in = in_.load(std::memory_order::acquire);
        const auto out = out_.load(std::memory_order::relaxed);
        return {this, out, in - out};
    }

    /*!
     * @brief Explicit const snapshot view for non-const buffers
     */
    ReadableView<true> const_readable_view() const noexcept { return readable_view(); }

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
     * @brief Visit elements from the head without consuming them
     * @tparam F Functor with signature `void(T&)`
     * @param callback_functor Invoked for each readable element in FIFO order
     * @param count Maximum number of elements to inspect (defaults to all available)
     * @return Number of elements actually visited
     * @note Consumer-only. Does not advance `out_` or destroy elements.
     *       Mutations performed through the callback are applied in place to the
     *       buffered elements.
     */
    template <typename F>
    requires requires(F& f, T& t) {
        { f(t) } noexcept;
    } size_t peek_front_n(F callback_functor, size_t count = std::numeric_limits<size_t>::max()) {
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
            callback_functor(element);
        };
        for (size_t i = 0; i < slice; i++)
            process(storage_[offset + i].data);
        for (size_t i = 0; i < count - slice; i++)
            process(storage_[i].data);

        return count;
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
     * @brief Pop readable elements before a snapshot iterator
     * @return Number of elements erased from the front
     * @note Consumer-only. `pos` must come from this buffer's current readable range.
     */
    size_t pop_front_until(const_iterator pos) {
        if (pos.buffer_ != this)
            return 0;

        const auto in = in_.load(std::memory_order::acquire);
        const auto out = out_.load(std::memory_order::relaxed);
        const auto count = pos.origin_ + pos.offset_ - out;
        if (count > in - out)
            return 0;

        return pop_front_n([](const T&) noexcept {}, count);
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

} // namespace rmcs_utility
