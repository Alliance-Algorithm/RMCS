#pragma once

#include <cstddef>
#include <cstring>

#include <algorithm>
#include <atomic>

namespace librmcs::utility {

template <typename T>
class RingBuffer {
public:
    constexpr explicit RingBuffer(size_t size) {
        if (size <= 2)
            size = 2;
        else
            size = round_up_to_next_power_of_2(size);
        mask    = size - 1;
        storage_ = new Storage[size];
    };

    ~RingBuffer() {
        clear();
        delete[] storage_;
    }

    size_t max_size() const { return mask + 1; }

    size_t readable() const {
        return in_.load(std::memory_order::acquire) - out_.load(std::memory_order::relaxed);
    }

    size_t writeable() const {
        return max_size()
             - (in_.load(std::memory_order::relaxed) - out_.load(std::memory_order::acquire));
    }

    T* front() {
        auto out = out_.load(std::memory_order::relaxed);

        if (out == in_.load(std::memory_order::acquire))
            return nullptr;
        else
            return std::launder(reinterpret_cast<T*>(storage_[out & mask].data));
    }

    T* back() {
        auto in = in_.load(std::memory_order::acquire);

        if (in == out_.load(std::memory_order::relaxed))
            return nullptr;
        else
            return std::launder(reinterpret_cast<T*>(storage_[in & mask].data));
    }

    template <typename F>
    requires requires(F f, std::byte* storage) { f(storage); }
    size_t emplace_back_multi(F construct_functor, size_t count = -1) {
        auto in  = in_.load(std::memory_order::relaxed);
        auto out = out_.load(std::memory_order::acquire);

        auto writeable = max_size() - (in - out);

        if (count > writeable)
            count = writeable;
        if (!count)
            return 0;

        auto offset = in & mask;
        auto slice  = std::min(count, max_size() - offset);

        for (size_t i = 0; i < slice; i++)
            construct_functor(storage_[offset + i].data);
        for (size_t i = 0; i < count - slice; i++)
            construct_functor(storage_[i].data);

        std::atomic_signal_fence(std::memory_order::release);
        in_.store(in + count, std::memory_order::release);

        return count;
    }

    template <typename... Args>
    bool emplace_back(Args&&... args) {
        return emplace_back_multi(
            [&](std::byte* storage) { new (storage) T{std::forward<Args...>(args...)}; }, 1);
    }

    template <typename F>
    requires requires(F f) { T{f()}; } size_t push_back_multi(F generator, size_t count = -1) {
        return emplace_back_multi([&](std::byte* storage) { new (storage) T{generator()}; }, count);
    }

    bool push_back(const T& value) {
        return emplace_back_multi([&](std::byte* storage) { new (storage) T{value}; }, 1);
    }
    bool push_back(T&& value) {
        return emplace_back_multi(
            [&](std::byte* storage) { new (storage) T{std::move(value)}; }, 1);
    }

    template <typename F>
    requires requires(F f, T t) { f(std::move(t)); }
    size_t pop_front_multi(F callback_functor, size_t count = -1) {
        auto in  = in_.load(std::memory_order::acquire);
        auto out = out_.load(std::memory_order::relaxed);

        auto readable = in - out;
        if (count > readable)
            count = readable;
        if (!count)
            return 0;

        auto offset = out & mask;
        auto slice  = std::min(count, max_size() - offset);

        auto process = [&callback_functor](std::byte* storage) {
            auto& element = *std::launder(reinterpret_cast<T*>(storage));
            callback_functor(std::move(element));
            std::destroy_at(&element);
        };
        for (size_t i = 0; i < slice; i++)
            process(storage_[offset + i].data);
        for (size_t i = 0; i < count - slice; i++)
            process(storage_[i].data);

        std::atomic_signal_fence(std::memory_order::release);
        out_.store(out + count, std::memory_order::release);

        return count;
    }

    template <typename F>
    requires requires(F f, T t) { f(std::move(t)); } bool pop_front(F&& callback_functor) {
        return pop_front_multi(std::forward<F>(callback_functor), 1);
    }

    size_t clear() { return pop_front_multi([](T&&) {}); }

private:
    constexpr static size_t round_up_to_next_power_of_2(size_t n) {
        n--;
        n |= n >> 1;
        n |= n >> 2;
        n |= n >> 4;
        n |= n >> 8;
        n |= n >> 16;
        n |= n >> 32;
        n++;
        return n;
    }

    size_t mask;

    std::atomic<size_t> in_{0}, out_{0};
    struct Storage {
        alignas(T) std::byte data[sizeof(T)];
    }* storage_;
};

} // namespace librmcs::utility
