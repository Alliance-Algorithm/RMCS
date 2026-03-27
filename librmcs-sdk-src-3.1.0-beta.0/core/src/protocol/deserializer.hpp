#pragma once

#include <coroutine>
#include <cstddef>
#include <cstring>
#include <span>
#include <utility>

#include "core/include/librmcs/data/datas.hpp"
#include "core/src/coroutine/lifo.hpp"
#include "core/src/protocol/constant.hpp"
#include "core/src/protocol/protocol.hpp"
#include "core/src/utility/assert.hpp"

namespace librmcs::core::protocol {

class DeserializeCallback {
public:
    DeserializeCallback() = default;
    DeserializeCallback(const DeserializeCallback&) = delete;
    DeserializeCallback& operator=(const DeserializeCallback&) = delete;
    DeserializeCallback(DeserializeCallback&&) = delete;
    DeserializeCallback& operator=(DeserializeCallback&&) = delete;
    virtual ~DeserializeCallback() = default;

    virtual void can_deserialized_callback(FieldId id, const data::CanDataView& data) = 0;

    virtual void uart_deserialized_callback(FieldId id, const data::UartDataView& data) = 0;

    virtual void gpio_digital_deserialized_callback(const data::GpioDigitalDataView& data) = 0;

    virtual void gpio_analog_deserialized_callback(const data::GpioAnalogDataView& data) = 0;

    virtual void accelerometer_deserialized_callback(const data::AccelerometerDataView& data) = 0;

    virtual void gyroscope_deserialized_callback(const data::GyroscopeDataView& data) = 0;

    virtual void error_callback() = 0;
};

class Deserializer : private coroutine::InlineLifoContext<1024> {
    friend coroutine::LifoStackedPromise<>;

public:
    constexpr explicit Deserializer(DeserializeCallback& callback)
        : callback_(callback)
        , main_task_(process_stream()) {}

    ~Deserializer() { finish_transfer(); }
    Deserializer(const Deserializer&) = delete;
    Deserializer& operator=(const Deserializer&) = delete;
    Deserializer(Deserializer&&) = delete;
    Deserializer& operator=(Deserializer&&) = delete;

    // Feed a new chunk of input bytes to satisfy the current peek_bytes() request.
    // May resume the coroutine immediately if the request can be fully satisfied.
    void feed(std::span<const std::byte> buffer) {
        utility::assert_debug(requested_bytes_);
        utility::assert_debug(pending_bytes_ < requested_bytes_);

        if (buffer.empty() || discard_mode_)
            return;

        input_cursor_ = buffer.data();
        input_end_ = buffer.data() + buffer.size();

        if (!pending_bytes_ && buffer.size() >= requested_bytes_) {
            // Fast path: current input already satisfies the request, no need to copy.
            resume();
            return;
        }

        auto required = static_cast<std::size_t>(requested_bytes_ - pending_bytes_);
        const bool sufficient = buffer.size() >= required;

        const size_t copied = sufficient ? required : buffer.size();
        std::memcpy(pending_bytes_buffer_ + pending_bytes_, input_cursor_, copied);

        pending_bytes_ += copied;
        input_cursor_ += copied;

        if (sufficient)
            resume();
    }

    void finish_transfer() {
        if (awaiting_field_first_byte_) {
            discard_mode_ = false;
            return;
        }

        // Input ended while parsing a field; truncated field.
        // Cancel the outstanding peek by making await_resume() return nullptr.
        discard_mode_ = true;
        resume();
        discard_mode_ = false;

        utility::assert_debug(awaiting_field_first_byte_);
    }

private:
    coroutine::LifoTask<void> process_stream();

    coroutine::LifoTask<bool> process_can_field(FieldId field_id);

    coroutine::LifoTask<bool> process_uart_field(FieldId field_id);

    coroutine::LifoTask<bool> process_gpio_field(FieldId field_id);

    coroutine::LifoTask<bool> process_imu_field(FieldId field_id);

    // Await until at least `size` contiguous bytes are available at the current read position.
    // Returns a pointer to a contiguous region of at least `size` bytes.
    // (from input buffer or pending cache)
    struct PeekBytesAwaiter {
        constexpr explicit PeekBytesAwaiter(Deserializer& owner, size_t size) noexcept
            : owner_(owner)
            , scheduler_awaiter_(owner_.suspend()) {
            utility::assert_debug(size);
            // Request must fit cache and not shrink a previous request.
            utility::assert_debug(size <= sizeof(pending_bytes_buffer_));
            utility::assert_debug(size >= owner.requested_bytes_);
            owner.requested_bytes_ = size;
        }

        constexpr bool await_ready() noexcept {
            utility::assert_debug(
                !owner_.input_cursor_ || owner_.input_end_ >= owner_.input_cursor_);
            const auto remaining = owner_.input_cursor_
                                     ? static_cast<size_t>(owner_.input_end_ - owner_.input_cursor_)
                                     : 0;

            const auto requested_bytes = owner_.requested_bytes_;
            const auto pending_bytes = owner_.pending_bytes_;
            utility::assert_debug(requested_bytes >= pending_bytes);

            // Fast path: satisfy the request directly from the input span (no copy).
            if (!pending_bytes && remaining >= requested_bytes)
                return true;

            const auto total_available = pending_bytes + remaining;

            // Not enough bytes: append everything we have into the pending cache and suspend.
            if (total_available < requested_bytes) {
                if (remaining) {
                    std::memcpy(
                        owner_.pending_bytes_buffer_ + pending_bytes, owner_.input_cursor_,
                        remaining);
                }
                owner_.pending_bytes_ = total_available;
                owner_.input_cursor_ = owner_.input_end_;
                return false;
            }

            // Enough bytes, but split across chunks: top up the pending cache to satisfy peek.
            utility::assert_debug(pending_bytes);
            const auto required = static_cast<std::size_t>(requested_bytes - pending_bytes);
            if (required) {
                utility::assert_debug(owner_.input_cursor_);
                std::memcpy(
                    owner_.pending_bytes_buffer_ + pending_bytes, owner_.input_cursor_, required);
                owner_.input_cursor_ += required;
                owner_.pending_bytes_ = requested_bytes;
            }
            return true;
        }

        constexpr void await_suspend(std::coroutine_handle<> h) noexcept {
            scheduler_awaiter_.await_suspend(h);
        }

        constexpr const std::byte* await_resume() const noexcept {
            utility::assert_debug(owner_.requested_bytes_);
            // Discard mode cancels outstanding peeks by returning nullptr.
            if (owner_.discard_mode_) [[unlikely]] {
                return nullptr;
            } else if (owner_.pending_bytes_) {
                utility::assert_debug(owner_.pending_bytes_ == owner_.requested_bytes_);
                return owner_.pending_bytes_buffer_;
            } else {
                utility::assert_debug(owner_.input_cursor_);
                utility::assert_debug(owner_.input_end_ >= owner_.input_cursor_);
                utility::assert_debug(
                    std::cmp_greater_equal(
                        owner_.input_end_ - owner_.input_cursor_, owner_.requested_bytes_));

                return owner_.input_cursor_;
            }
        }

    protected:
        Deserializer& owner_;

    private:
        coroutine::LifoContext::Awaiter scheduler_awaiter_;
    };

    // IMPORTANT CALL CONTRACT:
    // 1. Within a single parsing path, the pointer returned from any `co_await peek_bytes(...)` may
    //    only be used until the next `co_await peek_bytes(...)` is issued; after a subsequent peek,
    //    all previously returned pointers must be treated as invalid and never dereferenced.
    // 2. Only "expanding the window" is allowed: each subsequent `peek_bytes(new_size)` must
    //    satisfy `new_size >= old_size`.
    // REVIEWERS: Please verify all call sites strictly follow this contract.
    PeekBytesAwaiter peek_bytes(size_t size) { return PeekBytesAwaiter{*this, size}; }

    void consume_peeked() {
        utility::assert_debug(requested_bytes_);
        if (pending_bytes_) {
            utility::assert_debug(pending_bytes_ == requested_bytes_);
            pending_bytes_ = 0;
        } else {
            input_cursor_ += requested_bytes_;
        }
        requested_bytes_ = 0;
    }

    void consume_peeked_partial(size_t n) {
        utility::assert_debug(requested_bytes_);
        utility::assert_debug(n < requested_bytes_);

        if (pending_bytes_) {
            utility::assert_debug(pending_bytes_ == requested_bytes_);
            std::memmove(pending_bytes_buffer_, pending_bytes_buffer_ + n, requested_bytes_ - n);
            pending_bytes_ -= n;
        } else {
            input_cursor_ += n;
        }

        requested_bytes_ -= n;
    }

    void enter_discard_mode() {
        callback_.error_callback();

        // - Treat the current input chunk as exhausted (so the next peek_bytes() suspends)
        // - Ignore subsequent feed() calls while discard_mode_ is true
        // Recovery is driven by finish_transfer(): when suspended waiting for the first header
        // byte (awaiting_field_first_byte_ == true), finish_transfer() clears discard_mode_
        // and returns, allowing parsing to resume on the next transfer.

        pending_bytes_ = requested_bytes_ = 0;
        input_cursor_ = input_end_;
        discard_mode_ = true;
    }

    bool discard_mode_ = false;
    bool awaiting_field_first_byte_ = false;
    DeserializeCallback& callback_;

    size_t pending_bytes_ = 0, requested_bytes_ = 0;
    alignas(std::max_align_t) std::byte pending_bytes_buffer_[kProtocolBufferSize];

    const std::byte *input_cursor_ = nullptr, *input_end_ = nullptr;
    coroutine::LifoTask<void> main_task_;
};

} // namespace librmcs::core::protocol
