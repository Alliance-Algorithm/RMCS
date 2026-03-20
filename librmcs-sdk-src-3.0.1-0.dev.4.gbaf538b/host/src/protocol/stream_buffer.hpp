#pragma once

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <memory>
#include <span>
#include <utility>

#include "core/src/protocol/constant.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "host/src/transport/transport.hpp"

namespace librmcs::host::protocol {

/**
 * @brief A buffered stream writer for efficient sequential data transmission.
 *
 * StreamBuffer manages a buffer pool from a Transport and provides a
 * streaming interface for writing data. It automatically handles buffer
 * allocation, switching, and transmission, allowing users to focus on
 * writing data without managing buffer lifecycle.
 *
 * @section Usage Pattern
 * 1. Allocate memory via allocate() or allocate_up_to()
 * 2. Write data to the returned span
 * 3. Repeat steps 1-2 as needed
 * 4. Destruction automatically transmits any pending data
 *
 * @section Buffer Management
 * - Buffers are acquired lazily on first allocation
 * - When current buffer is full, it's automatically transmitted and replaced
 * - Destruction commits any pending buffer with written data
 * - All buffer lifecycle is managed internally - no manual cleanup needed
 *
 * @section Constraints
 * - Single allocation size must not exceed kProtocolBufferSize
 * - Move-only
 * - Not thread-safe - external synchronization required for concurrent access
 *
 * @section Example
 * @code
 *   StreamBuffer writer(transport);
 *
 *   // Allocate exactly 32 bytes
 *   auto span1 = writer.allocate(32);
 *   if (!span1.empty()) {
 *       // Write data to span1...
 *   }
 *
 *   // Allocate between 16-128 bytes (gets as much as possible)
 *   auto span2 = writer.allocate_up_to(16, 128);
 *   if (!span2.empty()) {
 *       // Write data to span2...
 *   }
 *
 *   // Destructor automatically transmits all written data
 * @endcode
 */
class StreamBuffer : public core::protocol::SerializeBuffer {
public:
    /**
     * @brief Constructs a StreamBuffer bound to the specified transport.
     *
     * @param transport The transport to use for buffer acquisition and transmission.
     *                  Must outlive this StreamBuffer instance.
     */
    explicit StreamBuffer(transport::Transport& transport) noexcept
        : transport_(transport) {}

    StreamBuffer(StreamBuffer&& other) noexcept
        : transport_(other.transport_)
        , buffer_(std::move(other.buffer_))
        , current_(other.current_)
        , end_(other.end_) {
        other.current_ = nullptr;
        other.end_ = nullptr;
    }
    StreamBuffer& operator=(StreamBuffer&&) = delete;
    StreamBuffer(const StreamBuffer&) = delete;
    StreamBuffer& operator=(const StreamBuffer&) = delete;

    /**
     * @brief Destructor. Automatically transmits any pending buffered data.
     *
     * If a buffer contains written data, it will be transmitted before
     * destruction completes. This ensures no data loss on scope exit.
     */
    ~StreamBuffer() override {
        if (buffer_)
            finalize_buffer();
    }

    /**
     * @brief Allocates a contiguous memory region of exactly the specified size.
     *
     * Returns a writable span that remains valid until the next call to
     * allocate(), allocate_up_to(), or destruction of this StreamBuffer.
     *
     * If the current buffer has insufficient space, it will be automatically
     * transmitted and a new buffer will be acquired.
     *
     * @param size Number of bytes to allocate.
     *             Must be in range (0, kProtocolBufferSize].
     *
     * @return A span of exactly 'size' bytes on success, or an empty span
     *         if buffer acquisition fails (e.g., resource exhaustion)
     *
     * @pre size must be greater than 0
     * @pre size must not exceed kProtocolBufferSize
     *
     * @par Example
     * @code
     *   auto span = writer.allocate(64);
     *   if (!span.empty()) {
     *       std::memcpy(span.data(), my_data, 64);
     *   }
     * @endcode
     */
    std::span<std::byte> allocate(std::size_t size) noexcept override {
        core::utility::assert_debug(
            0 < size && size <= core::protocol::kProtocolBufferSize && current_ <= end_);

        if (!buffer_) {
            if (!init_buffer())
                return {};
        } else if (std::cmp_less(end_ - current_, size)) {
            finalize_buffer();
            if (!init_buffer())
                return {};
        }

        auto* begin = current_;
        current_ += size;
        return {begin, size};
    }

    /**
     * @brief Allocates a memory region with flexible size between min and max.
     *
     * Attempts to allocate as much memory as possible within the specified
     * range. This is useful when you can work with variable amounts of data
     * (e.g., bulk operations where more is better but a minimum is required).
     *
     * Returns the largest available span up to max_size, as long as it's at
     * least min_size. The actual size depends on remaining buffer space.
     *
     * If the current buffer has less than min_size available, it will be
     * automatically transmitted and a new buffer will be acquired.
     *
     * @param min_size Minimum acceptable allocation size.
     *                 Must be in range (0, kProtocolBufferSize].
     * @param max_size Maximum desired allocation size. Must be >= min_size.
     *
     * @return A span of size in range [min_size, max_size] on success, or an
     *         empty span if buffer acquisition fails (e.g., resource exhaustion)
     *
     * @pre min_size must be greater than 0
     * @pre min_size must not exceed kProtocolBufferSize
     * @pre max_size must be >= min_size
     *
     * @par Example
     * @code
     *   // Try to copy up to 256 bytes, but need at least 64
     *   auto span = writer.allocate_up_to(64, 256);
     *   if (!span.empty()) {
     *       size_t copied = copy_data(span.data(), span.size());
     *       // span.size() might be 64, 128, 256, or anywhere in between
     *   }
     * @endcode
     */
    std::span<std::byte> allocate_up_to(std::size_t min_size, std::size_t max_size) noexcept {
        core::utility::assert_debug(
            0 < min_size && min_size <= core::protocol::kProtocolBufferSize && min_size <= max_size
            && current_ <= end_);

        if (!buffer_) {
            if (!init_buffer())
                return {};
        } else if (std::cmp_less(end_ - current_, min_size)) {
            finalize_buffer();
            if (!init_buffer())
                return {};
        }

        auto* begin = current_;
        auto size = std::min(static_cast<std::size_t>(end_ - current_), max_size);
        current_ += size;
        return {begin, size};
    }

private:
    bool init_buffer() noexcept {
        core::utility::assert_debug(!buffer_ && !current_ && !end_);

        buffer_ = transport_.acquire_transmit_buffer();
        if (!buffer_)
            return false;

        auto data = buffer_->data();
        current_ = data.data();
        end_ = current_ + data.size();
        core::utility::assert_debug(data.size() == core::protocol::kProtocolBufferSize);

        return true;
    }

    void finalize_buffer() noexcept {
        core::utility::assert_debug(buffer_ && current_ && end_ && current_ <= end_);

        const std::byte* begin = buffer_->data().data();
        const std::size_t payload_size = current_ - begin;
        core::utility::assert_debug(payload_size > 0);

        transport_.transmit(std::move(buffer_), payload_size);
        buffer_.reset();
        current_ = end_ = nullptr;
    }

    transport::Transport& transport_;

    std::unique_ptr<transport::TransportBuffer> buffer_ = nullptr;
    std::byte *current_ = nullptr, *end_ = nullptr;
};

} // namespace librmcs::host::protocol
