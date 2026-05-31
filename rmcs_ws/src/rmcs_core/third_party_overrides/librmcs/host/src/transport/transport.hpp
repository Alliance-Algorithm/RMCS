#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string_view>

#include "core/src/protocol/constant.hpp"

namespace librmcs::host::transport {

class TransportBuffer {
public:
    using BufferSpanType = std::span<std::byte, core::protocol::kProtocolBufferSize>;

    TransportBuffer() = default;
    TransportBuffer(const TransportBuffer&) = delete;
    TransportBuffer& operator=(const TransportBuffer&) = delete;
    TransportBuffer(TransportBuffer&&) = delete;
    TransportBuffer& operator=(TransportBuffer&&) = delete;
    virtual ~TransportBuffer() noexcept = default;

    virtual BufferSpanType data() const noexcept = 0;
};

class Transport {
public:
    Transport() = default;
    virtual ~Transport() noexcept = default;
    Transport(const Transport&) = delete;
    Transport& operator=(const Transport&) = delete;
    Transport(Transport&&) = delete;
    Transport& operator=(Transport&&) = delete;

    virtual std::unique_ptr<TransportBuffer> acquire_transmit_buffer() noexcept = 0;
    virtual void transmit(std::unique_ptr<TransportBuffer> buffer, size_t payload_size) = 0;
    virtual void release_transmit_buffer(std::unique_ptr<TransportBuffer> buffer) = 0;
    virtual void receive(std::function<void(std::span<const std::byte>)> callback) = 0;
    virtual bool healthy() const noexcept { return true; }
};

namespace usb {

struct ConnectionOptions {
    bool dangerously_skip_version_checks = false;
};

std::unique_ptr<Transport> create_transport(
    uint16_t usb_vid, int32_t usb_pid, std::string_view serial_filter,
    const ConnectionOptions& options);

} // namespace usb

} // namespace librmcs::host::transport
