#pragma once
#include "stream_context.hpp"
#include <functional>

namespace rmcs::debug {

using StreamTarget = StreamContext::StreamTarget;
using StreamType   = StreamContext::StreamType;
using VideoFormat  = StreamContext::VideoFormat;
using FrameRef     = StreamContext::FrameRef;

class StreamSession {
public:
    struct Config {
        StreamTarget target;
        StreamType type;
        VideoFormat format;
    };

public:
    StreamSession() noexcept;
    ~StreamSession() noexcept;

    StreamSession(const StreamSession&)            = delete;
    StreamSession& operator=(const StreamSession&) = delete;

    StreamSession(StreamSession&&) noexcept            = default;
    StreamSession& operator=(StreamSession&&) noexcept = default;

    auto set_notifier(std::function<void(const std::string&)>) noexcept -> void;

    auto open(const Config&) noexcept -> std::expected<void, std::string>;
    auto opened() const noexcept -> bool;

    auto push_frame(FrameRef) noexcept -> bool;

    auto session_description_protocol() const noexcept -> std::expected<std::string, std::string>;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}
