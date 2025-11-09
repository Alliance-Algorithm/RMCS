#pragma once

#include <expected>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <utility>

namespace rmcs::debug {

class StreamContext {
public:
    using FrameRef = cv::Mat const&;

    enum class StreamType : uint8_t {
        NONE,
        RTP_JEPG,
        RTP_H264,
    };
    struct VideoFormat {
        int w;
        int h;
        int hz;
    };
    struct StreamTarget {
        std::string host;
        std::string port;
    };

public:
    explicit StreamContext(StreamType stream_type, const VideoFormat& video_format,
        StreamTarget stream_target) noexcept
        : video_format_ { video_format }
        , stream_type_ { stream_type }
        , stream_target_ { std::move(stream_target) } { }

    static auto check_support() noexcept -> std::expected<void, std::string_view>;

    auto open() noexcept -> std::expected<void, std::string>;

    auto opened() const noexcept -> bool;

    auto session_description_protocol(const std::string_view& local_ip) const noexcept
        -> std::expected<std::string, std::string>;

    auto write(FrameRef frame) const noexcept -> void;

    auto video_format() const noexcept { return video_format_; }

    auto stream_type() const noexcept { return stream_type_; }

    auto stream_target() const noexcept { return stream_target_; }

    auto pipeline() const noexcept { return pipeline_; }

private:
    VideoFormat video_format_;
    StreamType stream_type_;
    StreamTarget stream_target_;

    std::unique_ptr<cv::VideoWriter> sender_;
    std::string pipeline_;
};

}
