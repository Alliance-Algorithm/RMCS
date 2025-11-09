#include "stream_context.hpp"
#include <opencv2/videoio/registry.hpp>

using namespace rmcs::debug;

namespace pipeline {
constexpr auto make_rtpjepg = [](int w, int h, int hz, std::string_view host,
                                  std::string_view port) {
    constexpr auto configuration = std::string_view {
        "appsrc "
        "! videoconvert "
        "! video/x-raw,format=YUY2,width={},height={},framerate={}/1 "
        "! jpegenc "
        "! rtpjpegpay "
        "! udpsink host={} port={}",
    };
    return std::format(configuration, w, h, hz, host, port);
};
constexpr auto make_rtph264 = [](int w, int h, int hz, std::string_view host,
                                  std::string_view port) {
    constexpr auto configuration = std::string_view {
        "appsrc "
        "! videoconvert "
        "! video/x-raw,format=I420,width={},height={},framerate={}/1 "
        "! x264enc tune=zerolatency bitrate=600 speed-preset=ultrafast "
        "! rtph264pay config-interval=1 pt=96 "
        "! udpsink host={} port={}",
    };
    return std::format(configuration, w, h, hz, host, port);
};
}
namespace sdp {
constexpr auto make_rtpjepg = [](std::string_view ip, std::string_view port) {
    constexpr auto configuration = std::string_view {
        "v=0\n"
        "m=video {} RTP/AVP 26\n"
        "c=IN IP4 {}\n"
        "a=rtpmap:26 JPEG/90000\n",
    };
    return std::format(configuration, port, ip);
};
constexpr auto make_rtph264 = [](std::string_view ip, std::string_view port) {
    constexpr auto configuration = std::string_view {
        "v=0\n"
        "m=video {} RTP/AVP 96\n"
        "c=IN IP4 {}\n"
        "a=rtpmap:96 H264/90000\n"
        "a=fmtp:96 packetization-mode=1;"
        "profile-level-id=42e01f;"
        "sprop-parameter-sets=Z0LgC5ZUCg+QgA==,aM4BqA==\n",
    };
    return std::format(configuration, port, ip);
};
}

auto StreamContext::check_support() noexcept -> std::expected<void, std::string_view> {
    if (cv::videoio_registry::getBackendName(cv::CAP_GSTREAMER).empty()) {
        return std::unexpected {
            "[ERROR] GStreamer backend not found in OpenCV. "
            "Reinstall OpenCV with GStreamer support.\n",
        };
    }
    return {};
}

auto StreamContext::open() noexcept -> std::expected<void, std::string> {
    switch (stream_type_) {
    case StreamType::RTP_JEPG:
        pipeline_ = pipeline::make_rtpjepg(video_format_.w, video_format_.h, video_format_.hz,
            stream_target_.host, stream_target_.port);
        break;
    case StreamType::RTP_H264:
        pipeline_ = pipeline::make_rtph264(video_format_.w, video_format_.h, video_format_.hz,
            stream_target_.host, stream_target_.port);
        break;
    case StreamType::NONE:
        return std::unexpected { "Unexpected stream type" };
    }

    sender_ = std::make_unique<cv::VideoWriter>(pipeline_, cv::CAP_GSTREAMER, 0, video_format_.hz,
        cv::Size { video_format_.w, video_format_.h }, true);

    if (!sender_->isOpened()) {
        sender_.reset();
        return std::unexpected {
            "\nUnable to open pipeline."
            "\nPlease install required packages or check your pipeline config"
            "\n  sudo apt install "
            "gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good"
            "\n  current pipeline:\n"
                + pipeline_,
        };
    }

    return {};
}

auto StreamContext::opened() const noexcept -> bool { return sender_ && sender_->isOpened(); }

auto StreamContext::session_description_protocol(const std::string_view& local_ip) const noexcept
    -> std::expected<std::string, std::string> {

    if (!opened()) return std::unexpected { "Pipeline is not opened" };

    switch (stream_type_) {
    case StreamType::RTP_JEPG:
        return sdp::make_rtpjepg(local_ip, stream_target_.port);
    case StreamType::RTP_H264:
        return sdp::make_rtph264(local_ip, stream_target_.port);
    case StreamType::NONE:
        return std::unexpected { "Unexpected stream type" };
    }

    return std::unexpected { "unreachable" };
}

auto StreamContext::write(FrameRef frame) const noexcept -> void {
    if (opened()) sender_->write(frame);
}
