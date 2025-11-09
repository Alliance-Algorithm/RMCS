#include "video.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

using namespace rmcs::module;

struct VideoCapturer::Impl final {
    explicit Impl() noexcept = default;

    explicit Impl(const std::string& path) { open(path); }

    auto set_framerate(double hz) noexcept {
        const auto milliseconds_count = static_cast<long>(1'000 / hz);
        interval_duration_            = std::chrono::milliseconds { milliseconds_count };
    }

    auto open(const std::string& path) -> void {
        capturer_ = std::make_unique<cv::VideoCapture>(path);
        if (!capturer_->isOpened())
            throw std::runtime_error { "VideoCapturer: Can not open video" };
    }

    auto read(std::chrono::milliseconds) const -> cv::Mat {
        auto mat = cv::Mat {};
        if (!capturer_->read(mat))
            throw std::runtime_error { "VideoCapturer: Some errors happened while reading video" };

        std::this_thread::sleep_until(last_read_timestamp_ + interval_duration_);

        last_read_timestamp_ = clock::now();
        return mat;
    }

    std::unique_ptr<cv::VideoCapture> capturer_;

    using clock = std::chrono::steady_clock;
    mutable clock::time_point last_read_timestamp_ { clock::now() };

    std::chrono::milliseconds interval_duration_ { 0 };
};

VideoCapturer::VideoCapturer() noexcept
    : pimpl { std::make_unique<Impl>() } { }

VideoCapturer::VideoCapturer(const std::string& path)
    : pimpl { std::make_unique<Impl>(path) } { }

VideoCapturer::~VideoCapturer() noexcept = default;

auto VideoCapturer::read(std::chrono::milliseconds timeout) const -> cv::Mat {
    return pimpl->read(timeout);
}

auto VideoCapturer::set_framerate(double hz) noexcept { pimpl->set_framerate(hz); }
