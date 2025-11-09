#pragma once
#include "utility/pimpl.hpp"
#include <chrono>
#include <opencv2/core/mat.hpp>

namespace rmcs::module {

class VideoCapturer {
    RMCS_PIMPL_DEFINITION(VideoCapturer)
public:
    /// @throw std::runtime_error Read error, timeout, e.g
    explicit VideoCapturer(const std::string& path);

    /// @throw std::runtime_error Read error, timeout, e.g
    auto read(std::chrono::milliseconds timeout) const -> cv::Mat;

    auto set_framerate(double hz) noexcept;
};

} // namespace rmcs::capturer
