#include "hikrobot.hpp"

#include <stdexcept>

#include "tools/logger.hpp"

namespace io
{
HikRobot::HikRobot(double exposure_ms, double gain, const std::string & vid_pid)
{
    hikcamera::Config config;
    config.exposure_us = static_cast<float>(exposure_ms * 1e3);
    config.gain = static_cast<float>(gain);
    config.framerate = 165.0F;
    camera_.configure(config);

    if (!vid_pid.empty()) {
        tools::logger()->info(
            "[HikRobot] vid_pid filter \"{}\" is ignored; using hikcamera device discovery.", vid_pid);
    }

    if (auto result = camera_.connect(); !result) {
        throw std::runtime_error("Failed to connect hikcamera: " + result.error());
    }
}

HikRobot::~HikRobot()
{
    if (!camera_.connected()) {
        return;
    }

    if (auto result = camera_.disconnect(); !result) {
        tools::logger()->warn("[HikRobot] Failed to disconnect hikcamera: {}", result.error());
    }
}

void HikRobot::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
    auto result = camera_.read_image_with_timestamp();
    if (!result) {
        throw std::runtime_error("Failed to read hikcamera image: " + result.error());
    }

    img = result->mat;
    timestamp = result->timestamp;
}

}  // namespace io
