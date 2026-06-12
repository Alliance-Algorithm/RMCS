#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

#include <eigen3/Eigen/Geometry>

namespace rmcs_msgs {

struct CameraFrameRaw {
    static constexpr std::uint32_t kWidth = 1440;
    static constexpr std::uint32_t kHeight = 1080;
    static constexpr std::size_t kFrameSize =
        static_cast<std::size_t>(kWidth) * static_cast<std::size_t>(kHeight);

    std::array<std::byte, kFrameSize> data;
    int opencv_cvt_color_code;

    Eigen::Quaterniond imu_snapshot;

    std::chrono::steady_clock::time_point exposure_timestamp;
    std::chrono::steady_clock::time_point image_reception_timestamp;
    std::chrono::steady_clock::time_point sync_publish_timestamp;
};

} // namespace rmcs_msgs
