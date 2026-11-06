#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

#include <eigen3/Eigen/Geometry>

namespace rmcs_msgs {

struct RgbdFrame {
    static constexpr std::uint32_t kWidth = 1280;
    static constexpr std::uint32_t kHeight = 720;
    static constexpr std::size_t kPixelCount =
        static_cast<std::size_t>(kWidth) * static_cast<std::size_t>(kHeight);

    struct Intrinsics {
        float fx, fy;
        float cx, cy;
        std::array<float, 5> coeffs;
    };

    // BGR8
    std::array<std::byte, 3 * kPixelCount> color;
    // Z16, aligned to color
    std::array<std::uint16_t, kPixelCount> depth;
    // depth * depth_scale = meters
    float depth_scale;
    Intrinsics color_intrinsics;
    std::uint64_t frame_id;

    Eigen::Quaterniond imu_snapshot;
    Eigen::Vector3d gyro_body = Eigen::Vector3d::Zero();

    std::chrono::steady_clock::time_point exposure_timestamp;
    std::chrono::steady_clock::time_point image_reception_timestamp;
    std::chrono::steady_clock::time_point sync_publish_timestamp;
};

} // namespace rmcs_msgs
