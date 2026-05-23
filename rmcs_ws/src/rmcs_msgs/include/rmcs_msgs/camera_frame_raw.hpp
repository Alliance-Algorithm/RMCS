#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include <eigen3/Eigen/Geometry>

#include "board_clock.hpp"

namespace rmcs_msgs {

struct CameraFrameRaw {
    static constexpr std::uint32_t kWidth = 1440;
    static constexpr std::uint32_t kHeight = 1080;
    static constexpr std::size_t kFrameSize =
        static_cast<std::size_t>(kWidth) * static_cast<std::size_t>(kHeight);

    std::array<std::byte, kFrameSize> data{};
    int opencv_cvt_color_code = 0;

    Eigen::Quaterniond imu_snapshot = Eigen::Quaterniond::Identity();

    BoardClock::time_point timestamp{};
};

} // namespace rmcs_msgs
