#pragma once

#include <chrono>

namespace rmcs_msgs {

struct HardSyncSnapshot {
    using Clock = std::chrono::steady_clock;

    bool valid = false;
    Clock::time_point exposure_timestamp{};
    double qw = 1.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
};

} // namespace rmcs_msgs
