#pragma once

#include <array>
#include <chrono>
#include <cstdint>

namespace rmcs_msgs {

enum class TargetSnapshotArmorType : uint8_t {
    BIG = 0,
    SMALL = 1,
};

enum class TargetSnapshotArmorName : uint8_t {
    ONE = 0,
    TWO = 1,
    THREE = 2,
    FOUR = 3,
    FIVE = 4,
    SENTRY = 5,
    OUTPOST = 6,
    BASE = 7,
    NOT_ARMOR = 8,
};

struct TargetSnapshot {
    using Clock = std::chrono::steady_clock;

    static constexpr size_t kStateSize = 11;

    bool valid = false;
    bool converged = false;
    uint8_t armor_count = 0;
    TargetSnapshotArmorType armor_type = TargetSnapshotArmorType::SMALL;
    TargetSnapshotArmorName armor_name = TargetSnapshotArmorName::NOT_ARMOR;
    Clock::time_point timestamp = Clock::time_point::min();
    std::array<double, kStateSize> state{};
};

} // namespace rmcs_msgs
