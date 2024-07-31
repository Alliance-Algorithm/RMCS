#pragma once

#include <cstdint>

#include "robot_color.hpp"

namespace rmcs_msgs {
enum class ArmorID : uint16_t {
    Unknown     = 0,
    Hero        = 1,
    Engineer    = 2,
    InfantryIII = 3,
    InfantryIV  = 4,
    InfantryV   = 5,
    Aerial      = 6,
    Sentry      = 7,
    Dart        = 8,
    Radar       = 9,
    Outpost     = 10,
    Base        = 11,
};

class RobotId {
public:
    enum Value : uint8_t {
        UNKNOWN = 0,

        RED_HERO         = 1,
        RED_ENGINEER     = 2,
        RED_INFANTRY_III = 3,
        RED_INFANTRY_IV  = 4,
        RED_INFANTRY_V   = 5,
        RED_AERIAL       = 6,
        RED_SENTRY       = 7,
        RED_DART         = 8,
        RED_RADAR        = 9,
        RED_OUTPOST      = 10,
        RED_BASE         = 11,

        BLUE_HERO         = 101,
        BLUE_ENGINEER     = 102,
        BLUE_INFANTRY_III = 103,
        BLUE_INFANTRY_IV  = 104,
        BLUE_INFANTRY_V   = 105,
        BLUE_AERIAL       = 106,
        BLUE_SENTRY       = 107,
        BLUE_DART         = 108,
        BLUE_RADAR        = 109,
        BLUE_OUTPOST      = 110,
        BLUE_BASE         = 111,
    };

    constexpr RobotId()
        : value_{UNKNOWN} {}
    constexpr explicit RobotId(uint8_t value)
        : value_{static_cast<Value>(value)} {}
    constexpr RobotId(Value value)                      // NOLINT(google-explicit-constructor)
        : value_{value} {}

    explicit constexpr operator uint8_t() const { return value_; }
    constexpr operator Value() const { return value_; } // NOLINT(google-explicit-constructor)

    constexpr RobotId& operator=(Value value) {
        value_ = value;
        return *this;
    }
    constexpr bool operator==(const Value value) const { return value_ == value; }
    constexpr bool operator!=(const Value value) const { return value_ != value; }

    constexpr RobotColor color() const { return value_ & 0x40 ? RobotColor::BLUE : RobotColor::RED; }

    constexpr ArmorID id() const {
        return value_ > 100 ? static_cast<ArmorID>(value_ - 100) : static_cast<ArmorID>(value_);
    }

private:
    Value value_;
};

} // namespace rmcs_msgs