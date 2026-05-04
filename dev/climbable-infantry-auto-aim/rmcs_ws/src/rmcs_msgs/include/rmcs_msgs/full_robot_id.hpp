#pragma once

#include <cstdint>

#include "robot_color.hpp"
#include "robot_id.hpp"

namespace rmcs_msgs {

class FullRobotId {
public:
    enum Value : uint16_t {
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

        RED_HERO_CLIENT         = 0x0101,
        RED_ENGINEER_CLIENT     = 0x0102,
        RED_INFANTRY_III_CLIENT = 0x0103,
        RED_INFANTRY_IV_CLIENT  = 0x0104,
        RED_INFANTRY_V_CLIENT   = 0x0105,
        RED_AERIAL_CLIENT       = 0x0106,

        BLUE_HERO_CLIENT         = 0x0165,
        BLUE_ENGINEER_CLIENT     = 0x0166,
        BLUE_INFANTRY_III_CLIENT = 0x0167,
        BLUE_INFANTRY_IV_CLIENT  = 0x0168,
        BLUE_INFANTRY_V_CLIENT   = 0x0169,
        BLUE_AERIAL_CLIENT       = 0x016A,

        REFEREE_SERVER = 0x8080,
    };

    constexpr FullRobotId()
        : value_{UNKNOWN} {}
    constexpr explicit FullRobotId(uint16_t value)
        : value_{static_cast<Value>(value)} {}
    constexpr FullRobotId(Value value)                  // NOLINT(google-explicit-constructor)
        : value_{value} {}
    constexpr FullRobotId(RobotId::Value value)         // NOLINT(google-explicit-constructor)
        : value_{static_cast<Value>(static_cast<uint16_t>(value))} {}

    explicit constexpr operator uint16_t() const { return value_; }
    constexpr operator Value() const { return value_; } // NOLINT(google-explicit-constructor)

    constexpr FullRobotId& operator=(Value value) {
        value_ = value;
        return *this;
    }
    constexpr bool operator==(const Value value) const { return value_ == value; }
    constexpr bool operator!=(const Value value) const { return value_ != value; }

    constexpr RobotColor color() const {
        if (value_ == REFEREE_SERVER) [[unlikely]]
            return RobotColor::UNKNOWN;

        return value_ & 0x40 ? RobotColor::BLUE : RobotColor::RED;
    }

    constexpr FullRobotId robot() const {
        if (value_ == REFEREE_SERVER) [[unlikely]]
            return FullRobotId::UNKNOWN;

        return static_cast<Value>(value_ & 0x0100);
    }

    constexpr FullRobotId client() const {
        if (value_ == REFEREE_SERVER) [[unlikely]]
            return FullRobotId::UNKNOWN;

        return static_cast<Value>(value_ | 0x0100);
    }

private:
    Value value_;
};

} // namespace rmcs_msgs