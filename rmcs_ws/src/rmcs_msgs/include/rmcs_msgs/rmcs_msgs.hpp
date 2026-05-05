#pragma once

#if defined(__has_include)
# if __has_include(<format>)
#  include <format>
#  if defined(__cpp_lib_format) && (__cpp_lib_format >= 201907L)
#   define RMCS_MSGS_HAS_STD_FORMAT 1
#  endif
# endif
#endif

#include "chassis_mode.hpp"
#include "full_robot_id.hpp"
#include "game_stage.hpp"
#include "gimbal_mode.hpp"
#include "keyboard.hpp"
#include "mouse.hpp"
#include "robot_color.hpp"
#include "robot_id.hpp"
#include "serial_interface.hpp"
#include "shoot_mode.hpp"
#include "shoot_status.hpp"
#include "switch.hpp"

namespace rmcs_msgs {

constexpr auto to_string(GameStage stage) noexcept -> const char* {
    switch (stage) {
    case GameStage::NOT_START: return "NOT_START";
    case GameStage::PREPARATION: return "PREPARATION";
    case GameStage::REFEREE_CHECK: return "REFEREE_CHECK";
    case GameStage::COUNTDOWN: return "COUNTDOWN";
    case GameStage::STARTED: return "STARTED";
    case GameStage::SETTLING: return "SETTLING";
    case GameStage::UNKNOWN: return "UNKNOWN";
    }
    return "INVALID";
}

constexpr auto to_string(ChassisMode mode) noexcept -> const char* {
    switch (mode) {
    case ChassisMode::AUTO: return "AUTO";
    case ChassisMode::SPIN: return "SPIN";
    case ChassisMode::STEP_DOWN: return "STEP_DOWN";
    case ChassisMode::LAUNCH_RAMP: return "LAUNCH_RAMP";
    }
    return "INVALID";
}

constexpr auto to_string(GimbalMode mode) noexcept -> const char* {
    switch (mode) {
    case GimbalMode::IMU: return "IMU";
    case GimbalMode::ENCODER: return "ENCODER";
    }
    return "INVALID";
}

constexpr auto to_string(RobotColor color) noexcept -> const char* {
    switch (color) {
    case RobotColor::UNKNOWN: return "UNKNOWN";
    case RobotColor::RED: return "RED";
    case RobotColor::BLUE: return "BLUE";
    }
    return "INVALID";
}

constexpr auto to_string(ArmorID id) noexcept -> const char* {
    switch (id) {
    case ArmorID::Unknown: return "Unknown";
    case ArmorID::Hero: return "Hero";
    case ArmorID::Engineer: return "Engineer";
    case ArmorID::InfantryIII: return "InfantryIII";
    case ArmorID::InfantryIV: return "InfantryIV";
    case ArmorID::InfantryV: return "InfantryV";
    case ArmorID::Aerial: return "Aerial";
    case ArmorID::Sentry: return "Sentry";
    case ArmorID::Dart: return "Dart";
    case ArmorID::Radar: return "Radar";
    case ArmorID::Outpost: return "Outpost";
    case ArmorID::Base: return "Base";
    }
    return "INVALID";
}

constexpr auto to_string(RobotId::Value id) noexcept -> const char* {
    switch (id) {
    case RobotId::UNKNOWN: return "UNKNOWN";
    case RobotId::RED_HERO: return "RED_HERO";
    case RobotId::RED_ENGINEER: return "RED_ENGINEER";
    case RobotId::RED_INFANTRY_III: return "RED_INFANTRY_III";
    case RobotId::RED_INFANTRY_IV: return "RED_INFANTRY_IV";
    case RobotId::RED_INFANTRY_V: return "RED_INFANTRY_V";
    case RobotId::RED_AERIAL: return "RED_AERIAL";
    case RobotId::RED_SENTRY: return "RED_SENTRY";
    case RobotId::RED_DART: return "RED_DART";
    case RobotId::RED_RADAR: return "RED_RADAR";
    case RobotId::RED_OUTPOST: return "RED_OUTPOST";
    case RobotId::RED_BASE: return "RED_BASE";
    case RobotId::BLUE_HERO: return "BLUE_HERO";
    case RobotId::BLUE_ENGINEER: return "BLUE_ENGINEER";
    case RobotId::BLUE_INFANTRY_III: return "BLUE_INFANTRY_III";
    case RobotId::BLUE_INFANTRY_IV: return "BLUE_INFANTRY_IV";
    case RobotId::BLUE_INFANTRY_V: return "BLUE_INFANTRY_V";
    case RobotId::BLUE_AERIAL: return "BLUE_AERIAL";
    case RobotId::BLUE_SENTRY: return "BLUE_SENTRY";
    case RobotId::BLUE_DART: return "BLUE_DART";
    case RobotId::BLUE_RADAR: return "BLUE_RADAR";
    case RobotId::BLUE_OUTPOST: return "BLUE_OUTPOST";
    case RobotId::BLUE_BASE: return "BLUE_BASE";
    }
    return "INVALID";
}

constexpr auto to_string(RobotId id) noexcept -> const char* {
    return to_string(static_cast<RobotId::Value>(id));
}

constexpr auto to_string(ShootMode mode) noexcept -> const char* {
    switch (mode) {
    case ShootMode::SINGLE: return "SINGLE";
    case ShootMode::AUTOMATIC: return "AUTOMATIC";
    case ShootMode::PRECISE: return "PRECISE";
    case ShootMode::LOW_LATENCY: return "LOW_LATENCY";
    case ShootMode::OVERDRIVE: return "OVERDRIVE";
    }
    return "INVALID";
}

constexpr auto to_string(Switch value) noexcept -> const char* {
    switch (value) {
    case Switch::UNKNOWN: return "UNKNOWN";
    case Switch::UP: return "UP";
    case Switch::DOWN: return "DOWN";
    case Switch::MIDDLE: return "MIDDLE";
    }
    return "INVALID";
}

constexpr auto to_string(FullRobotId::Value id) noexcept -> const char* {
    switch (id) {
    case FullRobotId::UNKNOWN: return "UNKNOWN";
    case FullRobotId::RED_HERO: return "RED_HERO";
    case FullRobotId::RED_ENGINEER: return "RED_ENGINEER";
    case FullRobotId::RED_INFANTRY_III: return "RED_INFANTRY_III";
    case FullRobotId::RED_INFANTRY_IV: return "RED_INFANTRY_IV";
    case FullRobotId::RED_INFANTRY_V: return "RED_INFANTRY_V";
    case FullRobotId::RED_AERIAL: return "RED_AERIAL";
    case FullRobotId::RED_SENTRY: return "RED_SENTRY";
    case FullRobotId::RED_DART: return "RED_DART";
    case FullRobotId::RED_RADAR: return "RED_RADAR";
    case FullRobotId::RED_OUTPOST: return "RED_OUTPOST";
    case FullRobotId::RED_BASE: return "RED_BASE";
    case FullRobotId::BLUE_HERO: return "BLUE_HERO";
    case FullRobotId::BLUE_ENGINEER: return "BLUE_ENGINEER";
    case FullRobotId::BLUE_INFANTRY_III: return "BLUE_INFANTRY_III";
    case FullRobotId::BLUE_INFANTRY_IV: return "BLUE_INFANTRY_IV";
    case FullRobotId::BLUE_INFANTRY_V: return "BLUE_INFANTRY_V";
    case FullRobotId::BLUE_AERIAL: return "BLUE_AERIAL";
    case FullRobotId::BLUE_SENTRY: return "BLUE_SENTRY";
    case FullRobotId::BLUE_DART: return "BLUE_DART";
    case FullRobotId::BLUE_RADAR: return "BLUE_RADAR";
    case FullRobotId::BLUE_OUTPOST: return "BLUE_OUTPOST";
    case FullRobotId::BLUE_BASE: return "BLUE_BASE";
    case FullRobotId::RED_HERO_CLIENT: return "RED_HERO_CLIENT";
    case FullRobotId::RED_ENGINEER_CLIENT: return "RED_ENGINEER_CLIENT";
    case FullRobotId::RED_INFANTRY_III_CLIENT: return "RED_INFANTRY_III_CLIENT";
    case FullRobotId::RED_INFANTRY_IV_CLIENT: return "RED_INFANTRY_IV_CLIENT";
    case FullRobotId::RED_INFANTRY_V_CLIENT: return "RED_INFANTRY_V_CLIENT";
    case FullRobotId::RED_AERIAL_CLIENT: return "RED_AERIAL_CLIENT";
    case FullRobotId::BLUE_HERO_CLIENT: return "BLUE_HERO_CLIENT";
    case FullRobotId::BLUE_ENGINEER_CLIENT: return "BLUE_ENGINEER_CLIENT";
    case FullRobotId::BLUE_INFANTRY_III_CLIENT: return "BLUE_INFANTRY_III_CLIENT";
    case FullRobotId::BLUE_INFANTRY_IV_CLIENT: return "BLUE_INFANTRY_IV_CLIENT";
    case FullRobotId::BLUE_INFANTRY_V_CLIENT: return "BLUE_INFANTRY_V_CLIENT";
    case FullRobotId::BLUE_AERIAL_CLIENT: return "BLUE_AERIAL_CLIENT";
    case FullRobotId::REFEREE_SERVER: return "REFEREE_SERVER";
    }
    return "INVALID";
}

constexpr auto to_string(FullRobotId id) noexcept -> const char* {
    return to_string(static_cast<FullRobotId::Value>(id));
}

} // namespace rmcs_msgs

#if defined(RMCS_MSGS_HAS_STD_FORMAT)
template <>
struct std::formatter<rmcs_msgs::GameStage> : std::formatter<const char*> {
    auto format(rmcs_msgs::GameStage value, std::format_context& ctx) const {
        return std::formatter<const char*>::format(rmcs_msgs::to_string(value), ctx);
    }
};

template <>
struct std::formatter<rmcs_msgs::ChassisMode> : std::formatter<const char*> {
    auto format(rmcs_msgs::ChassisMode value, std::format_context& ctx) const {
        return std::formatter<const char*>::format(rmcs_msgs::to_string(value), ctx);
    }
};

template <>
struct std::formatter<rmcs_msgs::GimbalMode> : std::formatter<const char*> {
    auto format(rmcs_msgs::GimbalMode value, std::format_context& ctx) const {
        return std::formatter<const char*>::format(rmcs_msgs::to_string(value), ctx);
    }
};

template <>
struct std::formatter<rmcs_msgs::RobotColor> : std::formatter<const char*> {
    auto format(rmcs_msgs::RobotColor value, std::format_context& ctx) const {
        return std::formatter<const char*>::format(rmcs_msgs::to_string(value), ctx);
    }
};

template <>
struct std::formatter<rmcs_msgs::ArmorID> : std::formatter<const char*> {
    auto format(rmcs_msgs::ArmorID value, std::format_context& ctx) const {
        return std::formatter<const char*>::format(rmcs_msgs::to_string(value), ctx);
    }
};

template <>
struct std::formatter<rmcs_msgs::RobotId::Value> : std::formatter<const char*> {
    auto format(rmcs_msgs::RobotId::Value value, std::format_context& ctx) const {
        return std::formatter<const char*>::format(rmcs_msgs::to_string(value), ctx);
    }
};

template <>
struct std::formatter<rmcs_msgs::RobotId> : std::formatter<const char*> {
    auto format(rmcs_msgs::RobotId value, std::format_context& ctx) const {
        return std::formatter<const char*>::format(rmcs_msgs::to_string(value), ctx);
    }
};

template <>
struct std::formatter<rmcs_msgs::ShootMode> : std::formatter<const char*> {
    auto format(rmcs_msgs::ShootMode value, std::format_context& ctx) const {
        return std::formatter<const char*>::format(rmcs_msgs::to_string(value), ctx);
    }
};

template <>
struct std::formatter<rmcs_msgs::Switch> : std::formatter<const char*> {
    auto format(rmcs_msgs::Switch value, std::format_context& ctx) const {
        return std::formatter<const char*>::format(rmcs_msgs::to_string(value), ctx);
    }
};

template <>
struct std::formatter<rmcs_msgs::FullRobotId::Value> : std::formatter<const char*> {
    auto format(rmcs_msgs::FullRobotId::Value value, std::format_context& ctx) const {
        return std::formatter<const char*>::format(rmcs_msgs::to_string(value), ctx);
    }
};

template <>
struct std::formatter<rmcs_msgs::FullRobotId> : std::formatter<const char*> {
    auto format(rmcs_msgs::FullRobotId value, std::format_context& ctx) const {
        return std::formatter<const char*>::format(rmcs_msgs::to_string(value), ctx);
    }
};
#endif
