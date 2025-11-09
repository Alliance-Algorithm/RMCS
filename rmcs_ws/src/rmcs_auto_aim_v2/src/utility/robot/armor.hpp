#pragma once
#include "utility/robot/id.hpp"
#include <opencv2/core/types.hpp>

namespace rmcs {

enum class ArmorColor : std::uint8_t { DARK, RED, BLUE, MIX };
constexpr auto get_enum_name(ArmorColor color) noexcept {
    constexpr std::array details { "DARK", "RED", "BLUE", "MIX" };
    return details[std::to_underlying(color)];
}

enum class ArmorShape : bool { LARGE, SMALL };
constexpr auto get_enum_name(ArmorShape shape) noexcept {
    constexpr std::array details { "LARGE", "SMALL" };
    return details[std::to_underlying(shape)];
};

using ArmorType = DeviceId;
constexpr auto get_enum_name(ArmorType type) noexcept { return rmcs::to_string(type); }

struct LightStrip {
    std::uint16_t unique_id;

    ArmorColor color;

    cv::Point2f p1;
    cv::Point2f p2;
};

struct Armor { };
using Armors = std::vector<Armor>;

}
