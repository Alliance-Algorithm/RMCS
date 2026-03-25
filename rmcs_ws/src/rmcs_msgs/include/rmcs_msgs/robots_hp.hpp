#pragma once
#include <cstdint>

namespace rmcs_msgs {

struct RobotsHp {
    std::uint16_t red_1;
    std::uint16_t red_2;
    std::uint16_t red_3;
    std::uint16_t red_4;
    std::uint16_t red_5;
    std::uint16_t red_7;
    std::uint16_t red_outpost;
    std::uint16_t red_base;
    std::uint16_t blue_1;
    std::uint16_t blue_2;
    std::uint16_t blue_3;
    std::uint16_t blue_4;
    std::uint16_t blue_5;
    std::uint16_t blue_7;
    std::uint16_t blue_outpost;
    std::uint16_t blue_base;
};

} // namespace rmcs_msgs
