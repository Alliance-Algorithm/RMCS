#pragma once

#include <bit>
#include <cstdint>

namespace world_exe::enumeration {
enum class ArmorIdFlag : uint32_t {
    Unknow = std::bit_cast<uint32_t>(-1),
    None   = 0b00000000,

    Hero        = 0b00000001, // 1
    Engineer    = 0b00000010, // 2
    InfantryIII = 0b00000100, // 3
    InfantryIV  = 0b00001000, // 4
    InfantryV   = 0b00010000, // 5
    Sentry      = 0b00100000, // 6
    Base        = 0b01000000, // 7
    Outpost     = 0b10000000, // 8

    Count = 8
};

}