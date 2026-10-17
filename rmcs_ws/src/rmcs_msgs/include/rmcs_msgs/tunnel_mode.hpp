#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class TunnelMode : std::uint8_t {
    UP,
    DOWN,
    PLAIN,
};

} // namespace rmcs_msgs
