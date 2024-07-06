#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class RobotColor : uint8_t {
    UNKNOWN = 0,
    RED     = 1,
    BLUE    = 2,
};

} // namespace rmcs_msgs