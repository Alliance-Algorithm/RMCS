#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class Switch : uint8_t { UNKNOWN = 0, UP = 1, DOWN = 2, MIDDLE = 3 };

} // namespace rmcs_msgs