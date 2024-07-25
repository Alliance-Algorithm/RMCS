#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class ControlMode : uint8_t { CLOSE = 0, MANUAL = 1, AUTO = 2 };

} // namespace rmcs_msgs