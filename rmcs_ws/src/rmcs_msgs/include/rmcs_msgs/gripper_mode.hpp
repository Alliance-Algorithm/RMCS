#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class GripperMode : uint8_t {
    Open,
    Close,
    Custom,
    None
};

} // namespace rmcs_msgs