#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class UpStairsMode : uint8_t {
    Step_By_One,
    Step_By_Two,
    Auto
};

} // namespace rmcs_msgs