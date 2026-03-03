#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class LegMode : uint8_t {
    Four_Wheel,        
    Six_Wheel,    
    None,
    Up_One_Stairs,
    Up_Two_Stairs,
    Down_Stairs
};

} // namespace rmcs_msgs