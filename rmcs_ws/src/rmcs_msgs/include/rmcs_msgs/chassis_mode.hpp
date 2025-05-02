#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class ChassisMode : uint8_t {
    SPIN,
    Four_Wheel_Normal_Move,        
    Six_Wheel_Normal_Move,    
    Yaw_Free,
    None,
    Up_Stairs
};

} // namespace rmcs_msgs