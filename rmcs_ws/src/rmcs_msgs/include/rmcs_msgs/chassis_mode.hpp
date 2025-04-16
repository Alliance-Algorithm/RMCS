#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class ChassisMode : uint8_t {
    SPIN,
    Four_Wheel_Normal_Move,        
    Six_Wheel_Normal_Move,   
    Flowing, 
    Yaw_Free,
};

} // namespace rmcs_msgs