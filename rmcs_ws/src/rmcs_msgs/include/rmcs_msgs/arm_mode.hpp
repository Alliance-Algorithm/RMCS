#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class ArmMode : uint8_t {
    Auto_Gold_Left,
    Auto_Gold_Right,
    Auto_Gold_Mid,
    Auto_Sliver,
    Auto_Walk,
    Auto_Ground,
    Auto_Storage,
    Auto_Extract,
    Up_Stairs,
    Customer,
    Vision_Exchange,
    DT7_Control_Position,
    DT7_Control_Orientation,
    None
};

} // namespace rmcs_msgs