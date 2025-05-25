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
    Auto_Storage_LB,
    Auto_Storage_RB,
    Auto_Extract,
    Auto_Up_Stairs,
    Customer,
    DT7_Control_Position,
    DT7_Control_Orientation,
    None
};

} // namespace rmcs_msgs