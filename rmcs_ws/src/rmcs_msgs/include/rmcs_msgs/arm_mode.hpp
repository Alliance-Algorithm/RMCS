#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class ArmMode : uint8_t {
    Auto_Walk,
    Auto_Storage_LB,
    Auto_Storage_RB,
    Auto_Extract,
    Auto_Up_One_Stairs,
    Auto_Up_Two_Stairs,
    Auto_Down_Stairs,
    Custome,
    DT7_Control_Position,
    DT7_Control_Orientation,
    None
};

} // namespace rmcs_msgs