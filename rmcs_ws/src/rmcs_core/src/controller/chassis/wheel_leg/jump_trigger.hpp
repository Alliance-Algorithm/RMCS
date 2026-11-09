#pragma once

namespace rmcs_core::controller::chassis::wheel_leg {

inline bool jump_requested(
    bool was_armed, bool ctrl, bool last_ctrl, bool was_middle, bool left_middle, bool right_up) {
    return was_armed && ((!last_ctrl && ctrl) || (was_middle && left_middle && right_up));
}

} // namespace rmcs_core::controller::chassis::wheel_leg
