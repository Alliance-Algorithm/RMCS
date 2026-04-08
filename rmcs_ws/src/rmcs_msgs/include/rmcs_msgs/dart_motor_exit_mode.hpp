#pragma once

namespace rmcs_msgs {

enum class ExitMode {
    WAIT_ZERO_VELOCITY, // 闭环零速等待，适合小负载
    WAIT_HOLD_TORQUE,   // 退出后保持 WAIT + hold torque，直到下一条 belt 命令覆盖
};

}
