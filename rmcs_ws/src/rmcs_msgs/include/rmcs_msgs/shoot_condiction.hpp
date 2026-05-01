#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class ShootCondiction : uint8_t {
//状态机
    FRICTION_WAITING = 0,

    PRELOADING = 1,

    SHOOT = 2,

    JAM = 3,

    FIRED = 4,
};
}