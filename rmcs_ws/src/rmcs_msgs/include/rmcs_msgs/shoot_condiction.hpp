#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class ShootCondiction : uint8_t {
    FRICTION_WAITING = 0,
    PRELOADING = 1,
    SHOOT = 2,
    JAM = 3,
    FIRED = 4,
};
}
