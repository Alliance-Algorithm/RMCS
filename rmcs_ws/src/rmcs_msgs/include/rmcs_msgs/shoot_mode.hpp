#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class ShootMode : uint8_t {
    // Fires one projectile each time the mouse is clicked.
    SINGLE = 0,

    // Continuously fires projectiles while the mouse is held down.
    AUTOMATIC = 1,

    // Fires one projectile each time the mouse is clicked, with improved accuracy at the cost of
    // increased firing delay.
    PRECISE = 2,

    // Fires one projectile each time the mouse is clicked, with reduced input delay for faster
    // response, may cause unintended emissions.
    LOW_LATENCY = 3,

    // Continuously fires projectiles while the mouse is held down, ignoring heat limits but causing
    // health loss.
    OVERDRIVE = 4,
};

}