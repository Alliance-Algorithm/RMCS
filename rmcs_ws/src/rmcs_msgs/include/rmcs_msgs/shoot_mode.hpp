#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class ShootMode : uint8_t {
    SINGLE    = 0, // Fires one projectile each time the mouse is clicked.
    PRECISE   = 1, // Fires one projectile each time the mouse is clicked, with improved accuracy at
                   // the cost of increased firing delay.
    AUTOMATIC = 2, // Continuously fires projectiles while the mouse is held down.
    OVERDRIVE = 3, // Continuously fires projectiles while the mouse is held down, ignoring heat
                   // limits but causing health loss.
};

}