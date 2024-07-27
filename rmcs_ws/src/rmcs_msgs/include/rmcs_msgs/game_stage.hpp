#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class GameStage : uint8_t {
    NOT_START     = 0,
    PREPARATION   = 1,
    REFEREE_CHECK = 2,
    COUNTDOWN     = 3,
    STARTED       = 4,
    SETTLING      = 5,
    UNKNOWN       = UINT8_MAX,
};

} // namespace rmcs_msgs