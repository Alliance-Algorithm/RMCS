#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class RelayMode : uint8_t {
    Open,
    Close
};

} // namespace rmcs_msgs