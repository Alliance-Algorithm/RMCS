#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class DartStationOpeningStatus : uint8_t {
    OPENED = 0,
    CLOSED = 1,
    RUNNING = 2,
    UNKNOWN = 3,

};

} // namespace rmcs_msgs