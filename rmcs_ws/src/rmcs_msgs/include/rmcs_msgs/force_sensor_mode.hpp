#pragma once

#include <cstdint>

namespace rmcs_msgs {

enum class ForceSensorCommand : uint8_t {
    UNKNOWN = 0,
    PEEL = 1,
    ZERO_CALIBRATION = 2,
    WEIGHT_READ = 3,
    COMMUNICATION_SETTING = 4,
    MEASUREMENT_SETTING = 5,
    SENSOR_CALIBRATION = 6
};

} // namespace rmcs_msgs