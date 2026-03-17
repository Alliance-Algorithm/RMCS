#pragma once

namespace rmcs_msgs {

enum class DartLimitingServoStatus {
    FREE = 0,
    LOCK = 1,
    WAIT = 2,
};

} // namespace rmcs_msgs
