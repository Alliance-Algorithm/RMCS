#pragma once

#include <utility>

namespace rmcs_core::controller::identification {

// Keep the existing integer values on executor ports and in recorded samples.
enum class IdentificationPhase : int {
    kFailed = -1,
    kIdle = 0,
    kPreparing = 1,
    kRunning = 2,
    kComplete = 3,
};

} // namespace rmcs_core::controller::identification
