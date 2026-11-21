#pragma once

namespace rmcs_msgs {

// Integer values published on /wheel_leg/rl/state and used by telemetry and replay.
enum class WheelLegRlState : int { kIdle = 1, kWaiting = 2, kRunning = 3, kRecovering = 4 };

} // namespace rmcs_msgs
