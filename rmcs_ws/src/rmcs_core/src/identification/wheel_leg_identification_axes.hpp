#pragma once

#include <array>

namespace rmcs_core::controller::identification {

// Shared registration order for controller and recorder ports. The first four
// entries are the paired leg drives, followed by the left and right wheel.
inline constexpr std::array kAxisNames{"left_hip_joint",   "left_knee_joint", "right_hip_joint",
                                       "right_knee_joint", "left_wheel",      "right_wheel"};

} // namespace rmcs_core::controller::identification
