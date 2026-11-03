#pragma once
namespace rmcs_core::controller::chassis::wheel_leg::mechanics {
inline constexpr double kThighLength = .21, kShankLength = .25045, kWheelRadius = .06;
inline constexpr double kBodyMass = 11.62, kThighMass = .426, kShankMass = .507, kWheelMass = .40;
inline constexpr double kThighComOffset = .06623, kShankComOffset = .15344;
inline constexpr double kThighComAngle = -.096, kShankComAngle = .002;
inline constexpr double kBodyComForward = .00918, kBodyComUp = -.052;
} // namespace rmcs_core::controller::chassis::wheel_leg::mechanics
