#pragma once
#include <cmath>
namespace rmcs_utility {

static double normalize_angle(double angle) {
    angle = std::fmod(angle + M_PI, 2 * M_PI);
    return angle < 0 ? angle + M_PI : angle - M_PI;
}

} // namespace rmcs_utility