#pragma once

#include <cmath>
#include <limits>
#include <numbers>

namespace rmcs_utility {

[[nodiscard]] inline double normalize_angle(double angle) noexcept {
    if (!std::isfinite(angle)) {
        return NAN;
    }

    constexpr double pi = std::numbers::pi_v<double>;
    angle               = std::fmod(angle + pi, 2.0 * pi);
    return angle < 0.0 ? angle + pi : angle - pi;
}

} // namespace rmcs_utility
