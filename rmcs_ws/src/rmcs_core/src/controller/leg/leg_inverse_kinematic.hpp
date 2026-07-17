#pragma once

#include <array>
#include <cmath>
#include <numbers>

namespace rmcs_core::controller::leg {

[[nodiscard]] inline std::array<double, 2> leg_inverse_kinematic(
    double f_x, double b_x, bool is_front_ecd_obtuse, bool is_back_ecd_obtuse) {
    constexpr double link1      = 240.0;
    constexpr double link2      = 120.0;
    constexpr double link3      = 160.0;
    constexpr double link_angle = 5.0 * std::numbers::pi / 6.0;

    const double theta_b = !is_back_ecd_obtuse ? std::asin(b_x / link1)
                                                : std::numbers::pi - std::asin(b_x / link1);
    const double x_link2 = link2 * std::sin(link_angle - theta_b);
    const double theta_f =
        !is_front_ecd_obtuse ? std::asin((f_x - x_link2) / link3)
                             : std::numbers::pi - std::asin((f_x - x_link2) / link3);

    return {theta_f, theta_b};
}

} // namespace rmcs_core::controller::leg
