#pragma once

#include <Eigen/Eigen>

#include <cmath>
#include <ctime>
#include <tuple>

namespace world_exe::v1::fire_control::trajectory_solver {
/// returns:
///      fly time in nano second
///      dir(normalized) in control spacing
static std::tuple<const std::time_t, const Eigen::Vector3d> gravity_only(
    const Eigen::Vector3d& target, const double& v, const double& g) {
    double yaw                = atan2(target.y(), target.x());
    Eigen::Vector2d target_2d = { sqrt(target.x() * target.x() + target.y() * target.y()),
        target.z() };
    ;

    double pitch = 0;

    auto a   = v * v;         // v0 ^ 2
    auto b   = a * a;         // v0 ^ 4
    double c = target_2d.x(); // xt ^ 2
    double d = c * c;         // xt ^ 4
    double e = g * g;         // g ^ 2

    double xt = sqrt(c); // target horizontal distance

    double f = b * d * (b - e * c - 2 * g * a * target_2d.y());
    if (f >= 0) {
        pitch = -atan((b * c - sqrt(f)) / (g * a * c * xt));
    }
    auto fly_time = xt / (cos(pitch) * v);

    return { static_cast<time_t>(fly_time * 1e9),
        { cos(yaw) * cos(pitch), sin(yaw) * cos(pitch), sin(pitch) } };
}
}