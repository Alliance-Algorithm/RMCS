#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>

namespace rmcs_core::controller::chassis::standing_reference {

using Vector3 = std::array<float, 3>;

struct GeometryKnot {
    float delta_rad;
    Vector3 center_m;
    float passive_inner_deg;
};

struct SideGeometry {
    Vector3 hip_origin_m;
    Vector3 hip_axis;
    float hip_reference_rad;
    float wheel_radius_m;
    std::vector<GeometryKnot> knots;
};

using GeometryTable = std::array<SideGeometry, 2>;

inline float norm(const Vector3& value) {
    return std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
}

inline bool finite(const Vector3& value) {
    return std::all_of(value.begin(), value.end(), [](float v) { return std::isfinite(v); });
}

struct CoordinateResult {
    std::array<float, 2> coordinate_m{};
    bool valid = false;
};

inline CoordinateResult wheel_angle_coordinate(
    const GeometryTable& table, const std::array<float, 6>& position, const Vector3& gravity) {
    const float magnitude = norm(gravity);
    if (!finite(gravity) || !(magnitude > .5F && magnitude < 1.5F)
        || !std::all_of(position.begin(), position.end(), [](float v) { return std::isfinite(v); })
        || !(gravity[2] / magnitude < -.94F))
        return {};
    const float pitch = std::atan2(gravity[0] / magnitude, -gravity[2] / magnitude);
    CoordinateResult result{{}, true};
    for (std::size_t side = 0; side < 2; ++side) {
        const auto& geometry = table[side];
        const std::size_t i = 2 * side;
        const float difference = position[i + 1] - position[i];
        const float delta = std::atan2(std::sin(difference), std::cos(difference));
        if (!(delta >= geometry.knots.front().delta_rad
              && delta <= geometry.knots.back().delta_rad))
            return {};
        const auto upper_iterator = std::upper_bound(
            geometry.knots.begin(), geometry.knots.end(), delta,
            [](float value, const GeometryKnot& knot) { return value < knot.delta_rad; });
        const auto upper_index = std::clamp<std::size_t>(
            upper_iterator - geometry.knots.begin(), 1, geometry.knots.size() - 1);
        const auto& lower = geometry.knots[upper_index - 1];
        const auto& upper = geometry.knots[upper_index];
        const float blend =
            std::clamp((delta - lower.delta_rad) / (upper.delta_rad - lower.delta_rad), 0.F, 1.F);
        Vector3 value{};
        for (std::size_t j = 0; j < 3; ++j)
            value[j] = lower.center_m[j] + blend * (upper.center_m[j] - lower.center_m[j])
                     - geometry.hip_origin_m[j];
        const auto& axis = geometry.hip_axis;
        const Vector3 cross{
            axis[1] * value[2] - axis[2] * value[1], axis[2] * value[0] - axis[0] * value[2],
            axis[0] * value[1] - axis[1] * value[0]};
        const float dot = axis[0] * value[0] + axis[1] * value[1] + axis[2] * value[2];
        const float cosine = std::cos(position[i] - geometry.hip_reference_rad);
        const float sine = std::sin(position[i] - geometry.hip_reference_rad);
        Vector3 center{};
        for (std::size_t j = 0; j < 3; ++j)
            center[j] = cosine * value[j] + sine * cross[j] + (1.F - cosine) * axis[j] * dot
                      + geometry.hip_origin_m[j];
        const float center_x = std::cos(pitch) * center[0] + std::sin(pitch) * center[2];
        constexpr float degrees_to_radians = .017453292519943295F;
        const float lower_beta = lower.passive_inner_deg * degrees_to_radians;
        const float upper_beta = upper.passive_inner_deg * degrees_to_radians;
        const float beta = lower_beta + blend * (upper_beta - lower_beta);
        const float spin = position[i] + (side == 0 ? beta : -beta) + position[4 + side];
        const float sign = axis[1] > 0.F ? 1.F : axis[1] < 0.F ? -1.F : 0.F;
        result.coordinate_m[side] = geometry.wheel_radius_m * (sign * spin + pitch) - center_x;
        if (!std::isfinite(result.coordinate_m[side]))
            return {};
    }
    return result;
}

} // namespace rmcs_core::controller::chassis::standing_reference
