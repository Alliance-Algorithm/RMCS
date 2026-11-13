#pragma once
#include "features.hpp"
#include "parameters.hpp"
#include <cstdint>
#include <optional>
namespace rmcs_core::controller::chassis {
inline constexpr PolicyParameters kWheelbipe{
    "wheelbipe",
    "wheelbipe-v202-foundation-v2",
    "bd8397a8cb057c5d7859ba79ffbc1310c7c75d2a9ddcfdb7ea844d02db001e69",
    50.,
    .5,
    1e100,
    1e100,
    60.,
    2.,
    0.,
    2.7,
    10.471975511965978,
    {{{-.1408932677945942, 1.4282761689185115}, {-.14084732125908495, 1.4283102010168576}}}};
class Wheelbipe {
public:
    void reset() {}
    std::optional<WheelLegFeatures> sample(
        std::uint64_t, const JointVector& position, const JointVector& velocity,
        const std::array<double, 3>& gyro, const std::array<double, 3>& gravity,
        const std::array<double, 3>& command, double height, const JointVector& previous_action,
        bool) {
        WheelLegFeatures features;
        for (std::size_t i = 0; i < 3; ++i) {
            if (!std::isfinite(command[i]) || !std::isfinite(gyro[i]) || !std::isfinite(gravity[i]))
                return std::nullopt;
            features.command[i] = command[i];
            features.gyro[i] = gyro[i];
            features.gravity[i] = gravity[i];
        }
        if (!std::isfinite(height))
            return std::nullopt;
        features.height = height;
        const auto nominal = paired_targets(
            {kNominalPosition[0], kNominalPosition[1], kNominalPosition[2], kNominalPosition[3]},
            {position[0], position[1], position[2], position[3]}, kWheelbipe.winding_bounds);
        for (std::size_t i = 0; i < position.size(); ++i) {
            const auto j = i;
            if (!std::isfinite(position[j]) || !std::isfinite(velocity[j])
                || !std::isfinite(previous_action[j]))
                return std::nullopt;
            if (i < 4)
                features.position[i] = position[j] - nominal[j];
            features.velocity[i] = velocity[j];
            features.previous_action[i] = previous_action[j];
        }
        return features;
    }
};
} // namespace rmcs_core::controller::chassis
