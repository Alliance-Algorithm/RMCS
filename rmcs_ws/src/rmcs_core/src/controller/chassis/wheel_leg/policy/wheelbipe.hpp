#pragma once
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
    2. * std::numbers::pi,
    {{{-.1408932677945942, 1.4282761689185115}, {-.14084732125908495, 1.4283102010168576}}}};
class Wheelbipe {
public:
    void reset() {}
    std::optional<std::array<double, 35>> sample(
        std::uint64_t, const JointVector& position, const JointVector& velocity,
        const std::array<double, 3>& gyro, const std::array<double, 3>& gravity,
        const std::array<double, 3>& command, double height, const JointVector& previous_action,
        bool) {
        // Policy order: LH,RH,LK,RK,LW,RW.
        constexpr std::array<std::size_t, 6> order{0, 2, 1, 3, 4, 5};
        const auto bounded = [](double value, float limit) {
            return std::clamp(static_cast<float>(value), -limit, limit);
        };
        std::array<double, 35> frame{};
        for (std::size_t i = 0; i < 3; ++i) {
            if (!std::isfinite(command[i]) || !std::isfinite(gyro[i]) || !std::isfinite(gravity[i]))
                return std::nullopt;
            frame[i] = bounded(command[i], 100.F);
            frame[4 + i] = .5F * bounded(gyro[i], 100.F);
            frame[7 + i] = bounded(gravity[i], 100.F);
        }
        if (!std::isfinite(height))
            return std::nullopt;
        frame[3] = 5.F * std::clamp(static_cast<float>(height), 0.F, 1.F);
        const auto nominal = paired_targets(
            {kNominalPosition[0], kNominalPosition[1], kNominalPosition[2], kNominalPosition[3]},
            {position[0], position[1], position[2], position[3]}, kWheelbipe.winding_bounds);
        for (std::size_t i = 0; i < order.size(); ++i) {
            const auto j = order[i];
            if (!std::isfinite(position[j]) || !std::isfinite(velocity[j])
                || !std::isfinite(previous_action[j]))
                return std::nullopt;
            if (i < 4)
                frame[10 + i] = std::clamp(
                    static_cast<float>(position[j] - nominal[j]),
                    -100.F, 100.F);
            frame[16 + i] = .1F * bounded(velocity[j], 200.F);
            frame[22 + i] = bounded(previous_action[j], 100.F);
        }
        frame[28] = 1.;
        return frame;
    }
};
} // namespace rmcs_core::controller::chassis
