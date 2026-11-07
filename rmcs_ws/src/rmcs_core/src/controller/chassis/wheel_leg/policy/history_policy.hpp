#pragma once

#include "parameters.hpp"

#include <cstdint>
#include <optional>

namespace rmcs_core::controller::chassis {

inline constexpr PolicyParameters kHistoryPolicy{
    "history",
    "gimbal_target_heading_v1",
    "",
    50.,
    .25,
    std::numbers::pi,
    90.,
    160.,
    2.5,
    0.,
    2.7,
    6. * std::numbers::pi,
    kMechanicalBounds,
    {-.42, .13742282595395358, .42, -.1374155762580851, 0., 0.},
    false};

class HistoryPolicy {
public:
    void reset() {}
    std::optional<std::array<double, 35>> sample(
        std::uint64_t, const JointVector& q, const JointVector& dq,
        const std::array<double, 3>& gyro, const std::array<double, 3>& gravity,
        const std::array<double, 3>& command, double height, const JointVector& issued, bool) {
        std::array<double, 35> frame{};
        std::copy(command.begin(), command.end(), frame.begin());
        frame[3] = height * 5.;
        for (int i = 0; i < 3; ++i) {
            frame[4 + i] = gyro[i] * .5;
            frame[7 + i] = gravity[i];
        }
        for (int side = 0; side < 2; ++side) {
            const int hip = 2 * side;
            const double a = static_cast<double>(static_cast<float>(q[hip]))
                           - static_cast<float>(kHistoryPolicy.nominal[hip]);
            const double b = static_cast<double>(static_cast<float>(q[hip + 1]))
                           - static_cast<float>(kHistoryPolicy.nominal[hip + 1]);
            const double turns =
                std::floor((static_cast<double>(a) + b) * .5 / (2. * std::numbers::pi) + .5);
            frame[10 + hip] = static_cast<float>(a - turns * 2. * std::numbers::pi);
            frame[11 + hip] = static_cast<float>(b - turns * 2. * std::numbers::pi);
        }
        for (int i = 0; i < 6; ++i) {
            frame[16 + i] = dq[i] * .1;
            frame[22 + i] = issued[i];
        }
        frame[28] = 1.;
        for (auto& value : frame) {
            if (!std::isfinite(value))
                return std::nullopt;
            value = std::clamp(value, -100., 100.);
        }
        return frame;
    }
};

} // namespace rmcs_core::controller::chassis
