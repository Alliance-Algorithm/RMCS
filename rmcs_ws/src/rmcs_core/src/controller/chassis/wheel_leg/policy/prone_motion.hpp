#pragma once
#include "local_odometry.hpp"
#include "parameters.hpp"
#include <optional>
namespace rmcs_core::controller::chassis {
inline constexpr LegBounds kProneWindingBounds{
    {{-.14062820701680934, 1.3692123767724917}, {-.1406283965709381, 1.3692125038958158}}};
inline constexpr PolicyParameters kProneMotion{
    "prone_motion",
    "wheel-leg-prone-motion-v4-odom-v2",
    "6613ab1b2a538fc31558b668117dbcb77bed4f84e93133d9c087c5752ec6c967",
    100.,
    .25,
    4.,
    90.,
    160.,
    2.5,
    1.,
    2.,
    10.471975511965978,
    kProneWindingBounds};
inline std::array<double, 6> measured_policy_history(const std::array<double, 6>& position) {
    constexpr JointVector frozen_nominal{-.42, .13742282595395358, .42, -.1374155762580851, 0., 0.};
    std::array<double, 6> result{};
    for (int side = 0; side < 2; ++side) {
        const int hip = 2 * side;
        const float sign = side == 0 ? 1.F : -1.F;
        const float first =
            static_cast<float>(position[hip]) - static_cast<float>(kNominalPosition[hip]);
        const float second =
            static_cast<float>(position[hip + 1]) - static_cast<float>(kNominalPosition[hip + 1]);
        const float nominal_delta =
            static_cast<float>(frozen_nominal[hip + 1] - frozen_nominal[hip]);
        const auto& bounds = kProneWindingBounds[side];
        const float delta = std::clamp(
            sign * (nominal_delta + second - first), static_cast<float>(bounds[0]),
            static_cast<float>(bounds[1]));
        const float offset_delta = sign * delta - nominal_delta;
        const float mean = .5F * (first + second);
        const float common = std::atan2(std::sin(mean), std::cos(mean));
        result[hip] = static_cast<double>((common - .5F * offset_delta) / .25F);
        result[hip + 1] = static_cast<double>((common + .5F * offset_delta) / .25F);
    }
    return result;
}

class ProneMotion {
public:
    void reset() { odometry_.reset(); }
    std::optional<WheelLegFeatures> sample(
        std::uint64_t policy_tick, const JointVector& position, const JointVector& velocity,
        const std::array<double, 3>& gyro, const std::array<double, 3>& gravity,
        const std::array<double, 3>& command, double height, const JointVector& applied_action,
        bool active) {
        StandingReferenceFeatures features;
        std::array<float, 6> q{};

        for (std::size_t i = 0; i < 3; ++i) {
            features.command[i] = static_cast<float>(command[i]);
            features.gyro[i] = .5F * static_cast<float>(gyro[i]);
            features.gravity[i] = static_cast<float>(gravity[i]);
        }
        features.height = 5.F * static_cast<float>(height);
        for (std::size_t i = 0; i < 6; ++i) {
            q[i] = i < 4 ? static_cast<float>(position[i]) : 0.F;
            features.velocity[i] = .1F * static_cast<float>(velocity[i]);
            features.previous_action[i] = static_cast<float>(applied_action[i]);
            if (i < 4) {
                const float delta = q[i] - static_cast<float>(kNominalPosition[i]);
                features.position[i] = std::atan2(std::sin(delta), std::cos(delta));
            }
        }
        bool finite_inputs =
            std::isfinite(features.height)
            && std::all_of(q.begin(), q.end(), [](float value) { return std::isfinite(value); });
        for (auto* values : {&features.command, &features.gyro, &features.gravity})
            for (auto& value : *values) {
                finite_inputs &= std::isfinite(value);
                value = std::clamp(value, -100.F, 100.F);
            }
        for (auto* values : {&features.position, &features.velocity, &features.previous_action})
            for (auto& value : *values) {
                finite_inputs &= std::isfinite(value);
                value = std::clamp(value, -100.F, 100.F);
            }
        features.height = std::clamp(features.height, -100.F, 100.F);
        {
            std::array<float, 4> nominal{};
            for (std::size_t i = 0; i < nominal.size(); ++i)
                nominal[i] = static_cast<float>(kNominalPosition[i]);
            odometry_.sample(policy_tick, {position[4], position[5]}, nominal, features, active);
        }
        if (!finite_inputs
            || !std::all_of(features.position.begin(), features.position.end(), [](float value) {
                   return std::isfinite(value);
               }))
            return std::nullopt;
        WheelLegFeatures physical;
        physical.command = command;
        physical.height = height;
        physical.gyro = gyro;
        physical.gravity = gravity;
        physical.velocity = velocity;
        physical.previous_action = applied_action;
        for (std::size_t i = 0; i < 6; ++i)
            physical.position[i] = features.position[i];
        for (std::size_t i = 0; i < 7; ++i)
            physical.auxiliary[i] = features.auxiliary[i];
        return physical;
    }

private:
    LocalStandingOdometry odometry_;
};
} // namespace rmcs_core::controller::chassis
