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
        // paired_recovery.paired_offsets retains its authenticated relative
        // projection chart when the full-vehicle absolute nominal changes.
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
    std::optional<std::array<double, 35>> sample(
        std::uint64_t policy_tick, const JointVector& position, const JointVector& velocity,
        const std::array<double, 3>& gyro, const std::array<double, 3>& gravity,
        const std::array<double, 3>& command, double height, const JointVector& applied_action,
        bool active) {
        std::array<float, 35> frame{};
        std::array<float, 6> q{};

        for (std::size_t i = 0; i < 3; ++i) {
            frame[i] = static_cast<float>(command[i]);
            frame[4 + i] = .5F * static_cast<float>(gyro[i]);
            frame[7 + i] = static_cast<float>(gravity[i]);
        }
        frame[3] = 5.F * static_cast<float>(height);
        for (std::size_t i = 0; i < 6; ++i) {
            // V4 subtracts float64 wheel origins in LocalStandingOdometry;
            // never round the cumulative wheel angles into this float frame.
            q[i] = i < 4 ? static_cast<float>(position[i]) : 0.F;
            frame[16 + i] = .1F * static_cast<float>(velocity[i]);
            frame[22 + i] = static_cast<float>(applied_action[i]);
            if (i < 4) {
                const float delta = q[i] - static_cast<float>(kNominalPosition[i]);
                frame[10 + i] = std::atan2(std::sin(delta), std::cos(delta));
            }
        }
        frame[28] = 1.F;
        const bool finite_inputs =
            std::all_of(
                frame.begin(), frame.end(), [](float value) { return std::isfinite(value); })
            && std::all_of(q.begin(), q.end(), [](float value) { return std::isfinite(value); });
        // Training reconstructs geometry legs from the wrapped actor channels;
        // only the wheel angles retain their continuous encoder turns.
        for (std::size_t i = 0; i < 4; ++i)
            q[i] = frame[10 + i] + static_cast<float>(kNominalPosition[i]);
        {
            // ChassisEnv builds/clips the float32 sensor frame before odometry.
            for (auto& value : frame)
                value = std::clamp(value, -100.F, 100.F);
            std::array<float, 4> nominal{};
            for (std::size_t i = 0; i < nominal.size(); ++i)
                nominal[i] = static_cast<float>(kNominalPosition[i]);
            odometry_.sample(policy_tick, {position[4], position[5]}, nominal, frame, active);
        }
        if (!finite_inputs || !std::all_of(frame.begin(), frame.end(), [](float value) {
                return std::isfinite(value);
            }))
            return std::nullopt;
        std::array<double, 35> result{};
        for (std::size_t i = 0; i < frame.size(); ++i)
            result[i] = static_cast<double>(std::clamp(frame[i], -100.F, 100.F));
        return result;
    }

private:
    LocalStandingOdometry odometry_;
};
} // namespace rmcs_core::controller::chassis
