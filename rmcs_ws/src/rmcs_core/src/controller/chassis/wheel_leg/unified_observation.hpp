#pragma once

#include "leg_target_winding.hpp"
#include "standing_model_geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string_view>

namespace rmcs_core::controller::chassis {

// SHA256 of the sorted, compact standing_position_observation JSON contract.
inline constexpr std::string_view kStandingObservationContractSha256 =
    "db0fc6121c73121dcd39eef0524ecfeab4b3aa69aa7f5700d4205499af2402a9";

inline std::array<double, 6> measured_policy_history(const std::array<double, 6>& position) {
    std::array<double, 6> result{};
    for (int side = 0; side < 2; ++side) {
        const int hip = 2 * side;
        const float sign = side == 0 ? 1.F : -1.F;
        const float first =
            static_cast<float>(position[hip]) - static_cast<float>(kV6PolicyProfile.nominal[hip]);
        const float second = static_cast<float>(position[hip + 1])
                           - static_cast<float>(kV6PolicyProfile.nominal[hip + 1]);
        const float nominal_delta =
            static_cast<float>(kV6PolicyProfile.nominal[hip + 1] - kV6PolicyProfile.nominal[hip]);
        const auto& bounds = kBranch107SignedDeltaBoundsRad[side];
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

class UnifiedObservation {
public:
    UnifiedObservation()
        : standing_(standing_reference::frozen_geometry107()) {}

    void reset(std::uint64_t policy_tick = 0) { standing_.reset(policy_tick); }

    std::optional<std::array<double, 35>> sample(
        std::uint64_t policy_tick, const std::array<double, 6>& position,
        const std::array<double, 6>& velocity, const std::array<double, 3>& gyro,
        const std::array<double, 3>& gravity, const std::array<double, 3>& command, double height,
        const std::array<double, 6>& applied_action, bool active) {
        std::array<float, 35> frame{};
        std::array<float, 6> q{};
        for (std::size_t i = 0; i < 3; ++i) {
            frame[i] = static_cast<float>(command[i]);
            frame[4 + i] = .5F * static_cast<float>(gyro[i]);
            frame[7 + i] = static_cast<float>(gravity[i]);
        }
        frame[3] = 5.F * static_cast<float>(height);
        for (std::size_t i = 0; i < 6; ++i) {
            q[i] = static_cast<float>(position[i]);
            frame[16 + i] = .1F * static_cast<float>(velocity[i]);
            frame[22 + i] = static_cast<float>(applied_action[i]);
            if (i < 4) {
                const float delta = q[i] - static_cast<float>(kV6PolicyProfile.nominal[i]);
                frame[10 + i] = std::atan2(std::sin(delta), std::cos(delta));
            }
        }
        frame[28] = 1.F;
        if (!std::all_of(
                frame.begin(), frame.end(), [](float value) { return std::isfinite(value); })
            || !std::all_of(q.begin(), q.end(), [](float value) { return std::isfinite(value); }))
            return std::nullopt;
        // Training reconstructs geometry legs from the wrapped actor channels;
        // only the wheel angles retain their continuous encoder turns.
        for (std::size_t i = 0; i < 4; ++i)
            q[i] = frame[10 + i] + static_cast<float>(kV6PolicyProfile.nominal[i]);
        const auto displacement = standing_.sample(policy_tick, q, frame, active);
        frame[14] = 20.F * displacement[0];
        frame[15] = 20.F * displacement[1];
        std::array<double, 35> result{};
        for (std::size_t i = 0; i < frame.size(); ++i)
            result[i] = static_cast<double>(std::clamp(frame[i], -100.F, 100.F));
        return result;
    }

private:
    standing_reference::StandingReferenceV3 standing_;
};

} // namespace rmcs_core::controller::chassis
