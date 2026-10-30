#pragma once

#include "leg_target_winding.hpp"
#include "standing_local_odometry.hpp"
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

inline std::array<double, 6> measured_policy_history(
    const std::array<double, 6>& position, const PolicyProfile& profile = kV6PolicyProfile) {
    std::array<double, 6> result{};
    for (int side = 0; side < 2; ++side) {
        const int hip = 2 * side;
        const float sign = side == 0 ? 1.F : -1.F;
        const float first =
            static_cast<float>(position[hip]) - static_cast<float>(profile.nominal[hip]);
        const float second =
            static_cast<float>(position[hip + 1]) - static_cast<float>(profile.nominal[hip + 1]);
        // paired_recovery.paired_offsets retains its authenticated relative
        // projection chart when the full-vehicle absolute nominal changes.
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
    explicit UnifiedObservation(bool local_standing_v4 = false)
        : standing_(standing_reference::frozen_geometry107())
        , local_standing_v4_(local_standing_v4) {}

    void reset(std::uint64_t policy_tick = 0) {
        standing_.reset(policy_tick);
        local_standing_.reset();
    }

    double policy_hz() const { return local_standing_v4_ ? 100.0 : 200.0; }

    void use_local_standing_v4() {
        local_standing_v4_ = true;
        reset();
    }

    std::string_view contract_sha256() const {
        return local_standing_v4_ ? kLocalStandingObservationContractSha256
                                  : kStandingObservationContractSha256;
    }

    std::optional<std::array<double, 35>> sample(
        std::uint64_t policy_tick, const std::array<double, 6>& position,
        const std::array<double, 6>& velocity, const std::array<double, 3>& gyro,
        const std::array<double, 3>& gravity, const std::array<double, 3>& command, double height,
        const std::array<double, 6>& applied_action, bool active) {
        std::array<float, 35> frame{};
        std::array<float, 6> q{};
        const auto& profile = local_standing_v4_ ? kFullVehicleV202PolicyProfile : kV6PolicyProfile;
        for (std::size_t i = 0; i < 3; ++i) {
            frame[i] = static_cast<float>(command[i]);
            frame[4 + i] = .5F * static_cast<float>(gyro[i]);
            frame[7 + i] = static_cast<float>(gravity[i]);
        }
        frame[3] = 5.F * static_cast<float>(height);
        for (std::size_t i = 0; i < 6; ++i) {
            // V4 subtracts float64 wheel origins in LocalStandingOdometry;
            // never round the cumulative wheel angles into this float frame.
            q[i] = i < 4 || !local_standing_v4_ ? static_cast<float>(position[i]) : 0.F;
            frame[16 + i] = .1F * static_cast<float>(velocity[i]);
            frame[22 + i] = static_cast<float>(applied_action[i]);
            if (i < 4) {
                const float delta = q[i] - static_cast<float>(profile.nominal[i]);
                frame[10 + i] = std::atan2(std::sin(delta), std::cos(delta));
            }
        }
        frame[28] = 1.F;
        const bool finite_inputs =
            std::all_of(
                frame.begin(), frame.end(), [](float value) { return std::isfinite(value); })
            && std::all_of(q.begin(), q.end(), [](float value) { return std::isfinite(value); });
        if (!local_standing_v4_ && !finite_inputs)
            return std::nullopt;
        // Training reconstructs geometry legs from the wrapped actor channels;
        // only the wheel angles retain their continuous encoder turns.
        for (std::size_t i = 0; i < 4; ++i)
            q[i] = frame[10 + i] + static_cast<float>(profile.nominal[i]);
        if (local_standing_v4_) {
            // ChassisEnv builds/clips the float32 sensor frame before odometry.
            for (auto& value : frame)
                value = std::clamp(value, -100.F, 100.F);
            std::array<float, 4> nominal{};
            for (std::size_t i = 0; i < nominal.size(); ++i)
                nominal[i] = static_cast<float>(profile.nominal[i]);
            local_standing_.sample(policy_tick, {position[4], position[5]}, nominal, frame, active);
        } else {
            const auto displacement = standing_.sample(policy_tick, q, frame, active);
            frame[14] = 20.F * displacement[0];
            frame[15] = 20.F * displacement[1];
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
    standing_reference::StandingReferenceV3 standing_;
    LocalStandingOdometry local_standing_;
    bool local_standing_v4_;
};

} // namespace rmcs_core::controller::chassis
