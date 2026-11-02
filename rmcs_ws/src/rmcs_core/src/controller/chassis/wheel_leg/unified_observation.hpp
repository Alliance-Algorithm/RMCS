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
inline constexpr std::string_view kProne200ObservationContractSha256 =
    "6613ab1b2a538fc31558b668117dbcb77bed4f84e93133d9c087c5752ec6c967";
inline constexpr std::string_view kWheelbipeObservationContractSha256 =
    "bd8397a8cb057c5d7859ba79ffbc1310c7c75d2a9ddcfdb7ea844d02db001e69";

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

    double policy_hz() const { return wheelbipe_ ? 50.0 : local_standing_v4_ ? 100.0 : 200.0; }
    bool prone_200hz() const { return local_standing_v2_; }
    bool wheelbipe() const { return wheelbipe_; }

    void use_local_standing_v4() {
        local_standing_v4_ = true;
        reset();
    }

    void use_local_standing_v2() {
        local_standing_v4_ = local_standing_v2_ = true;
        local_standing_.use_v2();
        reset();
    }

    void use_wheelbipe() {
        wheelbipe_ = true;
        reset();
    }

    std::string_view contract_sha256() const {
        if (wheelbipe_)
            return kWheelbipeObservationContractSha256;
        if (local_standing_v2_)
            return kProne200ObservationContractSha256;
        return local_standing_v4_ ? kLocalStandingObservationContractSha256
                                  : kStandingObservationContractSha256;
    }

    std::optional<std::array<double, 35>> sample(
        std::uint64_t policy_tick, const std::array<double, 6>& position,
        const std::array<double, 6>& velocity, const std::array<double, 3>& gyro,
        const std::array<double, 3>& gravity, const std::array<double, 3>& command, double height,
        const std::array<double, 6>& applied_action, bool active) {
        if (wheelbipe_)
            return sample_wheelbipe_(
                position, velocity, gyro, gravity, command, height, applied_action);
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
    static std::optional<std::array<double, 35>> sample_wheelbipe_(
        const std::array<double, 6>& position, const std::array<double, 6>& velocity,
        const std::array<double, 3>& gyro, const std::array<double, 3>& gravity,
        const std::array<double, 3>& command, double height,
        const std::array<double, 6>& previous_action) {
        // V14 clips physical inputs before scaling; raw relative leg positions
        // are deliberately not wrapped. Its policy order is LH,RH,LK,RK,LW,RW.
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
        for (std::size_t i = 0; i < order.size(); ++i) {
            const auto j = order[i];
            if (!std::isfinite(position[j]) || !std::isfinite(velocity[j])
                || !std::isfinite(previous_action[j]))
                return std::nullopt;
            if (i < 4)
                frame[10 + i] = std::clamp(
                    static_cast<float>(position[j])
                        - static_cast<float>(kWheelbipePolicyProfile.nominal[j]),
                    -100.F, 100.F);
            frame[16 + i] = .1F * bounded(velocity[j], 200.F);
            frame[22 + i] = bounded(previous_action[j], 100.F);
        }
        frame[28] = 1.;
        return frame;
    }

    standing_reference::StandingReferenceV3 standing_;
    LocalStandingOdometry local_standing_;
    bool local_standing_v4_;
    bool local_standing_v2_ = false;
    bool wheelbipe_ = false;
};

} // namespace rmcs_core::controller::chassis
