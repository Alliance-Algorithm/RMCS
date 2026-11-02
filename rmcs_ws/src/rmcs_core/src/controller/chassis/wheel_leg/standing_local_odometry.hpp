#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string_view>

#include "full_vehicle_v202.hpp"
#include "standing_friction_geometry.hpp"
#include "standing_local_geometry.hpp"

namespace rmcs_core::controller::chassis {

// SHA256 of the sorted, compact standing_odometry contract in the training plan.
inline constexpr std::string_view kLocalStandingObservationContractSha256 =
    full_vehicle_v202::kObservationContractSha256;

struct LocalStandingState {
    bool source_active = false;
    bool wheel_fault = false;
    bool reference_active = false;
    bool valid = false;
    std::array<double, 2> source_origin{}, source_last{};
    std::array<float, 2> source_speed{}, anchor_m{}, filtered_m{}, displacement_m{};
    float capture_age_s = 0.F;
    std::optional<std::uint64_t> source_tick, last_tick;
};

// Observation only. Continuous wheel-output radians arrive from the driver's
// high-rate unwrap, after hardware ratio/sign conversion, still in float64.
class LocalStandingOdometry {
public:
    void use_v2(bool enabled) {
        v2_ = enabled;
        reset();
    }
    void reset() { state_ = {}; }

    void sample(
        std::uint64_t tick, const std::array<double, 2>& wheels,
        const std::array<float, 4>& nominal, std::array<float, 35>& frame, bool enabled) {
        if (state_.last_tick && tick < *state_.last_tick)
            throw std::invalid_argument("Policy clock moved backwards; reset local odometry");
        const bool requested = enabled && std::isfinite(frame[0]) && std::isfinite(frame[1])
                            && std::isfinite(frame[2]) && std::abs(frame[0]) < .001F
                            && std::abs(frame[1]) < .001F && std::abs(frame[2]) < .001F;
        const auto local = local_wheel_angles_(tick, wheels, frame, requested);
        std::array<float, 6> q{};
        std::array<float, 6> velocity{};
        for (std::size_t i = 0; i < 6; ++i) {
            q[i] = i < 4 ? frame[10 + i] + nominal[i] : local[i - 4];
            velocity[i] = 10.F * frame[16 + i];
        }
        const standing_reference::Vector3 gravity{frame[7], frame[8], frame[9]};
        const standing_reference::Vector3 gyro{2.F * frame[4], 2.F * frame[5], 2.F * frame[6]};
        const float magnitude = standing_reference::norm(gravity);
        const bool imu_valid = standing_reference::finite(gravity)
                            && standing_reference::finite(gyro) && magnitude > .5F
                            && magnitude < 1.5F;
        const float gz = gravity[2] / std::max(magnitude, 1e-9F);
        const auto& table = v2_ ? standing_reference::friction_geometry110()
                                : standing_reference::local_geometry107();
        auto projected = q;
        bool projection_valid = true;
        bool standing = imu_valid && gz < static_cast<float>(-.9659258262890683);
        for (std::size_t side = 0; side < 2; ++side) {
            const auto& geometry = table[side];
            const std::size_t hip = side * 2;
            const float difference = q[hip + 1] - q[hip];
            const float delta = std::atan2(std::sin(difference), std::cos(difference));
            const float lower = geometry.knots.front().delta_rad;
            const float upper = geometry.knots.back().delta_rad;
            const float bounded = std::clamp(delta, lower, upper);
            const float overshoot = std::abs(delta - bounded);
            projection_valid &=
                std::isfinite(q[hip]) && std::isfinite(q[hip + 1]) && overshoot <= .012F;
            if (overshoot > 0.F) {
                const float margin = 16.F * std::numeric_limits<float>::epsilon()
                                   * (1.F + std::max(std::abs(q[hip]), std::abs(q[hip + 1])));
                const float interior = std::clamp(delta, lower + margin, upper - margin);
                const float error = delta - interior;
                projected[hip] += .5F * error;
                projected[hip + 1] -= .5F * error;
            }
            // EncoderLegGeometry uses the original sensor legs, a 1 mrad
            // validity margin, and the clamped physical table for support height.
            const auto center = wheel_center_(geometry, q[hip], bounded);
            standing &= std::isfinite(delta) && delta >= lower - .001F && delta <= upper + .001F
                     && -center[2] + .06F > .20F;
        }
        const auto coordinate =
            standing_reference::wheel_angle_coordinate(table, projected, gravity);
        const bool valid = standing && projection_valid && coordinate.valid && !state_.wheel_fault
                        && std::all_of(velocity.begin(), velocity.end(), [](float value) {
                               return std::isfinite(value);
                           });
        const bool active_request = requested && state_.source_active;
        const bool eligible =
            active_request && valid
            && (v2_
                || (standing_reference::norm(gyro) < .5F
                    && std::all_of(
                        velocity.begin(), velocity.begin() + 4,
                        [](float value) { return std::abs(value) < 3.F; })
                    && std::abs(velocity[4]) < 2.5F && std::abs(velocity[5]) < 2.5F));
        const bool fresh = !state_.last_tick || *state_.last_tick != tick;
        const bool reset_reference = !active_request || (imu_valid && gz > -.5F);
        if (reset_reference) {
            state_.reference_active = false;
            state_.capture_age_s = 0.F;
            state_.filtered_m = {};
        }
        if (fresh && !state_.reference_active && !reset_reference) {
            state_.capture_age_s = eligible ? state_.capture_age_s + .01F : 0.F;
            if (eligible && state_.capture_age_s + 1e-7F >= (v2_ ? .05F : .2F)) {
                state_.anchor_m = coordinate.coordinate_m;
                state_.reference_active = true;
                state_.filtered_m = {};
            }
        }
        state_.valid = active_request && valid && state_.reference_active && !reset_reference;
        if (state_.valid && fresh) {
            constexpr float alpha = static_cast<float>(.01 / (.05 + .01));
            for (std::size_t side = 0; side < 2; ++side) {
                const float limit = v2_ ? .2F : 1.F;
                const float delta = std::clamp(
                    coordinate.coordinate_m[side] - state_.anchor_m[side], -limit, limit);
                state_.filtered_m[side] += alpha * (delta - state_.filtered_m[side]);
            }
        }
        state_.displacement_m = state_.valid ? state_.filtered_m : std::array<float, 2>{};
        state_.last_tick = tick;
        frame[14] = (v2_ ? 5.F : 20.F) * state_.displacement_m[0];
        frame[15] = (v2_ ? 5.F : 20.F) * state_.displacement_m[1];
        frame[31] = state_.valid ? 1.F : 0.F;
    }

    const LocalStandingState& state() const { return state_; }

private:
    bool v2_ = false;
    std::array<float, 2> local_wheel_angles_(
        std::uint64_t tick, const std::array<double, 2>& wheel, const std::array<float, 35>& frame,
        bool requested) {
        const std::array<float, 2> speed{10.F * frame[20], 10.F * frame[21]};
        const bool finite = std::isfinite(wheel[0]) && std::isfinite(wheel[1])
                         && std::isfinite(speed[0]) && std::isfinite(speed[1]);
        const bool start = requested && !state_.source_active && finite;
        if (start)
            state_.source_origin = wheel;
        if (!requested)
            state_.wheel_fault = false;
        const bool fresh = requested && state_.source_active
                        && (!state_.source_tick || *state_.source_tick != tick);
        if (requested && !finite)
            state_.wheel_fault = true;
        if (fresh && state_.source_tick) {
            const float seconds =
                static_cast<float>(std::max<std::uint64_t>(tick - *state_.source_tick, 1)) * .01F;
            for (std::size_t side = 0; side < 2; ++side) {
                const float bound =
                    .02F
                    + 3.F * seconds
                          * std::max(std::abs(speed[side]), std::abs(state_.source_speed[side]));
                state_.wheel_fault |= std::abs(wheel[side] - state_.source_last[side]) > bound;
            }
        }
        state_.source_active = requested && (state_.source_active || start);
        if (requested && finite && (!state_.source_tick || *state_.source_tick != tick)) {
            state_.source_last = wheel;
            state_.source_speed = speed;
            state_.source_tick = tick;
        }
        std::array<float, 2> result{};
        if (state_.source_active && !state_.wheel_fault && finite)
            for (std::size_t side = 0; side < 2; ++side)
                result[side] = static_cast<float>(wheel[side] - state_.source_origin[side]);
        return result;
    }

    static standing_reference::Vector3
        wheel_center_(const standing_reference::SideGeometry& geometry, float hip, float delta) {
        const auto upper_iterator = std::upper_bound(
            geometry.knots.begin(), geometry.knots.end(), delta,
            [](float value, const auto& knot) { return value < knot.delta_rad; });
        const auto index = std::clamp<std::size_t>(
            upper_iterator - geometry.knots.begin(), 1, geometry.knots.size() - 1);
        const auto& lower = geometry.knots[index - 1];
        const auto& upper = geometry.knots[index];
        const float blend =
            std::clamp((delta - lower.delta_rad) / (upper.delta_rad - lower.delta_rad), 0.F, 1.F);
        standing_reference::Vector3 value{};
        for (std::size_t j = 0; j < 3; ++j)
            value[j] = lower.center_m[j] + blend * (upper.center_m[j] - lower.center_m[j])
                     - geometry.hip_origin_m[j];
        const auto& axis = geometry.hip_axis;
        const standing_reference::Vector3 cross{
            axis[1] * value[2] - axis[2] * value[1], axis[2] * value[0] - axis[0] * value[2],
            axis[0] * value[1] - axis[1] * value[0]};
        const float dot = axis[0] * value[0] + axis[1] * value[1] + axis[2] * value[2];
        const float cosine = std::cos(hip - geometry.hip_reference_rad);
        const float sine = std::sin(hip - geometry.hip_reference_rad);
        standing_reference::Vector3 center{};
        for (std::size_t j = 0; j < 3; ++j)
            center[j] = cosine * value[j] + sine * cross[j] + (1.F - cosine) * axis[j] * dot
                      + geometry.hip_origin_m[j];
        return center;
    }

    LocalStandingState state_;
};

} // namespace rmcs_core::controller::chassis
