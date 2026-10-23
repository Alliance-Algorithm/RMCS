#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>

// Sensor-only standing observation; no actuator commands. Ported from the
// frozen training observer and validated against recorded policy frames.
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

inline bool standing_gates_eligible(const std::array<float, 35>& observation) {
    if (!std::all_of(
            observation.begin(), observation.end(), [](float v) { return std::isfinite(v); }))
        return false;
    const auto clamp = [](float value) { return std::clamp(value, 0.F, 1.F); };
    const float request =
        std::max(std::abs(observation[0]) / .12F, std::abs(observation[2]) / .25F);
    const float tilt = clamp((-observation[9] - .94F) / .04F);
    const float gyro = clamp(
        2.F * std::sqrt(observation[4] * observation[4] + observation[5] * observation[5]) - .4F);
    const float quiet = clamp(1.F - request) * tilt * (1.F - gyro);
    const float original = clamp((observation[9] + .94F) / .44F);
    constexpr float severe_width = static_cast<float>(.5 - .08715574274765817);
    const float severe = clamp((observation[9] + .5F) / severe_width);
    float phase_angle = 0.F;
    for (std::size_t j = 10; j < 14; ++j)
        phase_angle = std::max(phase_angle, std::abs(observation[j]));
    float phase = clamp((phase_angle - 1.F) / .8F);
    float shape_angle = 0.F;
    for (std::size_t j : {10U, 12U}) {
        const float difference = observation[j + 1] - observation[j];
        shape_angle =
            std::max(shape_angle, std::abs(std::atan2(std::sin(difference), std::cos(difference))));
    }
    phase = std::max(phase, clamp((shape_angle - .2F) / .2F));
    return quiet >= .999F && std::max(severe, phase * original) <= .05F;
}

struct StandingReferenceState {
    bool active = false;
    std::array<float, 2> anchor_m{};
    std::array<float, 2> filtered_m{};
    std::uint64_t severe_reset_count = 0;
};

class StandingReferenceV3 {
public:
    explicit StandingReferenceV3(const GeometryTable& geometry, bool reset_on_severe_tilt = true)
        : geometry_(geometry)
        , reset_on_severe_tilt_(reset_on_severe_tilt) {}

    void reset(std::uint64_t policy_tick = 0) {
        state_ = {};
        entry_tick_ = policy_tick;
        last_tick_.reset();
    }

    std::array<float, 2> sample(
        std::uint64_t policy_tick, const std::array<float, 6>& position,
        const std::array<float, 35>& observation, bool active_control = true) {
        if (policy_tick < entry_tick_)
            throw std::invalid_argument(
                "Policy clock moved backwards; reset the standing reference");
        const Vector3 gravity{observation[7], observation[8], observation[9]};
        const float magnitude = norm(gravity);
        const bool severe =
            finite(gravity) && magnitude > .5F && magnitude < 1.5F && gravity[2] / magnitude > -.5F;
        if (reset_on_severe_tilt_ && severe) {
            const auto count =
                state_.severe_reset_count + static_cast<std::uint64_t>(state_.active);
            reset(policy_tick);
            state_.severe_reset_count = count;
        }
        const auto coordinate = wheel_angle_coordinate(geometry_, position, gravity);
        const bool requested =
            active_control && std::abs(observation[0]) < .1F && std::abs(observation[2]) < .2F;
        if (!requested) {
            state_.active = false;
            state_.filtered_m = {};
            last_tick_.reset();
        }
        const bool eligible = standing_gates_eligible(observation)
                           && static_cast<float>(policy_tick - entry_tick_) * .005F >= .05F;
        if (requested && eligible && coordinate.valid && !state_.active) {
            state_.anchor_m = coordinate.coordinate_m;
            state_.filtered_m = {};
            state_.active = true;
        }
        const bool use = state_.active && requested && coordinate.valid;
        if (use && (!last_tick_ || *last_tick_ != policy_tick)) {
            constexpr float alpha = static_cast<float>(.005 / (.15 + .005));
            for (std::size_t side = 0; side < 2; ++side) {
                const float displacement =
                    std::clamp(coordinate.coordinate_m[side] - state_.anchor_m[side], -.25F, .25F);
                state_.filtered_m[side] += alpha * (displacement - state_.filtered_m[side]);
            }
            last_tick_ = policy_tick;
        }
        return use ? state_.filtered_m : std::array<float, 2>{};
    }

    const StandingReferenceState& state() const { return state_; }

private:
    const GeometryTable& geometry_;
    bool reset_on_severe_tilt_;
    StandingReferenceState state_;
    std::uint64_t entry_tick_ = 0;
    std::optional<std::uint64_t> last_tick_;
};

} // namespace rmcs_core::controller::chassis::standing_reference
