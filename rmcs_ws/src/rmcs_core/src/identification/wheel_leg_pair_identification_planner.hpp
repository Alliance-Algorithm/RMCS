#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace rmcs_core::controller::identification {

struct PairLimits {
    // All joint arrays are ordered left hip, left auxiliary knee, right hip, right auxiliary knee.
    std::array<double, 4> sign{}, offset{}, root_min{}, root_max{}, max_speed{}, max_acceleration{},
        braking_acceleration{}, max_torque{};
    std::array<double, 2> spring_min{}, spring_max{};
    double joint_margin = 0.0, spring_margin = 0.0;
};

// The earlier VEL driver exposed q = remainder(sign*angle_api + offset, 2*pi).
// Apply the offset BEFORE wrapping, or a real pose across the wrap seam will
// appear one complete revolution outside its calibrated model coordinates.
inline double model_angle_from_api(double api, double sign, double offset, bool phase_api) {
    const double model = sign * api + offset;
    return phase_api ? std::remainder(model, 2.0 * std::numbers::pi) : model;
}

// The thighs rotate continuously: +/-pi is only a phase seam, not a hard stop.
inline double unwrap_model_phase(double phase, double previous) {
    return previous + std::remainder(phase - previous, 2.0 * std::numbers::pi);
}

inline double align_auxiliary_to_pair_branch(
    double knee_phase, double hip, double delta_min, double delta_max) {
    const double middle = (delta_min + delta_max) / 2.0;
    const double displacement = hip + middle - knee_phase;
    const double revolutions = std::round(displacement / (2.0 * std::numbers::pi));
    return knee_phase + 2.0 * std::numbers::pi * revolutions;
}

enum class PairFault {
    kNone,
    kNonfinite,
    kRootLimit,
    kSpringLimit,
    kKneeLimit,
    kSpeed,
    kStoppingDistance,
    kAcceleration,
    kTorque
};

struct PairSample {
    std::array<double, 2> position{}, velocity{}, acceleration{};
    double beta = 0.0;
    int segment_id = -1;
    bool validation = false;
    std::uint8_t waveform = 0; // 0=dwell, 1=move, 2=multisine, 3=step, 4=triangle, 5=validate
};

inline void validate_limits(const PairLimits& limits) {
    const auto finite = [](double x) { return std::isfinite(x); };
    for (std::size_t i = 0; i < 4; ++i) {
        if (!finite(limits.sign[i]) || std::abs(limits.sign[i]) != 1.0 || !finite(limits.offset[i])
            || !finite(limits.root_min[i]) || !finite(limits.root_max[i])
            || limits.root_max[i] <= limits.root_min[i] || !finite(limits.max_speed[i])
            || limits.max_speed[i] <= 0 || !finite(limits.max_acceleration[i])
            || limits.max_acceleration[i] <= 0 || !finite(limits.braking_acceleration[i])
            || limits.braking_acceleration[i] <= 0
            || limits.braking_acceleration[i] > limits.max_acceleration[i]
            || !finite(limits.max_torque[i]) || limits.max_torque[i] <= 0)
            throw std::invalid_argument("Invalid measured joint calibration or limits");
    }
    for (std::size_t side = 0; side < 2; ++side)
        if (!finite(limits.spring_min[side]) || !finite(limits.spring_max[side])
            || limits.spring_min[side] >= limits.spring_max[side])
            throw std::invalid_argument("Invalid measured spring travel");
    if (!finite(limits.joint_margin) || limits.joint_margin <= 0 || !finite(limits.spring_margin)
        || limits.spring_margin <= 0)
        throw std::invalid_argument("Positive physical boundary margins required");
}

inline bool motor_states_safe(
    std::size_t selected_side, const std::array<int, 4>& status, const std::array<int, 4>& fault,
    bool running, bool allow_selected_fault = false) {
    if (selected_side > 1)
        return false;
    for (std::size_t i = 0; i < status.size(); ++i) {
        if (i / 2 == selected_side && allow_selected_fault && status[i] >= 8 && status[i] <= 14
            && fault[i] == status[i] && status[2 * selected_side] != 1
            && status[2 * selected_side + 1] != 1)
            continue;
        if (fault[i] != 0 || (i / 2 != selected_side && status[i] != 0)
            || (i / 2 == selected_side && (status[i] < (running ? 1 : 0) || status[i] > 1)))
            return false;
    }
    return true;
}

inline PairFault check_probe_pair(
    const PairLimits& limits, std::size_t side, const std::array<double, 2>& q,
    const std::array<double, 2>& dq, bool check_dynamic_limits = true,
    bool check_drive_difference = true) {
    if (side > 1 || !std::isfinite(q[0]) || !std::isfinite(q[1]) || !std::isfinite(dq[0])
        || !std::isfinite(dq[1]))
        return PairFault::kNonfinite;
    const double delta = q[1] - q[0], delta_speed = dq[1] - dq[0];
    const double min = limits.spring_min[side] + limits.spring_margin;
    const double max = limits.spring_max[side] - limits.spring_margin;
    if (check_drive_difference && (delta <= min || delta >= max))
        return PairFault::kSpringLimit;
    for (std::size_t j = 0; j < 2; ++j) {
        const auto i = 2 * side + j;
        if (q[j] <= limits.root_min[i] + limits.joint_margin
            || q[j] >= limits.root_max[i] - limits.joint_margin)
            return PairFault::kRootLimit;
    }
    // Reference/initial-pose admission always checks the drive difference.
    // An explicit observation run may make that uncalibrated proxy diagnostic
    // after admission. Independent root bounds and finite data still apply.
    if (!check_dynamic_limits)
        return PairFault::kNone;
    const double brake =
        std::min(limits.braking_acceleration[2 * side], limits.braking_acceleration[2 * side + 1]);
    if (delta - min < std::pow(std::max(0.0, -delta_speed), 2) / (2 * brake)
        || max - delta < std::pow(std::max(0.0, delta_speed), 2) / (2 * brake))
        return PairFault::kStoppingDistance;
    for (std::size_t j = 0; j < 2; ++j) {
        const auto i = 2 * side + j;
        if (std::abs(dq[j]) > limits.max_speed[i])
            return PairFault::kSpeed;
        if (q[j] - limits.root_min[i] - limits.joint_margin
                < std::pow(std::max(0.0, -dq[j]), 2) / (2 * limits.braking_acceleration[i])
            || limits.root_max[i] - limits.joint_margin - q[j]
                   < std::pow(std::max(0.0, dq[j]), 2) / (2 * limits.braking_acceleration[i]))
            return PairFault::kStoppingDistance;
    }
    return PairFault::kNone;
}

inline PairFault
    check_probe_reference(const PairLimits& limits, std::size_t side, const PairSample& sample) {
    // Planned kinematics must fit geometry, speed and acceleration budgets.
    // An unidentified braking estimate cannot certify (or reject) this path.
    if (const auto fault = check_probe_pair(limits, side, sample.position, sample.velocity, false);
        fault != PairFault::kNone)
        return fault;
    for (std::size_t j = 0; j < 2; ++j)
        if (std::abs(sample.velocity[j]) > limits.max_speed[2 * side + j])
            return PairFault::kSpeed;
    for (std::size_t j = 0; j < 2; ++j)
        if (!std::isfinite(sample.acceleration[j])
            || std::abs(sample.acceleration[j]) > limits.max_acceleration[2 * side + j])
            return PairFault::kAcceleration;
    return PairFault::kNone;
}

struct PdResult {
    double position_error, velocity_target, speed_error, preclip, integral_torque, torque;
};

// Same PC position PD as the RL path. The desired velocity is zero; the
// trajectory derivative is telemetry only, never a feedforward torque term.
inline PdResult
    pd_step(double q_ref, double q, double dq, double kp, double kd, double max_torque) {
    const double error = q_ref - q;
    const double requested = kp * error - kd * dq;
    return {error, 0.0, -dq, requested, 0.0, std::clamp(requested, -max_torque, max_torque)};
}

inline std::array<double, 6> selected_torque_api(
    std::size_t side, const PairLimits& limits, const std::array<double, 2>& torque_model) {
    std::array<double, 6> command{};
    if (side < 2)
        for (std::size_t j = 0; j < 2; ++j)
            command[2 * side + j] = limits.sign[2 * side + j] * torque_model[j];
    return command;
}

} // namespace rmcs_core::controller::identification
