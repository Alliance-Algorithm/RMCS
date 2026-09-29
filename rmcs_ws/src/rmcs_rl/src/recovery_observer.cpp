#include "recovery_observer.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <ranges>
#include <stdexcept>
#include <utility>

namespace rmcs::rl {
namespace {
constexpr double kPi = std::numbers::pi;

struct SideState {
    double knee_deg;
    double knee_slope;
    double slider_m;
    double slider_slope;
    Eigen::Vector3d wheel_center_m;
};

SideState lookup(const RecoverySideTable& table, double hip, double auxiliary) {
    const double relative = auxiliary - hip;
    const auto upper = std::clamp<std::size_t>(
        std::upper_bound(table.delta_rad.begin(), table.delta_rad.end(), relative)
            - table.delta_rad.begin(),
        1, table.delta_rad.size() - 1);
    const auto lower = upper - 1;
    const double width = table.delta_rad[upper] - table.delta_rad[lower];
    const double fraction = std::clamp((relative - table.delta_rad[lower]) / width, 0.0, 1.0);
    const auto mix = [lower, upper, fraction](const auto& values) {
        return values[lower] + fraction * (values[upper] - values[lower]);
    };
    const Eigen::Vector3d at_zero = mix(table.wheel_at_hip_zero_m);
    const Eigen::Vector3d displacement = at_zero - table.hip_origin_m;
    const double c = std::cos(hip), s = std::sin(hip);
    const Eigen::Vector3d rotated = c * displacement + s * table.hip_axis.cross(displacement)
                                  + (1 - c) * table.hip_axis * table.hip_axis.dot(displacement);
    return {
        mix(table.inner_knee_deg),
        (table.inner_knee_deg[upper] - table.inner_knee_deg[lower]) / width,
        mix(table.slider_m),
        (table.slider_m[upper] - table.slider_m[lower]) / width,
        table.hip_origin_m + rotated,
    };
}
} // namespace

RecoveryObserver::RecoveryObserver(RecoveryMechanism mechanism)
    : mechanism_(std::move(mechanism)) {
    if (!std::isfinite(mechanism_.wheel_radius_m) || mechanism_.wheel_radius_m <= 0
        || !std::isfinite(mechanism_.spring_stroke_m) || mechanism_.spring_stroke_m <= 0
        || mechanism_.shell_points_body_m.size() < 4
        || !std::ranges::all_of(
            mechanism_.spring_force_n, [](double x) { return std::isfinite(x); }))
        throw std::invalid_argument("Incomplete recovery mechanism and shell geometry");
    for (const auto& point : mechanism_.shell_points_body_m)
        if (!point.allFinite())
            throw std::invalid_argument("Non-finite shell geometry");
    for (const auto& side : mechanism_.sides) {
        const auto count = side.delta_rad.size();
        if (count < 2 || side.inner_knee_deg.size() != count || side.slider_m.size() != count
            || side.wheel_at_hip_zero_m.size() != count || !side.hip_origin_m.allFinite()
            || !side.hip_axis.allFinite() || std::abs(side.hip_axis.norm() - 1.0) > 1e-3
            || !std::isfinite(side.spring_compression_at_zero_m))
            throw std::invalid_argument("Incomplete per-side closed-chain calibration");
        for (std::size_t i = 0; i < count; ++i) {
            if (!std::isfinite(side.delta_rad[i]) || !std::isfinite(side.inner_knee_deg[i])
                || !std::isfinite(side.slider_m[i]) || !side.wheel_at_hip_zero_m[i].allFinite()
                || side.inner_knee_deg[i] < 40.0 || side.inner_knee_deg[i] > 110.0
                || side.slider_m[i] < 0.0 || side.slider_m[i] > mechanism_.spring_stroke_m
                || (i && side.delta_rad[i] <= side.delta_rad[i - 1]))
                throw std::invalid_argument("Invalid closed-chain LUT point or ordering");
            if (i > 0
                && (side.inner_knee_deg[i] - side.inner_knee_deg[i - 1])
                           * (side.inner_knee_deg[1] - side.inner_knee_deg[0])
                       <= 0)
                throw std::invalid_argument("Knee LUT must be strictly monotonic");
            if (i > 0
                && std::abs(
                       (side.inner_knee_deg[i] - side.inner_knee_deg[i - 1])
                       / (side.delta_rad[i] - side.delta_rad[i - 1]))
                       < 0.1)
                throw std::invalid_argument("Knee LUT slope is too small for torque projection");
        }
    }
    reset();
}

void RecoveryObserver::reset() noexcept {
    last_wheel_velocity_.setZero();
    last_probe_.setZero();
    last_omega_.setZero();
    probe_evidence_ = {};
    for (auto& side : evidence_age_)
        side.fill(10.0);
    last_height_m_ = filtered_height_rate_mps_ = 0.0;
    impact_age_s_ = 10.0;
    support_seconds_ = quiet_seconds_ = 0.0;
    tick_ = 0;
    initialized_ = false;
}

RecoveryFeedback RecoveryObserver::update(
    const RecoveryVector6& q, const RecoveryVector6& dq, const Eigen::Vector3d& gravity,
    const Eigen::Vector3d& omega, const Eigen::Vector3d& acceleration, double dt) {
    RecoveryFeedback result;
    result.q = q;
    result.dq = dq;
    result.gravity = gravity;
    result.omega = omega;
    if (!std::isfinite(dt) || dt <= 0 || dt > 0.02 || !q.allFinite() || !dq.allFinite()
        || !gravity.allFinite() || !omega.allFinite() || !acceleration.allFinite()
        || std::abs(gravity.norm() - 1.0) > 0.1) {
        reset();
        return result;
    }

    std::array<Eigen::Vector3d, 2> wheel;
    std::array<double, 2> height;
    result.geometry_valid = true;
    for (int side = 0; side < 2; ++side) {
        const auto& table = mechanism_.sides[side];
        const int hip = 2 * side;
        const double delta = q[hip + 1] - q[hip];
        const auto pose = lookup(table, q[hip], q[hip + 1]);
        result.geometry_valid &= delta >= table.delta_rad.front() - 0.002
                              && delta <= table.delta_rad.back() + 0.002 && pose.knee_deg >= 40.0
                              && pose.knee_deg <= 110.0;
        const double compression = table.spring_compression_at_zero_m - pose.slider_m;
        result.geometry_valid &= compression >= 0.0 && compression <= mechanism_.spring_stroke_m;
        const double u = std::clamp(compression / mechanism_.spring_stroke_m, 0.0, 1.0);
        const auto& c = mechanism_.spring_force_n;
        const double force = c[0] + u * (c[1] + u * (c[2] + u * c[3]));
        result.spring_compensation_nm[side] = force * pose.slider_slope;
        result.inner_knee_deg[side] = pose.knee_deg;
        result.inner_knee_slope_deg_per_rad[side] = pose.knee_slope;
        result.spring_compression_m[side] = compression;
        wheel[side] = pose.wheel_center_m;
        height[side] = mechanism_.wheel_radius_m + gravity.dot(wheel[side]);
    }
    if (!result.geometry_valid)
        return result;
    result.spring_compensation_valid = true;
    result.wheel_heights_valid = true;
    result.wheel_height_difference = -gravity.dot(wheel[0] - wheel[1]);
    result.height_if_grounded = (height[0] + height[1]) / 2.0;
    const double candidate_height = result.height_if_grounded;
    const double angle = std::acos(std::clamp(-gravity.z(), -1.0, 1.0));
    const double acceleration_norm = acceleration.norm();
    result.specific_force_norm_mps2 = acceleration_norm;
    const double gyro_acceleration = initialized_ ? (omega - last_omega_).norm() / dt : 0.0;
    const bool impact =
        acceleration_norm > 16.0 || (angle > 20.0 * kPi / 180.0 && gyro_acceleration > 20.0);
    impact_age_s_ = impact ? 0.0 : impact_age_s_ + dt;
    const double raw_rate = initialized_ ? (candidate_height - last_height_m_) / dt : 0.0;
    filtered_height_rate_mps_ += dt / (0.05 + dt) * (raw_rate - filtered_height_rate_mps_);
    last_height_m_ = candidate_height;
    const auto wheel_vel = dq.tail<2>().eval();
    const bool geometrically_supported = candidate_height > 0.27 && candidate_height < 0.36
                                      && std::abs(result.wheel_height_difference) < 0.03
                                      && angle < 20.0 * kPi / 180.0 && omega.norm() < 1.5
                                      && dq.head<4>().cwiseAbs().maxCoeff() < 6.0
                                      && wheel_vel.cwiseAbs().maxCoeff() < 10.0
                                      && acceleration_norm > 6.0 && !impact && initialized_;
    const double wheel_acceleration =
        initialized_ ? ((wheel_vel - last_wheel_velocity_) / dt).cwiseAbs().maxCoeff() : 0.0;
    for (int side = 0; side < 2; ++side) {
        for (int direction = 0; direction < 2; ++direction) {
            evidence_age_[side][direction] += dt;
            const bool tested =
                std::abs(last_probe_[side]) >= 0.14 && (last_probe_[side] > 0) == (direction == 1);
            if (tested && geometrically_supported && wheel_acceleration < 100.0
                && gyro_acceleration < 90.0) {
                probe_evidence_[side][direction] = true;
                evidence_age_[side][direction] = 0.0;
            }
            if (!geometrically_supported || evidence_age_[side][direction] > 0.6)
                probe_evidence_[side][direction] = false;
        }
    }
    bool all_probe_directions = true;
    for (const auto& side : probe_evidence_)
        for (bool evidence : side)
            all_probe_directions &= evidence;
    quiet_seconds_ = last_probe_.cwiseAbs().maxCoeff() > 0.0 ? 0.0 : quiet_seconds_ + dt;
    support_seconds_ = geometrically_supported ? support_seconds_ + dt : 0.0;
    result.contact_candidate =
        geometrically_supported
        || (candidate_height > 0.12 && angle < 145.0 * kPi / 180.0 && impact_age_s_ < 0.15);

    double shell_clearance = 10.0;
    for (const auto& point : mechanism_.shell_points_body_m)
        shell_clearance = std::min(shell_clearance, candidate_height - gravity.dot(point));
    result.body_clear = geometrically_supported && shell_clearance > 0.01;
    result.height_valid = result.contact_candidate && std::abs(height[0] - height[1]) < 0.03;
    result.settled =
        geometrically_supported && all_probe_directions && result.body_clear
        && quiet_seconds_ >= 0.05 && support_seconds_ >= 0.1 && angle < 8.0 * kPi / 180.0
        && omega.norm() < 0.75 && std::abs(filtered_height_rate_mps_) < 0.15
        && std::abs(mechanism_.wheel_radius_m * (wheel_vel[0] - wheel_vel[1]) / 2.0) < 0.25;
    result.support_confirmed = result.settled;
    last_probe_.setZero();
    last_wheel_velocity_ = wheel_vel;
    last_omega_ = omega;
    initialized_ = true;
    return result;
}

Eigen::Vector2d RecoveryObserver::probe_command(bool preparing, const RecoveryFeedback& feedback) {
    Eigen::Vector2d pulse = Eigen::Vector2d::Zero();
    bool all_directions = true;
    for (const auto& side : probe_evidence_)
        for (bool evidence : side)
            all_directions &= evidence;
    const double angle = std::acos(std::clamp(-feedback.gravity.z(), -1.0, 1.0));
    const bool eligible = preparing && feedback.geometry_valid && feedback.height_valid
                       && !all_directions && feedback.height_if_grounded > 0.21
                       && feedback.height_if_grounded < 0.42 && angle < 65.0 * kPi / 180.0
                       && feedback.omega.norm() < 1.5
                       && feedback.dq.head<4>().cwiseAbs().maxCoeff() < 2.0
                       && feedback.dq.tail<2>().cwiseAbs().maxCoeff() < 6.0;
    if (eligible) {
        const int stage = (tick_ / 3) % 4;
        pulse[stage / 2] = stage % 2 == 0 ? 0.18 : -0.18;
    }
    ++tick_;
    last_probe_ = pulse;
    return pulse;
}

void RecoveryObserver::constrain_policy_goal(Eigen::Vector4d& goal, double knee_margin_deg) const {
    for (int side = 0; side < 2; ++side) {
        const auto& table = mechanism_.sides[side];
        const double desired = goal[2 * side + 1] - goal[2 * side];
        const double direction = table.inner_knee_deg[1] - table.inner_knee_deg[0];
        const auto invert = [&table, direction](double knee_deg) {
            for (std::size_t i = 1; i < table.delta_rad.size(); ++i) {
                const double a = table.inner_knee_deg[i - 1], b = table.inner_knee_deg[i];
                if (knee_deg >= std::min(a, b) && knee_deg <= std::max(a, b))
                    return table.delta_rad[i - 1]
                         + (knee_deg - a) * (table.delta_rad[i] - table.delta_rad[i - 1]) / (b - a);
            }
            return (knee_deg < table.inner_knee_deg.front()) == (direction > 0)
                     ? table.delta_rad.front()
                     : table.delta_rad.back();
        };
        const double a = invert(40.0 + knee_margin_deg);
        const double b = invert(110.0 - knee_margin_deg);
        goal[2 * side + 1] = goal[2 * side] + std::clamp(desired, std::min(a, b), std::max(a, b));
    }
}

} // namespace rmcs::rl
