#include "recovery_observer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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
    Eigen::Vector3d knee_axis;
    Eigen::Vector3d wheel_axis;
};

SideState lookup(const RecoverySideTable& table, double hip, double auxiliary) {
    const double relative = auxiliary - hip;
    const auto upper = std::clamp<std::size_t>(
        std::lower_bound(table.delta_rad.begin(), table.delta_rad.end(), relative)
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
    const auto rotate = [&](const Eigen::Vector3d& vector) -> Eigen::Vector3d {
        return c * vector + s * table.hip_axis.cross(vector)
             + (1 - c) * table.hip_axis * table.hip_axis.dot(vector);
    };
    return {
        mix(table.inner_knee_deg),
        (table.inner_knee_deg[upper] - table.inner_knee_deg[lower]) / width,
        mix(table.slider_m),
        table.slider_slope_m_per_rad.empty()
            ? (table.slider_m[upper] - table.slider_m[lower]) / width
            : mix(table.slider_slope_m_per_rad),
        table.hip_origin_m + rotate(displacement),
        table.knee_axis_at_hip_zero.empty() ? Eigen::Vector3d::Zero().eval()
                                            : rotate(mix(table.knee_axis_at_hip_zero)),
        table.wheel_axis_at_hip_zero.empty() ? Eigen::Vector3d::Zero().eval()
                                             : rotate(mix(table.wheel_axis_at_hip_zero)),
    };
}
} // namespace

RecoveryObserver::RecoveryObserver(RecoveryMechanism mechanism, RecoveryObserverConfig config)
    : mechanism_(std::move(mechanism))
    , config_(config) {
    const auto positive_values = std::array{
        config_.probe_torque_nm,
        config_.minimum_submitted_torque_nm,
        config_.minimum_feedback_torque_nm,
        config_.feedback_torque_ratio,
        config_.pulse_seconds,
        config_.response_seconds,
        config_.quiet_seconds,
        config_.support_seconds,
        config_.evidence_ttl_seconds,
        config_.maximum_sample_age_seconds,
        config_.maximum_imu_sample_age_seconds,
        config_.maximum_submission_age_seconds,
        config_.maximum_feedback_interval_seconds,
        config_.maximum_wheel_acceleration_rad_s2,
        config_.maximum_gyro_acceleration_rad_s2};
    if (!std::ranges::all_of(positive_values, [](double x) { return std::isfinite(x) && x > 0; })
        || config_.minimum_submitted_torque_nm > config_.probe_torque_nm
        || config_.minimum_feedback_torque_nm > config_.probe_torque_nm
        || config_.feedback_torque_ratio > 1.0 || config_.response_seconds >= config_.pulse_seconds
        || config_.evidence_ttl_seconds <= 4 * config_.pulse_seconds + config_.quiet_seconds)
        throw std::invalid_argument("Invalid recovery support probe thresholds");
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
        const bool has_axes = !side.knee_axis_at_hip_zero.empty()
                           || !side.wheel_axis_at_hip_zero.empty() || side.passive_knee_sign != 0.0;
        if ((has_axes
             && (side.knee_axis_at_hip_zero.size() != count
                 || side.wheel_axis_at_hip_zero.size() != count
                 || !std::isfinite(side.passive_knee_sign)
                 || std::abs(std::abs(side.passive_knee_sign) - 1.0) > 1e-6))
            || (!side.slider_slope_m_per_rad.empty()
                && side.slider_slope_m_per_rad.size() != count))
            throw std::invalid_argument("Incomplete recovery joint-axis/slider-derivative LUT");
        for (std::size_t i = 0; i < count; ++i) {
            if (!std::isfinite(side.delta_rad[i]) || !std::isfinite(side.inner_knee_deg[i])
                || !std::isfinite(side.slider_m[i]) || !side.wheel_at_hip_zero_m[i].allFinite()
                || side.inner_knee_deg[i] < 40.0 || side.inner_knee_deg[i] > 110.0
                || side.slider_m[i] < 0.0 || side.slider_m[i] > mechanism_.spring_stroke_m
                || (i && side.delta_rad[i] <= side.delta_rad[i - 1]))
                throw std::invalid_argument("Invalid closed-chain LUT point or ordering");
            if ((has_axes
                 && (!side.knee_axis_at_hip_zero[i].allFinite()
                     || !side.wheel_axis_at_hip_zero[i].allFinite()
                     || std::abs(side.knee_axis_at_hip_zero[i].norm() - 1.0) > 1e-3
                     || std::abs(side.wheel_axis_at_hip_zero[i].norm() - 1.0) > 1e-3))
                || (!side.slider_slope_m_per_rad.empty()
                    && !std::isfinite(side.slider_slope_m_per_rad[i])))
                throw std::invalid_argument("Invalid recovery joint axis or slider derivative");
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
    clear_sensor_evidence_();
    last_height_m_ = filtered_height_rate_mps_ = 0.0;
    impact_age_s_ = 10.0;
    alignment_seconds_ = probe_elapsed_seconds_ = last_update_dt_ = 0.0;
    initialized_ = false;
}

void RecoveryObserver::clear_sensor_evidence_() noexcept {
    probe_evidence_ = {};
    bad_probe_samples_ = {};
    for (auto& side : evidence_age_)
        side.fill(config_.evidence_ttl_seconds + 1.0);
    last_wheel_sample_ns_ = {};
    last_wheel_sequence_ = {};
    probe_submission_ns_ = {};
    probe_request_ns_ = {};
    probe_submission_direction_ = {};
    last_sensor_ns_ = current_sensor_ns_ = 0;
    last_imu_sample_ns_ = last_imu_sequence_ = 0;
    last_omega_.setZero();
    support_seconds_ = quiet_seconds_ = lost_support_seconds_ = 0.0;
}

bool RecoveryObserver::update_wheel_samples_(
    const RecoverySensorData& sensors, const Eigen::Vector2d& velocity,
    std::array<bool, 2>& new_samples, Eigen::Vector2d& acceleration) {
    const auto invalid = [this] {
        clear_sensor_evidence_();
        return false;
    };
    if (sensors.steady_ns == 0
        || sensors.steady_ns > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())
        || (last_sensor_ns_ != 0 && sensors.steady_ns <= last_sensor_ns_)
        || !sensors.wheel_torque_feedback_nm.allFinite()
        || !sensors.wheel_torque_submitted_nm.allFinite())
        return invalid();
    for (int side = 0; side < 2; ++side) {
        const auto stamp = sensors.wheel_feedback_ns[side];
        const auto sequence = sensors.wheel_feedback_sequence[side];
        if (stamp == 0 || sequence == 0 || stamp > sensors.steady_ns
            || (sensors.steady_ns - stamp) * 1e-9 > config_.maximum_sample_age_seconds)
            return invalid();
        if (last_wheel_sequence_[side] != 0) {
            if (sequence < last_wheel_sequence_[side]
                || (sequence == last_wheel_sequence_[side]
                    && (stamp != last_wheel_sample_ns_[side]
                        || velocity[side] != last_wheel_velocity_[side])))
                return invalid();
            if (sequence != last_wheel_sequence_[side]) {
                if (stamp <= last_wheel_sample_ns_[side])
                    return invalid();
                const double sample_dt = (stamp - last_wheel_sample_ns_[side]) * 1e-9;
                if (sample_dt > config_.maximum_feedback_interval_seconds)
                    return invalid();
                acceleration[side] =
                    std::abs(velocity[side] - last_wheel_velocity_[side]) / sample_dt;
                new_samples[side] = true;
            }
        }
    }
    for (int side = 0; side < 2; ++side) {
        last_wheel_sequence_[side] = sensors.wheel_feedback_sequence[side];
        last_wheel_sample_ns_[side] = sensors.wheel_feedback_ns[side];
        last_wheel_velocity_[side] = velocity[side];
    }
    current_sensor_ns_ = last_sensor_ns_ = sensors.steady_ns;
    return true;
}

bool RecoveryObserver::update_imu_sample_(
    const RecoverySensorData& sensors, const Eigen::Vector3d& omega, bool& new_sample,
    double& acceleration) {
    const auto stamp = sensors.imu_feedback_ns;
    const auto sequence = sensors.imu_feedback_sequence;
    const auto invalid = [this] {
        clear_sensor_evidence_();
        return false;
    };
    if (stamp == 0 || sequence == 0 || stamp > sensors.steady_ns
        || (sensors.steady_ns - stamp) * 1e-9 > config_.maximum_imu_sample_age_seconds)
        return invalid();
    if (last_imu_sequence_ != 0) {
        if (sequence < last_imu_sequence_
            || (sequence == last_imu_sequence_
                && (stamp != last_imu_sample_ns_ || omega != last_omega_)))
            return invalid();
        if (sequence != last_imu_sequence_) {
            if (stamp <= last_imu_sample_ns_)
                return invalid();
            const double sample_dt = (stamp - last_imu_sample_ns_) * 1e-9;
            if (sample_dt > config_.maximum_feedback_interval_seconds)
                return invalid();
            acceleration = (omega - last_omega_).norm() / sample_dt;
            new_sample = true;
        }
    }
    last_imu_sequence_ = sequence;
    last_imu_sample_ns_ = stamp;
    last_omega_ = omega;
    return true;
}

RecoveryFeedback RecoveryObserver::update(
    const RecoveryVector6& q, const RecoveryVector6& dq, const Eigen::Vector3d& gravity,
    const Eigen::Vector3d& omega, const Eigen::Vector3d& acceleration, double dt) {
    return update(q, dq, gravity, omega, acceleration, dt, RecoverySensorData{});
}

RecoveryFeedback RecoveryObserver::update(
    const RecoveryVector6& q, const RecoveryVector6& dq, const Eigen::Vector3d& gravity,
    const Eigen::Vector3d& omega, const Eigen::Vector3d& acceleration, double dt,
    const RecoverySensorData& sensors) {
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
    const bool attitude_valid = sensors.world_base_orientation
                             && sensors.world_base_orientation->coeffs().allFinite()
                             && std::abs(sensors.world_base_orientation->norm() - 1.0) < 1e-3;
    result.world_wheel_omega_valid = attitude_valid;
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
        const bool axes_valid =
            !table.knee_axis_at_hip_zero.empty() && !table.wheel_axis_at_hip_zero.empty();
        result.world_wheel_omega_valid &= axes_valid;
        if (attitude_valid && axes_valid) {
            const double passive_speed =
                table.passive_knee_sign * pose.knee_slope * (dq[hip + 1] - dq[hip]) * kPi / 180.0;
            const Eigen::Vector3d wheel_omega_body = omega + table.hip_axis * dq[hip]
                                                   + pose.knee_axis * passive_speed
                                                   + pose.wheel_axis * dq[4 + side];
            result.world_wheel_omega[side] =
                (*sensors.world_base_orientation * wheel_omega_body).y();
        }
    }
    if (!result.geometry_valid) {
        reset();
        return result;
    }
    result.spring_compensation_valid = true;
    result.wheel_heights_valid = true;
    result.wheel_height_difference = -gravity.dot(wheel[0] - wheel[1]);
    result.height_if_grounded = (height[0] + height[1]) / 2.0;
    const double candidate_height = result.height_if_grounded;
    const double angle = std::acos(std::clamp(-gravity.z(), -1.0, 1.0));
    const double acceleration_norm = acceleration.norm();
    result.specific_force_norm_mps2 = acceleration_norm;
    std::array<bool, 2> new_samples{};
    Eigen::Vector2d wheel_acceleration = Eigen::Vector2d::Zero();
    bool new_imu_sample = false;
    double gyro_acceleration = 0.0;
    const auto wheel_vel = dq.tail<2>().eval();
    const bool sensors_valid =
        update_wheel_samples_(sensors, wheel_vel, new_samples, wheel_acceleration)
        && update_imu_sample_(sensors, omega, new_imu_sample, gyro_acceleration);
    const bool impact =
        acceleration_norm > 16.0 || (angle > 20.0 * kPi / 180.0 && gyro_acceleration > 20.0);
    impact_age_s_ = impact ? 0.0 : impact_age_s_ + dt;
    const double raw_rate = initialized_ ? (candidate_height - last_height_m_) / dt : 0.0;
    filtered_height_rate_mps_ += dt / (0.05 + dt) * (raw_rate - filtered_height_rate_mps_);
    last_height_m_ = candidate_height;
    bool submissions_fresh = sensors_valid;
    for (int side = 0; side < 2; ++side) {
        const auto submitted_ns = sensors.wheel_torque_submitted_ns[side];
        submissions_fresh &= sensors.wheel_tx_kind[side] == 1 && submitted_ns != 0
                          && submitted_ns <= sensors.steady_ns
                          && (sensors.steady_ns - std::min(submitted_ns, sensors.steady_ns)) * 1e-9
                                 <= config_.maximum_submission_age_seconds;
    }
    const bool geometrically_supported =
        candidate_height > 0.27 && candidate_height < 0.36
        && std::abs(result.wheel_height_difference) < 0.03 && angle < 20.0 * kPi / 180.0
        && omega.norm() < 1.5 && dq.head<4>().cwiseAbs().maxCoeff() < 6.0
        && wheel_vel.cwiseAbs().maxCoeff() < 10.0 && !impact && initialized_;
    result.geometrically_supported = geometrically_supported;
    lost_support_seconds_ = geometrically_supported ? 0.0 : lost_support_seconds_ + dt;
    result.height_rate_mps = filtered_height_rate_mps_;
    result.gyro_acceleration_rad_s2 = gyro_acceleration;
    result.wheel_acceleration_rad_s2 = wheel_acceleration;
    for (int side = 0; side < 2; ++side) {
        const double expected_probe = last_probe_[side];
        const int expected_direction = (expected_probe > 0.0) - (expected_probe < 0.0);
        const double submitted = sensors.wheel_torque_submitted_nm[side];
        const bool probe_submitted =
            submissions_fresh && std::abs(expected_probe) >= config_.minimum_submitted_torque_nm
            && probe_request_ns_[side] != 0
            && sensors.wheel_torque_submitted_ns[side] >= probe_request_ns_[side]
            && expected_direction * submitted >= config_.minimum_submitted_torque_nm;
        if (!probe_submitted) {
            probe_submission_ns_[side] = 0;
            probe_submission_direction_[side] = 0;
        } else if (probe_submission_direction_[side] != expected_direction) {
            probe_submission_ns_[side] = sensors.wheel_torque_submitted_ns[side];
            probe_submission_direction_[side] = expected_direction;
        }
        // A recent re-transmission can be newer than the latest motor reply.
        // Retain the first submission of this pulse, then require a later reply.
        const bool current_response =
            probe_submitted && new_samples[side] && new_imu_sample
            && sensors.wheel_feedback_ns[side] > probe_submission_ns_[side]
            && sensors.imu_feedback_ns > probe_submission_ns_[side]
            && expected_direction * sensors.wheel_torque_feedback_nm[side] >= std::max(
                   config_.minimum_feedback_torque_nm,
                   config_.feedback_torque_ratio * std::abs(submitted));
        const bool pulse_mature =
            current_response
            && (sensors.wheel_feedback_ns[side] - probe_submission_ns_[side]) * 1e-9 + 1e-12
                   >= config_.response_seconds;
        for (int direction = 0; direction < 2; ++direction) {
            evidence_age_[side][direction] += dt;
            const bool tested =
                expected_direction != 0 && (expected_direction > 0) == (direction == 1);
            const bool loaded_response =
                tested && pulse_mature && geometrically_supported && acceleration_norm > 6.0
                && wheel_acceleration[side] < config_.maximum_wheel_acceleration_rad_s2
                && gyro_acceleration < config_.maximum_gyro_acceleration_rad_s2;
            if (loaded_response) {
                bad_probe_samples_[side][direction] = 0;
                // Match the reference's loaded-sample predicate. Fresh CAN
                // current and IMU samples additionally establish causality;
                // demanding consecutive low-acceleration samples changed the
                // physical behavior under encoder/solver jitter.
                probe_evidence_[side][direction] = true;
                evidence_age_[side][direction] = 0.0;
            }
            if (tested && pulse_mature && geometrically_supported && !loaded_response)
                ++bad_probe_samples_[side][direction];
            if (!sensors_valid || acceleration_norm <= 6.0 || lost_support_seconds_ >= 0.05
                || bad_probe_samples_[side][direction] >= 2
                || evidence_age_[side][direction] > config_.evidence_ttl_seconds)
                probe_evidence_[side][direction] = false;
            if (probe_evidence_[side][direction])
                result.probe_evidence_mask |= std::uint8_t{1} << (2 * side + direction);
        }
    }
    bool all_probe_directions = true;
    for (const auto& side : probe_evidence_)
        for (bool evidence : side)
            all_probe_directions &= evidence;
    quiet_seconds_ =
        sensors_valid && submissions_fresh && last_probe_.isZero() ? quiet_seconds_ + dt : 0.0;
    support_seconds_ = sensors_valid && geometrically_supported && acceleration_norm > 6.0
                         ? support_seconds_ + dt
                         : 0.0;
    result.probe_confirmed = sensors_valid && geometrically_supported && all_probe_directions;
    result.contact_candidate =
        geometrically_supported || result.probe_confirmed
        || (candidate_height > 0.12 && angle < 145.0 * kPi / 180.0 && impact_age_s_ < 0.15);

    double shell_clearance = 10.0;
    for (const auto& point : mechanism_.shell_points_body_m)
        shell_clearance = std::min(shell_clearance, candidate_height - gravity.dot(point));
    result.body_clear = geometrically_supported && shell_clearance > 0.01;
    result.height_valid = result.contact_candidate && acceleration_norm > 6.0
                       && std::abs(height[0] - height[1]) < 0.03;
    const bool aligned = candidate_height > 0.20 && candidate_height < 0.43
                      && std::abs(result.wheel_height_difference) < 0.03
                      && angle < 65.0 * kPi / 180.0 && acceleration_norm > 6.0;
    alignment_seconds_ = std::clamp(alignment_seconds_ + (aligned ? dt : -2 * dt), 0.0, 0.2);
    result.alignment_candidate = alignment_seconds_ >= 0.03 && acceleration_norm > 3.0;
    result.body_contact_suspected = candidate_height < 0.27 && angle < 65.0 * kPi / 180.0;
    result.settled =
        sensors_valid && submissions_fresh && geometrically_supported && all_probe_directions
        && acceleration_norm > 6.0 && result.body_clear && quiet_seconds_ >= config_.quiet_seconds
        && support_seconds_ >= config_.support_seconds && angle < 8.0 * kPi / 180.0
        && omega.norm() < 0.75 && std::abs(filtered_height_rate_mps_) < 0.15
        && std::abs(mechanism_.wheel_radius_m * (wheel_vel[0] - wheel_vel[1]) / 2.0) < 0.25;
    result.support_confirmed = result.settled;
    last_probe_.setZero();
    last_update_dt_ = dt;
    // The reference pulse schedule runs continuously, including ineligible ticks.
    probe_elapsed_seconds_ = std::fmod(probe_elapsed_seconds_ + dt, 4 * config_.pulse_seconds);
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
    const bool eligible = preparing && feedback.geometry_valid && !all_directions
                       && feedback.height_if_grounded > 0.21 && feedback.height_if_grounded < 0.42
                       && angle < 65.0 * kPi / 180.0 && feedback.omega.norm() < 1.5
                       && feedback.dq.head<4>().cwiseAbs().maxCoeff() < 2.0
                       && feedback.dq.tail<2>().cwiseAbs().maxCoeff() < 6.0;
    if (eligible) {
        const double elapsed = std::fmod(
            probe_elapsed_seconds_ - last_update_dt_ + 4 * config_.pulse_seconds,
            4 * config_.pulse_seconds);
        const int stage = static_cast<int>((elapsed + 1e-12) / config_.pulse_seconds) % 4;
        pulse[stage / 2] = stage % 2 == 0 ? config_.probe_torque_nm : -config_.probe_torque_nm;
    }
    last_probe_ = pulse;
    for (int side = 0; side < 2; ++side)
        probe_request_ns_[side] = pulse[side] != 0.0 ? current_sensor_ns_ : 0;
    return pulse;
}

std::array<double, 2> RecoveryObserver::inner_knee_limits_deg(std::size_t side) const noexcept {
    if (side >= mechanism_.sides.size())
        return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
    const auto& knee = mechanism_.sides[side].inner_knee_deg;
    return {std::min(knee.front(), knee.back()), std::max(knee.front(), knee.back())};
}

void RecoveryObserver::constrain_policy_goal(Eigen::Vector4d& goal, double knee_margin_deg) const {
    if (!std::isfinite(knee_margin_deg) || knee_margin_deg < 0)
        throw std::invalid_argument("Invalid recovery knee margin");
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
        const auto limits = inner_knee_limits_deg(side);
        const double margin = std::min(knee_margin_deg, (limits[1] - limits[0]) / 2);
        const double a = invert(limits[0] + margin);
        const double b = invert(limits[1] - margin);
        goal[2 * side + 1] = goal[2 * side] + std::clamp(desired, std::min(a, b), std::max(a, b));
    }
}

} // namespace rmcs::rl
