#include "v6_recovery_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace rmcs::rl {
namespace {
constexpr float kPi = std::numbers::pi_v<float>;
constexpr float kTau = 2.0f * kPi;
constexpr float radians(double degrees) {
    return static_cast<float>(degrees * std::numbers::pi / 180.0);
}
float wrapped(float value) { return std::atan2(std::sin(value), std::cos(value)); }
bool positive_finite(double value) { return std::isfinite(value) && value > 0.0; }
} // namespace

std::string_view v6_recovery_phase_name(V6RecoveryPhase phase) {
    constexpr std::array<std::string_view, 11> names{
        "SELECT", "FOLD", "PLANT", "ORBIT", "THRUST", "SIDE", "CAPTURE",
        "PREPARE", "BLEND", "RL", "FAILED"};
    const auto index = static_cast<std::size_t>(phase);
    return index < names.size() ? names[index] : "UNKNOWN";
}

V6RecoveryController::V6RecoveryController(const V6RecoveryConfig& config) : config_(config) {
    const std::array values{
        config.dt, config.dynamic_capture_height_min, config.stable_seconds, config.max_script_seconds,
        config.prepare_speed_rad_s, config.push_speed_rad_s, config.orbit_speed_rad_s,
        config.capture_speed_rad_s, config.side_speed_rad_s, config.side_angle_rad,
        config.plant_angle_rad, config.orbit_turns, config.handover_height_min,
        config.handover_height_max, config.handover_tilt_rad, config.handover_gyro_max,
        config.reroute_tilt_rad, config.reroute_support_lost_seconds, config.pitch_kp_nm_rad,
        config.pitch_kd_nm_s_rad, config.wheel_damping_nm_s_rad,
        config.prepare_tilt_limit_deg, config.capture_tilt_limit_deg};
    if (!std::all_of(values.begin(), values.end(), positive_finite)
        || config.dt != 0.005 || !std::isfinite(config.blend_seconds)
        || (config.blend_seconds != 0.0 && config.blend_seconds != 0.2)
        || (config.dynamic_takeover && config.blend_seconds != 0.0)
        || config.dynamic_capture_height_min < 0.20
        || config.dynamic_capture_height_min > config.handover_height_min
        || config.stable_seconds < 1.0 || config.max_script_seconds > 8.0
        || config.max_reroutes < 0 || config.max_reroutes > 1
        || config.handover_height_min >= config.handover_height_max
        || config.handover_tilt_rad >= config.reroute_tilt_rad
        || config.reroute_tilt_rad >= std::numbers::pi
        || config.prepare_tilt_limit_deg > config.capture_tilt_limit_deg
        || config.capture_tilt_limit_deg >= 90.0
        || !config.nominal.allFinite() || !config.fold.allFinite()
        || !config.thrust.allFinite() || !config.support.allFinite()
        || !config.rl_nominal.allFinite() || !config.root_axis_signs.allFinite()
        || !config.wheel_axis_signs.allFinite())
        throw std::invalid_argument("Invalid V6 recovery configuration");
    for (int i = 0; i < 4; ++i)
        if (std::abs(config.root_axis_signs[i]) != 1.0
            || config.root_axis_signs[i] != config.root_axis_signs[i ^ 1])
            throw std::invalid_argument("V6 recovery roots require signed coupled pairs");
    for (int i = 0; i < 2; ++i)
        if (std::abs(config.wheel_axis_signs[i]) != 1.0)
            throw std::invalid_argument("V6 recovery requires signed wheel motor axes");
    publish_();
}

std::uint64_t V6RecoveryController::ticks_(double seconds) const {
    return static_cast<std::uint64_t>(std::ceil(seconds / config_.dt - 1e-9));
}

void V6RecoveryController::reset_fsm_(const Vector6f& q, bool active) {
    enabled_ = active;
    phase_ = active ? V6RecoveryPhase::kSelect : V6RecoveryPhase::kRl;
    motion_released_ = !active;
    targets_ = continuous_q_ = canonical_q_ = last_raw_q_ = q;
    targets_.tail<2>().setZero();
    route_ = V6RecoveryRoute::kUpright;
    age_ticks_ = phase_ticks_ = ready_ticks_ = stable_ticks_ = plant_ticks_ = unsupported_ticks_ = 0;
    reroute_count_ = failure_code_ = 0;
    continuous_pitch_ = last_pitch_ = orbit_angle_ = plant_pitch_ = 0.0f;
    pitch_initialized_ = wheel_balance_active_ = false;
    orbit_start_.setZero();
    thrust_anchor_.setZero();
    side_start_.setZero();
    side_direction_.setZero();
}

void V6RecoveryController::reset(const V6RecoveryVector6& q, bool active,
                                 double initial_release_seconds) {
    if (!q.allFinite() || !std::isfinite(initial_release_seconds)
        || initial_release_seconds < 0.0 || initial_release_seconds >= config_.max_script_seconds)
        throw std::invalid_argument("V6 recovery reset requires finite angles and bounded release");
    membership_ = active;
    initial_release_seconds_ = initial_release_seconds;
    elapsed_ticks_ = 0;
    release_finished_ = !active || initial_release_seconds == 0.0;
    reset_fsm_(q.cast<float>(), active && release_finished_);
    publish_();
}

V6RecoveryController::Vector4f V6RecoveryController::paired_delta_(
    const Vector4f& goal, const Vector4f& reference) {
    Vector4f difference = goal - reference;
    for (int side = 0; side < 2; ++side) {
        // torch.round uses ties-to-even. nearbyint has the same default rounding.
        const float turns = std::nearbyint(difference[2 * side] / kTau);
        difference.segment<2>(2 * side).array() -= turns * kTau;
    }
    return difference;
}

Eigen::Vector4d V6RecoveryController::paired_delta(const Eigen::Vector4d& goal,
                                                  const Eigen::Vector4d& reference) {
    return paired_delta_(goal.cast<float>(), reference.cast<float>()).cast<double>();
}

void V6RecoveryController::enter_(V6RecoveryPhase phase) {
    phase_ = phase;
    phase_ticks_ = 0;
    if (phase == V6RecoveryPhase::kRl && config_.release_motion_on_takeover)
        motion_released_ = true;
}

void V6RecoveryController::fail_(int code) {
    if (!enabled_ || phase_ == V6RecoveryPhase::kFailed)
        return;
    enter_(V6RecoveryPhase::kFailed);
    failure_code_ = code;
    motion_released_ = false;
}

void V6RecoveryController::publish_() {
    command_.targets = targets_.cast<double>();
    command_.continuous_q = continuous_q_.cast<double>();
    command_.phase = phase_;
    command_.route = route_;
    command_.failed = phase_ == V6RecoveryPhase::kFailed;
    command_.release_finished = release_finished_;
    command_.pure_rl = (!membership_ || release_finished_) && phase_ == V6RecoveryPhase::kRl;
    command_.blending = enabled_ && phase_ == V6RecoveryPhase::kBlend;
    command_.scripted = enabled_ && phase_ < V6RecoveryPhase::kBlend;
    command_.blend = command_.pure_rl ? 1.0 : command_.blending && config_.blend_seconds > 0.0
        ? static_cast<double>(std::clamp(static_cast<float>(phase_ticks_)
              * static_cast<float>(config_.dt) / static_cast<float>(config_.blend_seconds), 0.0f, 1.0f))
        : 0.0;
    command_.motion_released = !membership_ || (release_finished_ && motion_released_);
    command_.motion_hold = membership_ && !command_.motion_released;
    command_.wheel_balance_active = wheel_balance_active_;
    command_.failure_code = failure_code_;
    command_.age_ticks = age_ticks_;
    command_.phase_ticks = phase_ticks_;
    command_.stable_ticks = stable_ticks_;
    command_.reroute_count = reroute_count_;
}

const V6RecoveryCommand& V6RecoveryController::update(const V6RecoveryFeedback& feedback) {
    const bool valid = feedback.q.allFinite() && feedback.dq.allFinite()
        && feedback.gravity.allFinite() && feedback.gyro.allFinite()
        && std::isfinite(feedback.estimated_height) && feedback.gravity.norm() > 0.5;
    if (!valid) {
        // Invalid sensing also terminates a zero-effort release episode.
        enabled_ = membership_;
        fail_(1);
        publish_();
        return command_;
    }
    const Vector6f raw_q = feedback.q.cast<float>();
    canonical_q_ = raw_q;
    for (int i = 0; i < 6; ++i)
        continuous_q_[i] += wrapped(raw_q[i] - last_raw_q_[i]);
    last_raw_q_ = raw_q;
    if (membership_ && !release_finished_
        && static_cast<double>(elapsed_ticks_) * config_.dt >= initial_release_seconds_) {
        // Accumulated encoder winding survives passive initial release.
        reset_fsm_(continuous_q_, true);
        release_finished_ = true;
    }
    if (membership_ && !release_finished_) {
        ++elapsed_ticks_;
        publish_();
        return command_;
    }
    Eigen::Vector3f gravity = feedback.gravity.cast<float>();
    gravity /= std::max(gravity.norm(), 1e-6f);
    const Eigen::Vector3f gyro = feedback.gyro.cast<float>();
    const Vector6f dq = feedback.dq.cast<float>();
    const float height = static_cast<float>(feedback.estimated_height);
    const float tilt = std::acos(std::clamp(-gravity.z(), -1.0f, 1.0f));
    const float pitch = std::atan2(gravity.x(), -gravity.z());
    continuous_pitch_ = pitch_initialized_ ? continuous_pitch_ + wrapped(pitch - last_pitch_) : pitch;
    last_pitch_ = pitch;
    pitch_initialized_ = true;
    if (enabled_ && phase_ != V6RecoveryPhase::kFailed && phase_ != V6RecoveryPhase::kRl)
        ++age_ticks_;
    if (enabled_ && phase_ != V6RecoveryPhase::kFailed)
        ++phase_ticks_;

    const bool recapturable = phase_ == V6RecoveryPhase::kPrepare || phase_ == V6RecoveryPhase::kCapture;
    const bool unsupported = enabled_ && recapturable && !feedback.support
        && tilt > static_cast<float>(config_.reroute_tilt_rad);
    unsupported_ticks_ = unsupported ? unsupported_ticks_ + 1 : 0;
    if (unsupported && unsupported_ticks_ >= ticks_(config_.reroute_support_lost_seconds)
        && reroute_count_ < config_.max_reroutes) {
        ++reroute_count_;
        unsupported_ticks_ = 0;
        enter_(V6RecoveryPhase::kSelect);
    }
    const bool near_upright = tilt < radians(15.0) && height > 0.20f;
    V6RecoveryRoute selected = tilt < radians(70.0) ? V6RecoveryRoute::kPlant
        : pitch >= 0.0f ? V6RecoveryRoute::kOrbitPositive : V6RecoveryRoute::kOrbitNegative;
    if (tilt >= radians(70.0) && std::abs(gravity.y()) > 0.7f)
        selected = V6RecoveryRoute::kSide;
    if (near_upright)
        selected = V6RecoveryRoute::kUpright;
    if (phase_ == V6RecoveryPhase::kSelect) {
        route_ = selected;
        enter_(near_upright ? V6RecoveryPhase::kPrepare : V6RecoveryPhase::kFold);
    }
    // Every transition below uses the same pre-transition snapshot, matching
    // the batched native masks. A newly entered phase never advances twice.
    const auto phase = phase_;
    const float elapsed = static_cast<float>(phase_ticks_) * static_cast<float>(config_.dt);
    const bool folding = phase == V6RecoveryPhase::kFold;
    const bool planting = phase == V6RecoveryPhase::kPlant;
    const bool orbiting = phase == V6RecoveryPhase::kOrbit;
    const bool thrusting = phase == V6RecoveryPhase::kThrust;
    const bool side = phase == V6RecoveryPhase::kSide;
    const bool preparing = phase == V6RecoveryPhase::kPrepare;
    const bool capturing = phase == V6RecoveryPhase::kCapture;
    const bool blending = phase == V6RecoveryPhase::kBlend;
    const Vector4f axes = config_.root_axis_signs.cast<float>();
    Vector4f goal = config_.nominal.cast<float>();
    if (folding)
        goal = config_.fold.cast<float>();
    if ((preparing || capturing || blending) && feedback.support && tilt < radians(85.0)) {
        const float extension = std::clamp((tilt - radians(12.0)) / radians(43.0), 0.0f, 1.0f);
        goal = config_.nominal.cast<float>()
            + extension * (config_.support - config_.nominal).cast<float>()
            - pitch * axes.cwiseInverse();
    }
    if (planting) {
        goal = config_.fold.cast<float>();
        if (std::abs(gravity.y()) < 0.5f) {
            const float sign = pitch > 0.0f ? 1.0f : pitch < 0.0f ? -1.0f : 0.0f;
            const float offset = std::clamp(-pitch - static_cast<float>(config_.plant_angle_rad) * sign,
                                            -2.2f, 2.2f);
            goal += offset * axes.cwiseInverse();
        }
    }
    if (orbiting) {
        orbit_angle_ += static_cast<float>(config_.orbit_speed_rad_s) * static_cast<float>(config_.dt);
        const float direction = route_ == V6RecoveryRoute::kOrbitPositive ? 1.0f : -1.0f;
        goal = orbit_start_ + direction * orbit_angle_ * axes.cwiseInverse();
    }
    if (thrusting)
        goal = thrust_anchor_ - (continuous_pitch_ - plant_pitch_) * axes.cwiseInverse();
    const float side_duration = static_cast<float>(config_.side_angle_rad / config_.side_speed_rad_s);
    if (side) {
        float stroke = elapsed <= side_duration
            ? elapsed * static_cast<float>(config_.side_speed_rad_s)
            : static_cast<float>(config_.side_angle_rad)
                - std::max(elapsed - side_duration - 0.08f, 0.0f) * static_cast<float>(config_.side_speed_rad_s);
        stroke = std::clamp(stroke, 0.0f, static_cast<float>(config_.side_angle_rad));
        goal = side_start_ + side_direction_.cwiseProduct(axes.cwiseInverse()) * stroke;
    }
    Vector4f reference = targets_.head<4>();
    const Vector4f delta = orbiting || thrusting || side ? Vector4f(goal - reference)
                                                       : paired_delta_(goal, reference);
    float speed = static_cast<float>(config_.prepare_speed_rad_s);
    if (preparing || thrusting) speed = static_cast<float>(config_.push_speed_rad_s);
    if (capturing) speed = static_cast<float>(config_.capture_speed_rad_s);
    if (orbiting) speed = static_cast<float>(config_.orbit_speed_rad_s);
    if (side) speed = static_cast<float>(config_.side_speed_rad_s);
    const float fraction = std::min(speed * static_cast<float>(config_.dt)
                                     / std::max(delta.cwiseAbs().maxCoeff(), 1e-6f), 1.0f);
    if ((enabled_ && phase_ < V6RecoveryPhase::kBlend) || (enabled_ && blending))
        reference += delta * fraction;
    targets_.head<4>() = reference;
    const float remaining = delta.cwiseAbs().maxCoeff() * (1.0f - fraction);
    const float measured_error = paired_delta_(reference, continuous_q_.head<4>()).cwiseAbs().maxCoeff();
    if (folding && remaining < 0.02f && measured_error < 0.2f && elapsed >= 0.3f) {
        if (route_ == V6RecoveryRoute::kOrbitPositive || route_ == V6RecoveryRoute::kOrbitNegative) {
            orbit_start_ = reference;
            orbit_angle_ = 0.0f;
            enter_(V6RecoveryPhase::kOrbit);
        } else if (route_ == V6RecoveryRoute::kSide) {
            side_start_ = reference;
            const float direction = gravity.y() >= 0.0f ? -1.0f : 1.0f;
            side_direction_ << direction, direction, -direction, -direction;
            enter_(V6RecoveryPhase::kSide);
        } else {
            enter_(V6RecoveryPhase::kPlant);
        }
    }
    const bool converted = side && elapsed > 0.05f && std::abs(gravity.y()) < 0.65f;
    if (converted) {
        route_ = selected;
        enter_(near_upright ? V6RecoveryPhase::kPrepare : V6RecoveryPhase::kFold);
    }
    if (side && !converted && elapsed >= static_cast<float>(2.0 * config_.side_angle_rad / config_.side_speed_rad_s + 0.4))
        fail_(2);
    plant_ticks_ = planting && feedback.support ? plant_ticks_ + 1 : 0;
    const bool planted = planting && remaining < 0.15f && measured_error < 0.2f && plant_ticks_ >= ticks_(0.06);
    if (planted) enter_(V6RecoveryPhase::kPrepare);
    if (planting && !planted && elapsed >= 2.5f) fail_(3);
    const bool orbit_contact = orbiting && feedback.support && tilt < radians(145.0)
        && height > 0.12f && orbit_angle_ > 3.7f;
    if (orbit_contact) {
        thrust_anchor_ = reference + paired_delta_(config_.thrust.cast<float>(), reference);
        plant_pitch_ = continuous_pitch_;
        enter_(V6RecoveryPhase::kThrust);
    }
    const bool captured = (orbiting && tilt < radians(30.0) && height > 0.22f && feedback.support)
        || (thrusting && tilt < radians(65.0) && height > 0.24f && gyro.norm() < 8.0f && feedback.support);
    if (captured) enter_(V6RecoveryPhase::kCapture);
    if (orbiting && !orbit_contact && !captured
        && orbit_angle_ >= static_cast<float>(2.0 * std::numbers::pi * config_.orbit_turns))
        fail_(4);
    if (capturing && tilt < radians(20.0) && feedback.support && height > 0.22f)
        enter_(V6RecoveryPhase::kPrepare);
    const bool stable = feedback.support && feedback.body_clear
        && tilt < static_cast<float>(config_.handover_tilt_rad)
        && gyro.norm() < static_cast<float>(config_.handover_gyro_max)
        && height > static_cast<float>(config_.handover_height_min)
        && height < static_cast<float>(config_.handover_height_max)
        && dq.tail<2>().cwiseAbs().maxCoeff() * 0.06f < 0.25f
        && dq.head<4>().cwiseAbs().maxCoeff() < 2.0f;
    const bool ready = preparing && stable && remaining < 0.02f && measured_error < 0.12f;
    ready_ticks_ = ready ? ready_ticks_ + 1 : 0;
    // A valid measured capture ends the current script, even before PREPARE.
    // Use the current phase so an earlier failure cannot be cleared here.
    const bool captured_for_rl = enabled_ && phase_ < V6RecoveryPhase::kBlend
        && config_.dynamic_takeover
        && feedback.rl_capture_ready && feedback.support && feedback.support_confirmed
        && height > static_cast<float>(config_.dynamic_capture_height_min)
        && height < static_cast<float>(config_.handover_height_max);
    if (captured_for_rl || (preparing && ready_ticks_ >= ticks_(0.1)))
        enter_(config_.blend_seconds == 0.0 ? V6RecoveryPhase::kRl : V6RecoveryPhase::kBlend);
    if (blending && phase_ticks_ >= ticks_(config_.blend_seconds)) enter_(V6RecoveryPhase::kRl);
    const bool holding_rl = enabled_ && phase == V6RecoveryPhase::kRl && !motion_released_;
    stable_ticks_ = holding_rl && stable ? stable_ticks_ + 1 : 0;
    motion_released_ = motion_released_ || (holding_rl && stable_ticks_ >= ticks_(config_.stable_seconds));
    if (enabled_ && (phase_ == V6RecoveryPhase::kBlend || (phase_ == V6RecoveryPhase::kRl && !motion_released_))
        && tilt > radians(45.0)) fail_(5);
    if (enabled_ && phase_ != V6RecoveryPhase::kRl && age_ticks_ >= ticks_(config_.max_script_seconds)) fail_(6);
    if (membership_ && phase_ != V6RecoveryPhase::kRl
        && static_cast<double>(elapsed_ticks_) * config_.dt >= config_.max_script_seconds) fail_(6);

    // RecoveryTraining updates wheels after the FSM transition. Entering BLEND
    // recomputes once; subsequent BLEND ticks retain that velocity reference.
    const bool is_capture = phase_ == V6RecoveryPhase::kCapture;
    const bool is_prepare = phase_ == V6RecoveryPhase::kPrepare;
    const bool is_blend = enabled_ && phase_ == V6RecoveryPhase::kBlend;
    const float balance_limit = radians(is_capture ? config_.capture_tilt_limit_deg : config_.prepare_tilt_limit_deg);
    const bool eligible = enabled_ && release_finished_ && feedback.support && tilt < balance_limit
        && (is_prepare || is_capture || (is_blend && phase == V6RecoveryPhase::kPrepare))
        && config_.wheel_balance_enabled;
    const bool retained = is_blend && wheel_balance_active_ && feedback.support && tilt < balance_limit;
    Eigen::Vector2f desired = retained ? Eigen::Vector2f(targets_.tail<2>()) : Eigen::Vector2f::Zero();
    if (eligible) {
        const float effort = static_cast<float>(config_.pitch_kp_nm_rad) * pitch
                           + static_cast<float>(config_.pitch_kd_nm_s_rad) * gyro.y();
        desired = effort * config_.wheel_axis_signs.cast<float>() / 0.6f
                + (1.0f - static_cast<float>(config_.wheel_damping_nm_s_rad) / 0.6f) * dq.tail<2>();
    }
    wheel_balance_active_ = eligible || retained;
    targets_.tail<2>() = desired;
    ++elapsed_ticks_;
    publish_();
    return command_;
}

V6RecoveryVector6 V6RecoveryController::torques(const V6RecoveryFeedback& feedback,
                                                const V6RecoveryVector6& rl_torque) const {
    if (!membership_)
        return rl_torque;
    if (!release_finished_ || command_.failed || !feedback.q.allFinite()
        || !feedback.dq.allFinite() || !feedback.wheel_probe_torque.allFinite() || !rl_torque.allFinite())
        return V6RecoveryVector6::Zero();
    const Vector6f q = project_feedback(feedback.q).cast<float>();
    const Vector6f dq = feedback.dq.cast<float>();
    Vector6f script;
    script.head<4>() = 160.0f * (targets_.head<4>() - q.head<4>()) - 2.5f * dq.head<4>();
    script.tail<2>() = 0.6f * (targets_.tail<2>() - dq.tail<2>())
                    + feedback.wheel_probe_torque.cast<float>();
    script.head<4>() = script.head<4>().cwiseMax(-40.0f).cwiseMin(40.0f);
    script.tail<2>() = script.tail<2>().cwiseMax(-4.5f).cwiseMin(4.5f);
    Vector6f actor = rl_torque.cast<float>();
    actor.head<4>() = actor.head<4>().cwiseMax(-40.0f).cwiseMin(40.0f);
    actor.tail<2>() = actor.tail<2>().cwiseMax(-4.5f).cwiseMin(4.5f);
    const float alpha = static_cast<float>(command_.blend);
    return ((1.0f - alpha) * script + alpha * actor).cast<double>();
}

V6RecoveryVector6 V6RecoveryController::project_feedback(const V6RecoveryVector6& input) const {
    const Vector6f raw_q = input.cast<float>();
    Vector6f q;
    for (int i = 0; i < 6; ++i) {
        const float turns = std::nearbyint((canonical_q_[i] - raw_q[i]) / kTau);
        q[i] = turns == 0.0f ? raw_q[i] : raw_q[i] + turns * kTau;
    }
    return q.cast<double>();
}

V6RecoveryVector6 V6RecoveryController::effective_action_history(const V6RecoveryVector6& rl_action) const {
    if (!membership_ || command_.pure_rl)
        return rl_action;
    if (!release_finished_ || command_.failed)
        return V6RecoveryVector6::Zero();
    Vector6f script = Vector6f::Zero();
    const Vector4f delta = targets_.head<4>() - config_.rl_nominal.head<4>().cast<float>();
    for (int i = 0; i < 4; ++i)
        script[i] = wrapped(delta[i]) / 0.25f;
    script.tail<2>() = targets_.tail<2>() / 10.0f;
    const float alpha = static_cast<float>(command_.blend);
    return ((1.0f - alpha) * script + alpha * rl_action.cast<float>()).cast<double>();
}

} // namespace rmcs::rl
