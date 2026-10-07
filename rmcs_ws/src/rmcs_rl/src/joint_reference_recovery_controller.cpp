#include "joint_reference_recovery_controller.hpp"

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

std::string_view recovery_phase_name(JointReferenceRecoveryPhase phase) {
    constexpr std::array<std::string_view, 11> names{"SELECT", "FOLD", "PLANT",   "ORBIT",
                                                     "THRUST", "SIDE", "CAPTURE", "PREPARE",
                                                     "BLEND",  "RL",   "FAILED"};
    const auto index = static_cast<std::size_t>(phase);
    return index < names.size() ? names[index] : "UNKNOWN";
}

JointReferenceRecoveryController::JointReferenceRecoveryController(
    const JointReferenceRecoveryConfig& config)
    : config_(config) {
    const std::array values{
        config.dt,
        config.dynamic_capture_height_min,
        config.stable_seconds,
        config.max_script_seconds,
        config.prepare_speed_rad_s,
        config.push_speed_rad_s,
        config.orbit_speed_rad_s,
        config.capture_speed_rad_s,
        config.side_speed_rad_s,
        config.side_angle_rad,
        config.plant_angle_rad,
        config.orbit_turns,
        config.handover_height_min,
        config.handover_height_max,
        config.handover_tilt_rad,
        config.handover_gyro_max,
        config.reroute_tilt_rad,
        config.reroute_support_lost_seconds,
        config.pitch_kp_nm_rad,
        config.pitch_kd_nm_s_rad,
        config.wheel_damping_nm_s_rad,
        config.prepare_tilt_limit_deg,
        config.capture_tilt_limit_deg};
    if (!std::all_of(values.begin(), values.end(), positive_finite)
        || (config.dt != 0.005 && config.dt != 0.001) || !std::isfinite(config.blend_seconds)
        || (config.blend_seconds != 0.0 && config.blend_seconds != 0.2)
        || (config.dynamic_takeover && config.blend_seconds != 0.0)
        || config.dynamic_capture_height_min < 0.20
        || config.dynamic_capture_height_min > config.handover_height_min
        || config.stable_seconds < 1.0 || config.max_script_seconds > 8.0 || config.max_reroutes < 0
        || config.max_reroutes > 1 || config.handover_height_min >= config.handover_height_max
        || config.handover_tilt_rad >= config.reroute_tilt_rad
        || config.reroute_tilt_rad >= std::numbers::pi
        || config.prepare_tilt_limit_deg > config.capture_tilt_limit_deg
        || config.capture_tilt_limit_deg >= 90.0 || !config.nominal.allFinite()
        || !config.fold.allFinite() || !config.thrust.allFinite() || !config.support.allFinite()
        || !config.rl_nominal.allFinite() || !config.root_axis_signs.allFinite()
        || !config.wheel_axis_signs.allFinite())
        throw std::invalid_argument("Invalid Joint reference recovery configuration");
    for (int i = 0; i < 4; ++i)
        if (std::abs(config.root_axis_signs[i]) != 1.0
            || config.root_axis_signs[i] != config.root_axis_signs[i ^ 1])
            throw std::invalid_argument(
                "Joint reference recovery roots require signed coupled pairs");
    for (int i = 0; i < 2; ++i)
        if (std::abs(config.wheel_axis_signs[i]) != 1.0)
            throw std::invalid_argument(
                "Joint reference recovery requires signed wheel motor axes");
    publish_();
}

std::uint64_t JointReferenceRecoveryController::ticks_(double seconds) const {
    return static_cast<std::uint64_t>(std::ceil(seconds / config_.dt - 1e-9));
}

bool JointReferenceRecoveryController::reached_(
    std::uint64_t ticks, double elapsed, double seconds) const {
    return config_.dt == 0.005 ? ticks >= ticks_(seconds) : elapsed + 1e-12 >= seconds;
}

bool JointReferenceRecoveryController::phase_reached_(
    const StepContext& step, double seconds) const {
    return config_.dt == 0.005 ? step.elapsed >= static_cast<float>(seconds)
                               : step.elapsed_seconds + 1e-12 >= seconds;
}

float JointReferenceRecoveryController::phase_elapsed_() const {
    return config_.dt == 0.005 ? static_cast<float>(phase_ticks_) * static_cast<float>(config_.dt)
                               : static_cast<float>(durations_.phase);
}

void JointReferenceRecoveryController::reset_fsm_(const Vector6f& q, bool active) {
    enabled_ = active;
    phase_ = active ? JointReferenceRecoveryPhase::kSelect : JointReferenceRecoveryPhase::kRl;
    motion_released_ = !active;
    targets_ = continuous_q_ = canonical_q_ = last_raw_q_ = q;
    targets_.tail<2>().setZero();
    route_ = JointReferenceRecoveryRoute::kUpright;
    age_ticks_ = phase_ticks_ = ready_ticks_ = stable_ticks_ = plant_ticks_ = unsupported_ticks_ =
        0;
    durations_ = {};
    reroute_count_ = failure_code_ = 0;
    continuous_pitch_ = last_pitch_ = orbit_angle_ = plant_pitch_ = 0.0f;
    pitch_initialized_ = wheel_balance_active_ = false;
    orbit_start_.setZero();
    thrust_anchor_.setZero();
    side_start_.setZero();
    side_direction_.setZero();
    side_lower_leg_ = 0;
}

void JointReferenceRecoveryController::reset(
    const JointReferenceRecoveryVector6& q, bool active, double initial_release_seconds) {
    if (!q.allFinite() || !std::isfinite(initial_release_seconds) || initial_release_seconds < 0.0
        || initial_release_seconds >= config_.max_script_seconds)
        throw std::invalid_argument(
            "Joint reference recovery reset requires finite angles and bounded release");
    membership_ = active;
    initial_release_seconds_ = initial_release_seconds;
    elapsed_ticks_ = 0;
    elapsed_seconds_ = 0.0;
    release_finished_ = !active || initial_release_seconds == 0.0;
    reset_fsm_(q.cast<float>(), active && release_finished_);
    publish_();
}

JointReferenceRecoveryController::Vector4f JointReferenceRecoveryController::paired_delta_(
    const Vector4f& goal, const Vector4f& reference) {
    Vector4f difference = goal - reference;
    for (int side = 0; side < 2; ++side) {
        // torch.round uses ties-to-even. nearbyint has the same default rounding.
        const float turns = std::nearbyint(difference[2 * side] / kTau);
        difference.segment<2>(2 * side).array() -= turns * kTau;
    }
    return difference;
}

Eigen::Vector4d JointReferenceRecoveryController::paired_delta(
    const Eigen::Vector4d& goal, const Eigen::Vector4d& reference) {
    return paired_delta_(goal.cast<float>(), reference.cast<float>()).cast<double>();
}

void JointReferenceRecoveryController::enter_(JointReferenceRecoveryPhase phase) {
    phase_ = phase;
    phase_ticks_ = 0;
    durations_.phase = 0.0;
    if (phase == JointReferenceRecoveryPhase::kRl && config_.release_motion_on_takeover)
        motion_released_ = true;
}

void JointReferenceRecoveryController::fail_(int code) {
    if (!enabled_ || phase_ == JointReferenceRecoveryPhase::kFailed)
        return;
    enter_(JointReferenceRecoveryPhase::kFailed);
    failure_code_ = code;
    motion_released_ = false;
}

void JointReferenceRecoveryController::publish_() {
    command_.targets = targets_.cast<double>();
    command_.continuous_q = continuous_q_.cast<double>();
    command_.phase = phase_;
    command_.route = route_;
    command_.failed = phase_ == JointReferenceRecoveryPhase::kFailed;
    command_.release_finished = release_finished_;
    command_.pure_rl =
        (!membership_ || release_finished_) && phase_ == JointReferenceRecoveryPhase::kRl;
    command_.blending = enabled_ && phase_ == JointReferenceRecoveryPhase::kBlend;
    command_.scripted = enabled_ && phase_ < JointReferenceRecoveryPhase::kBlend;
    command_.blend =
        command_.pure_rl ? 1.0
        : command_.blending && config_.blend_seconds > 0.0
            ? static_cast<double>(std::clamp(
                  phase_elapsed_() / static_cast<float>(config_.blend_seconds), 0.0f, 1.0f))
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

const JointReferenceRecoveryCommand&
    JointReferenceRecoveryController::update(const JointReferenceRecoveryFeedback& feedback) {
    return update(feedback, config_.dt);
}

const JointReferenceRecoveryCommand& JointReferenceRecoveryController::update(
    const JointReferenceRecoveryFeedback& feedback, double elapsed_seconds) {
    const bool valid = std::isfinite(elapsed_seconds) && elapsed_seconds > 0.0
                    && elapsed_seconds <= 0.02 && feedback.q.allFinite() && feedback.dq.allFinite()
                    && feedback.gravity.allFinite() && feedback.gyro.allFinite()
                    && std::isfinite(feedback.estimated_height) && feedback.gravity.norm() > 0.5;
    if (!valid) {
        // Invalid sensing also terminates a zero-effort release episode.
        enabled_ = membership_;
        fail_(1);
        publish_();
        return command_;
    }
    double dt = config_.dt == 0.005 ? config_.dt : elapsed_seconds;
    elapsed_seconds_ += dt;
    const Vector6f raw_q = feedback.q.cast<float>();
    canonical_q_ = raw_q;
    for (int i = 0; i < 6; ++i)
        continuous_q_[i] += wrapped(raw_q[i] - last_raw_q_[i]);
    last_raw_q_ = raw_q;
    if (membership_ && !release_finished_
        && (config_.dt == 0.005 ? static_cast<double>(elapsed_ticks_) * config_.dt
                                : elapsed_seconds_ + 1e-12)
               >= initial_release_seconds_) {
        if (config_.dt == 0.001) {
            // The portion spent releasing cannot also advance the active script.
            // Preserve route selection in this update, even at an exact boundary.
            dt = std::clamp(elapsed_seconds_ - initial_release_seconds_, 0.0, dt);
            if (dt <= 1e-12)
                dt = 0.0;
        }
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
    continuous_pitch_ =
        pitch_initialized_ ? continuous_pitch_ + wrapped(pitch - last_pitch_) : pitch;
    last_pitch_ = pitch;
    pitch_initialized_ = true;
    if (enabled_ && phase_ != JointReferenceRecoveryPhase::kFailed
        && phase_ != JointReferenceRecoveryPhase::kRl) {
        ++age_ticks_;
        durations_.age += dt;
    }
    if (enabled_ && phase_ != JointReferenceRecoveryPhase::kFailed) {
        ++phase_ticks_;
        durations_.phase += dt;
    }

    const bool recapturable = phase_ == JointReferenceRecoveryPhase::kPrepare
                           || phase_ == JointReferenceRecoveryPhase::kCapture;
    const bool unsupported = enabled_ && recapturable && !feedback.support
                          && tilt > static_cast<float>(config_.reroute_tilt_rad);
    unsupported_ticks_ = unsupported ? unsupported_ticks_ + 1 : 0;
    durations_.unsupported = unsupported ? durations_.unsupported + dt : 0.0;
    if (unsupported
        && reached_(
            unsupported_ticks_, durations_.unsupported, config_.reroute_support_lost_seconds)) {
        if (route_ == JointReferenceRecoveryRoute::kSide) {
            // A failed side capture must not restart a full sagittal revolution.
            fail_(2);
        } else if (reroute_count_ < config_.max_reroutes) {
            ++reroute_count_;
            unsupported_ticks_ = 0;
            durations_.unsupported = 0.0;
            enter_(JointReferenceRecoveryPhase::kSelect);
        }
    }
    const bool near_upright = tilt < radians(15.0) && height > 0.20f;
    JointReferenceRecoveryRoute selected =
        tilt < radians(70.0) ? JointReferenceRecoveryRoute::kPlant
        : pitch >= 0.0f      ? JointReferenceRecoveryRoute::kOrbitPositive
                             : JointReferenceRecoveryRoute::kOrbitNegative;
    if (tilt >= radians(70.0) && std::abs(gravity.y()) > 0.7f)
        selected = JointReferenceRecoveryRoute::kSide;
    if (near_upright)
        selected = JointReferenceRecoveryRoute::kUpright;
    if (phase_ == JointReferenceRecoveryPhase::kSelect) {
        route_ = selected;
        if (route_ == JointReferenceRecoveryRoute::kSide)
            side_lower_leg_ = gravity.y() >= 0.0f ? 0 : 1;
        enter_(
            near_upright ? JointReferenceRecoveryPhase::kPrepare
                         : JointReferenceRecoveryPhase::kFold);
    }
    // Transition predicates share one snapshot. A newly entered phase cannot
    // consume another trajectory step; wheel balancing uses the resulting phase.
    const StepContext step{
        phase_,
        selected,
        gravity,
        gyro,
        dq,
        height,
        tilt,
        pitch,
        static_cast<float>(dt),
        phase_elapsed_(),
        durations_.phase,
        near_upright};
    const auto goal = update_goal_(step, feedback.support);
    const auto reference = advance_reference_(step, goal);
    update_transitions_(step, reference, feedback, dt);
    update_wheels_(step, feedback.support);
    ++elapsed_ticks_;
    publish_();
    return command_;
}

JointReferenceRecoveryController::JointGoal
    JointReferenceRecoveryController::update_goal_(const StepContext& step, bool supported) {
    const auto phase = step.phase;
    const auto& gravity = step.gravity;
    const float tilt = step.tilt;
    const float pitch = step.pitch;
    const float elapsed = step.elapsed;
    const bool folding = phase == JointReferenceRecoveryPhase::kFold;
    const bool planting = phase == JointReferenceRecoveryPhase::kPlant;
    const bool orbiting = phase == JointReferenceRecoveryPhase::kOrbit;
    const bool thrusting = phase == JointReferenceRecoveryPhase::kThrust;
    const bool side = phase == JointReferenceRecoveryPhase::kSide;
    const bool preparing = phase == JointReferenceRecoveryPhase::kPrepare;
    const bool capturing = phase == JointReferenceRecoveryPhase::kCapture;
    const bool blending = phase == JointReferenceRecoveryPhase::kBlend;
    const Vector4f axes = config_.root_axis_signs.cast<float>();
    Vector4f goal = config_.nominal.cast<float>();
    if (folding) {
        goal = config_.fold.cast<float>();
        if (route_ == JointReferenceRecoveryRoute::kSide) {
            // Lengthen the upper leg before the grounded leg provides the roll impulse.
            const int upper = 1 - side_lower_leg_;
            goal.segment<2>(2 * upper) = config_.thrust.segment<2>(2 * upper).cast<float>();
        }
    }
    if ((preparing || capturing || blending) && supported && tilt < radians(85.0)) {
        const float extension = std::clamp((tilt - radians(12.0)) / radians(43.0), 0.0f, 1.0f);
        goal = config_.nominal.cast<float>()
             + extension * (config_.support - config_.nominal).cast<float>()
             - pitch * axes.cwiseInverse();
    }
    if (planting) {
        goal = config_.fold.cast<float>();
        if (std::abs(gravity.y()) < 0.5f) {
            const float sign = pitch > 0.0f ? 1.0f : pitch < 0.0f ? -1.0f : 0.0f;
            const float offset = std::clamp(
                -pitch - static_cast<float>(config_.plant_angle_rad) * sign, -2.2f, 2.2f);
            goal += offset * axes.cwiseInverse();
        }
    }
    if (orbiting) {
        orbit_angle_ += static_cast<float>(config_.orbit_speed_rad_s) * step.dt;
        const float direction =
            route_ == JointReferenceRecoveryRoute::kOrbitPositive ? 1.0f : -1.0f;
        goal = orbit_start_ + direction * orbit_angle_ * axes.cwiseInverse();
    }
    if (thrusting)
        goal = thrust_anchor_ - (continuous_pitch_ - plant_pitch_) * axes.cwiseInverse();
    if (side) {
        // A single grounded-leg stroke; returning it here rolls the body onto its back.
        const float stroke = std::min(
            elapsed * static_cast<float>(config_.side_speed_rad_s),
            static_cast<float>(config_.side_angle_rad));
        goal = side_start_ + side_direction_.cwiseProduct(axes.cwiseInverse()) * stroke;
    }
    float speed = static_cast<float>(config_.prepare_speed_rad_s);
    if (preparing || thrusting)
        speed = static_cast<float>(config_.push_speed_rad_s);
    if (capturing)
        speed = static_cast<float>(config_.capture_speed_rad_s);
    if (orbiting)
        speed = static_cast<float>(config_.orbit_speed_rad_s);
    if (side)
        speed = static_cast<float>(config_.side_speed_rad_s);
    return {
        goal, speed,
        orbiting || thrusting || side ? Winding::kContinuous : Winding::kCoupledNearest};
}

JointReferenceRecoveryController::ReferenceStep
    JointReferenceRecoveryController::advance_reference_(
        const StepContext& step, const JointGoal& goal) {
    Vector4f reference = targets_.head<4>();
    const Vector4f delta = goal.winding == Winding::kContinuous
                             ? Vector4f(goal.position - reference)
                             : paired_delta_(goal.position, reference);
    const float fraction =
        std::min(goal.speed * step.dt / std::max(delta.cwiseAbs().maxCoeff(), 1e-6f), 1.0f);
    if ((enabled_ && phase_ < JointReferenceRecoveryPhase::kBlend)
        || (enabled_ && step.phase == JointReferenceRecoveryPhase::kBlend))
        reference += delta * fraction;
    targets_.head<4>() = reference;
    const float remaining = delta.cwiseAbs().maxCoeff() * (1.0f - fraction);
    const float measured_error =
        paired_delta_(reference, continuous_q_.head<4>()).cwiseAbs().maxCoeff();
    return {reference, remaining, measured_error};
}

void JointReferenceRecoveryController::update_transitions_(
    const StepContext& step, const ReferenceStep& reference_step,
    const JointReferenceRecoveryFeedback& feedback, double dt) {
    const auto phase = step.phase;
    const auto& gyro = step.gyro;
    const auto& dq = step.dq;
    const float height = step.height;
    const float tilt = step.tilt;
    const auto& reference = reference_step.position;
    const float remaining = reference_step.remaining;
    const float measured_error = reference_step.measured_error;
    const bool folding = phase == JointReferenceRecoveryPhase::kFold;
    const bool planting = phase == JointReferenceRecoveryPhase::kPlant;
    const bool orbiting = phase == JointReferenceRecoveryPhase::kOrbit;
    const bool thrusting = phase == JointReferenceRecoveryPhase::kThrust;
    const bool side = phase == JointReferenceRecoveryPhase::kSide;
    const bool preparing = phase == JointReferenceRecoveryPhase::kPrepare;
    const bool capturing = phase == JointReferenceRecoveryPhase::kCapture;
    const bool blending = phase == JointReferenceRecoveryPhase::kBlend;
    if (folding && remaining < 0.02f && measured_error < 0.2f && phase_reached_(step, 0.3)) {
        if (route_ == JointReferenceRecoveryRoute::kOrbitPositive
            || route_ == JointReferenceRecoveryRoute::kOrbitNegative) {
            orbit_start_ = reference;
            orbit_angle_ = 0.0f;
            enter_(JointReferenceRecoveryPhase::kOrbit);
        } else if (route_ == JointReferenceRecoveryRoute::kSide) {
            side_start_ = reference;
            side_direction_.setZero();
            side_direction_.segment<2>(2 * side_lower_leg_).setOnes();
            enter_(JointReferenceRecoveryPhase::kSide);
        } else {
            enter_(JointReferenceRecoveryPhase::kPlant);
        }
    }
    const bool side_upright = side && tilt < radians(65.0);
    if (side_upright)
        enter_(JointReferenceRecoveryPhase::kCapture);
    if (side && !side_upright && phase_reached_(step, 2.5))
        fail_(2);
    plant_ticks_ = planting && feedback.support ? plant_ticks_ + 1 : 0;
    durations_.plant = planting && feedback.support ? durations_.plant + dt : 0.0;
    const bool planted = planting && remaining < 0.15f && measured_error < 0.2f
                      && reached_(plant_ticks_, durations_.plant, 0.06);
    if (planted)
        enter_(JointReferenceRecoveryPhase::kPrepare);
    if (planting && !planted && phase_reached_(step, 2.5))
        fail_(3);
    const bool orbit_contact = orbiting && feedback.support && tilt < radians(145.0)
                            && height > 0.12f && orbit_angle_ > 3.7f;
    if (orbit_contact) {
        thrust_anchor_ = reference + paired_delta_(config_.thrust.cast<float>(), reference);
        plant_pitch_ = continuous_pitch_;
        enter_(JointReferenceRecoveryPhase::kThrust);
    }
    const bool captured = (orbiting && tilt < radians(30.0) && height > 0.22f && feedback.support)
                       || (thrusting && tilt < radians(65.0) && height > 0.24f && gyro.norm() < 8.0f
                           && feedback.support);
    if (captured)
        enter_(JointReferenceRecoveryPhase::kCapture);
    if (orbiting && !orbit_contact && !captured
        && orbit_angle_ >= static_cast<float>(2.0 * std::numbers::pi * config_.orbit_turns))
        fail_(4);
    if (capturing && tilt < radians(20.0) && feedback.support && height > 0.22f)
        enter_(JointReferenceRecoveryPhase::kPrepare);
    const bool stable = feedback.support && feedback.body_clear
                     && tilt < static_cast<float>(config_.handover_tilt_rad)
                     && gyro.norm() < static_cast<float>(config_.handover_gyro_max)
                     && height > static_cast<float>(config_.handover_height_min)
                     && height < static_cast<float>(config_.handover_height_max)
                     && dq.tail<2>().cwiseAbs().maxCoeff() * 0.06f < 0.25f
                     && dq.head<4>().cwiseAbs().maxCoeff() < 2.0f;
    const bool ready = preparing && stable && remaining < 0.02f && measured_error < 0.12f;
    ready_ticks_ = ready ? ready_ticks_ + 1 : 0;
    durations_.ready = ready ? durations_.ready + dt : 0.0;
    // A valid measured capture ends the current script, even before PREPARE.
    // Use the current phase so an earlier failure cannot be cleared here.
    // Dynamic capture uses conditional support and the caller's motion bounds.
    // Bidirectional probes remain part of static handover; their acceleration
    // test cannot require a balancing wheel to settle before a moving capture.
    // Acquire several actual sensor frames before trusting a cold IMU/filter
    // baseline. This is startup sensing time, not a standing-stability dwell.
    const bool captured_for_rl = enabled_ && phase_ < JointReferenceRecoveryPhase::kBlend
                              && reached_(age_ticks_, durations_.age, 0.06)
                              && config_.dynamic_takeover && feedback.rl_capture_ready
                              && feedback.support
                              && height > static_cast<float>(config_.dynamic_capture_height_min)
                              && height < static_cast<float>(config_.handover_height_max);
    if (captured_for_rl || (preparing && reached_(ready_ticks_, durations_.ready, 0.1)))
        enter_(
            config_.blend_seconds == 0.0 ? JointReferenceRecoveryPhase::kRl
                                         : JointReferenceRecoveryPhase::kBlend);
    if (blending && reached_(phase_ticks_, durations_.phase, config_.blend_seconds))
        enter_(JointReferenceRecoveryPhase::kRl);
    const bool holding_rl =
        enabled_ && phase == JointReferenceRecoveryPhase::kRl && !motion_released_;
    stable_ticks_ = holding_rl && stable ? stable_ticks_ + 1 : 0;
    durations_.stable = holding_rl && stable ? durations_.stable + dt : 0.0;
    motion_released_ =
        motion_released_
        || (holding_rl && reached_(stable_ticks_, durations_.stable, config_.stable_seconds));
    if (enabled_
        && (phase_ == JointReferenceRecoveryPhase::kBlend
            || (phase_ == JointReferenceRecoveryPhase::kRl && !motion_released_))
        && tilt > radians(45.0))
        fail_(5);
    if (enabled_ && phase_ != JointReferenceRecoveryPhase::kRl
        && reached_(age_ticks_, durations_.age, config_.max_script_seconds))
        fail_(6);
    if (membership_ && phase_ != JointReferenceRecoveryPhase::kRl
        && (config_.dt == 0.005 ? static_cast<double>(elapsed_ticks_) * config_.dt
                                : elapsed_seconds_ + 1e-12)
               >= config_.max_script_seconds)
        fail_(6);
}

void JointReferenceRecoveryController::update_wheels_(const StepContext& step, bool supported) {
    const auto phase = step.phase;
    const auto& gyro = step.gyro;
    const auto& dq = step.dq;
    const float tilt = step.tilt;
    const float pitch = step.pitch;
    // RecoveryTraining updates wheels after the FSM transition. Entering BLEND
    // recomputes once; subsequent BLEND ticks retain that velocity reference.
    const bool is_capture = phase_ == JointReferenceRecoveryPhase::kCapture;
    const bool is_prepare = phase_ == JointReferenceRecoveryPhase::kPrepare;
    const bool is_blend = enabled_ && phase_ == JointReferenceRecoveryPhase::kBlend;
    const float balance_limit =
        radians(is_capture ? config_.capture_tilt_limit_deg : config_.prepare_tilt_limit_deg);
    const bool eligible = enabled_ && release_finished_ && supported && tilt < balance_limit
                       && (is_prepare || is_capture
                           || (is_blend && phase == JointReferenceRecoveryPhase::kPrepare))
                       && config_.wheel_balance_enabled;
    const bool retained = is_blend && wheel_balance_active_ && supported && tilt < balance_limit;
    Eigen::Vector2f desired =
        retained ? Eigen::Vector2f(targets_.tail<2>()) : Eigen::Vector2f::Zero();
    if (eligible) {
        const float effort = static_cast<float>(config_.pitch_kp_nm_rad) * pitch
                           + static_cast<float>(config_.pitch_kd_nm_s_rad) * gyro.y();
        desired = effort * config_.wheel_axis_signs.cast<float>() / 0.6f
                + (1.0f - static_cast<float>(config_.wheel_damping_nm_s_rad) / 0.6f) * dq.tail<2>();
    }
    wheel_balance_active_ = eligible || retained;
    targets_.tail<2>() = desired;
}

JointReferenceRecoveryVector6 JointReferenceRecoveryController::torques(
    const JointReferenceRecoveryFeedback& feedback,
    const JointReferenceRecoveryVector6& rl_torque) const {
    if (!membership_)
        return rl_torque;
    if (!release_finished_ || command_.failed || !feedback.q.allFinite() || !feedback.dq.allFinite()
        || !feedback.wheel_probe_torque.allFinite() || !rl_torque.allFinite())
        return JointReferenceRecoveryVector6::Zero();
    const Vector6f q = project_feedback(feedback.q).cast<float>();
    const Vector6f dq = feedback.dq.cast<float>();
    Vector6f script;
    script.head<4>() = 160.0f * (targets_.head<4>() - q.head<4>()) - 2.5f * dq.head<4>();
    script.tail<2>() =
        0.6f * (targets_.tail<2>() - dq.tail<2>()) + feedback.wheel_probe_torque.cast<float>();
    script.head<4>() = script.head<4>().cwiseMax(-40.0f).cwiseMin(40.0f);
    script.tail<2>() = script.tail<2>().cwiseMax(-4.5f).cwiseMin(4.5f);
    Vector6f actor = rl_torque.cast<float>();
    actor.head<4>() = actor.head<4>().cwiseMax(-40.0f).cwiseMin(40.0f);
    actor.tail<2>() = actor.tail<2>().cwiseMax(-4.5f).cwiseMin(4.5f);
    const float alpha = static_cast<float>(command_.blend);
    return ((1.0f - alpha) * script + alpha * actor).cast<double>();
}

JointReferenceRecoveryVector6 JointReferenceRecoveryController::project_feedback(
    const JointReferenceRecoveryVector6& input) const {
    const Vector6f raw_q = input.cast<float>();
    Vector6f q;
    for (int i = 0; i < 6; ++i) {
        const float turns = std::nearbyint((canonical_q_[i] - raw_q[i]) / kTau);
        q[i] = turns == 0.0f ? raw_q[i] : raw_q[i] + turns * kTau;
    }
    return q.cast<double>();
}

JointReferenceRecoveryVector6 JointReferenceRecoveryController::effective_action_history(
    const JointReferenceRecoveryVector6& rl_action) const {
    if (!membership_ || command_.pure_rl)
        return rl_action;
    if (!release_finished_ || command_.failed)
        return JointReferenceRecoveryVector6::Zero();
    Vector6f script = Vector6f::Zero();
    const Vector4f delta = targets_.head<4>() - config_.rl_nominal.head<4>().cast<float>();
    for (int i = 0; i < 4; ++i)
        script[i] = wrapped(delta[i]) / 0.25f;
    script.tail<2>() = targets_.tail<2>() / 10.0f;
    const float alpha = static_cast<float>(command_.blend);
    return ((1.0f - alpha) * script + alpha * rl_action.cast<float>()).cast<double>();
}

} // namespace rmcs::rl
