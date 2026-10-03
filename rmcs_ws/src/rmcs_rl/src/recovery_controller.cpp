#include "recovery_controller.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <ranges>
#include <stdexcept>
#include <utility>

namespace rmcs::rl {

namespace {
constexpr double kPi = std::numbers::pi;
constexpr std::array<std::pair<RecoveryPhase, std::string_view>, 12> kPhaseNames{{
    {RecoveryPhase::kIdle, "IDLE"},
    {RecoveryPhase::kFold, "FOLD"},
    {RecoveryPhase::kPlant, "PLANT"},
    {RecoveryPhase::kPrepare, "PREPARE"},
    {RecoveryPhase::kBlend, "BLEND"},
    {RecoveryPhase::kComplete, "COMPLETE"},
    {RecoveryPhase::kFailed, "FAILED"},
    {RecoveryPhase::kOrbit, "ORBIT"},
    {RecoveryPhase::kThrust, "THRUST"},
    {RecoveryPhase::kSideSwing, "SIDE_SWING"},
    {RecoveryPhase::kCapture, "CAPTURE"},
    {RecoveryPhase::kWaitGround, "WAIT_GROUND"},
}};

double tilt(const Eigen::Vector3d& gravity) {
    return std::acos(std::clamp(-gravity.z(), -1.0, 1.0));
}

double signed_pitch(const Eigen::Vector3d& gravity) {
    return std::atan2(gravity.x(), -gravity.z());
}
} // namespace

std::string_view recovery_phase_name(RecoveryPhase phase) noexcept {
    for (const auto& [value, name] : kPhaseNames)
        if (value == phase)
            return name;
    return "UNKNOWN";
}

std::optional<RecoveryPhase> recovery_phase_from_name(std::string_view name) noexcept {
    for (const auto& [value, label] : kPhaseNames)
        if (label == name)
            return value;
    return std::nullopt;
}

double conditional_dm_output_bound(
    double speed_rad_s, double rated_rpm, double rated_torque_nm, double peak_torque_nm) noexcept {
    const double rated_speed = rated_rpm * 2.0 * kPi / 60.0;
    return std::max(
        0.0,
        peak_torque_nm - (peak_torque_nm - rated_torque_nm) * std::abs(speed_rad_s) / rated_speed);
}

bool neutral_recovery_request(bool auto_mode, const Eigen::Vector3d& velocity) noexcept {
    return auto_mode && velocity.allFinite() && velocity.cwiseAbs().maxCoeff() <= 0.02;
}

bool hold_recovery_command(
    bool recovery_started, bool preparing, double seconds_since_rl,
    double stable_seconds) noexcept {
    return recovery_started
        && (preparing || !std::isfinite(seconds_since_rl) || seconds_since_rl < 1.0
            || !std::isfinite(stable_seconds) || stable_seconds < 1.0);
}

bool recovery_upright_for_motion(const RecoveryFeedback& feedback) noexcept {
    return feedback.geometry_valid && feedback.height_valid && feedback.body_clear
        && feedback.gravity.allFinite() && feedback.omega.allFinite() && feedback.dq.allFinite()
        && feedback.gravity.z() < -std::cos(10.0 * kPi / 180.0) && feedback.omega.norm() < 0.75
        && feedback.height_if_grounded > 0.285 && feedback.height_if_grounded < 0.325
        && feedback.dq.head<4>().cwiseAbs().maxCoeff() < 2.0
        && feedback.dq.tail<2>().cwiseAbs().maxCoeff() < 6.0;
}

void RecoveryPeakBudget::configure(double rated_torque_nm, double available_seconds) {
    if (!std::isfinite(rated_torque_nm) || rated_torque_nm <= 0 || !std::isfinite(available_seconds)
        || available_seconds < 0)
        throw std::invalid_argument("Invalid measured above-rated DM torque budget");
    rated_torque_nm_ = rated_torque_nm;
    available_seconds_ = available_seconds;
    reset();
}

void RecoveryPeakBudget::limit(Eigen::Vector4d& torque, double dt) noexcept {
    for (int i = 0; i < 4; ++i) {
        if (std::abs(torque[i]) > rated_torque_nm_) {
            used_seconds_[i] += dt;
            if (used_seconds_[i] > available_seconds_)
                torque[i] = std::copysign(rated_torque_nm_, torque[i]);
        }
    }
}

RecoveryController::RecoveryController(RecoveryConfig config)
    : config_(std::move(config)) {
    const auto positive_parameters =
        std::array{config_.orbit_speed,   config_.side_speed,     config_.rollover_speed,
                   config_.capture_speed, config_.fold_speed,     config_.prepare_speed,
                   config_.stand_speed,   config_.active_timeout, config_.blend_seconds};
    if (!std::ranges::all_of(
            positive_parameters, [](double value) { return std::isfinite(value) && value > 0.0; })
        || !config_.fold.allFinite() || !config_.thrust.allFinite()
        || !config_.side_extended.allFinite() || !config_.plant.allFinite()
        || !config_.stand.allFinite() || !config_.upright.allFinite()
        || !config_.support_extended.allFinite() || !config_.upright_support_extended.allFinite()
        || !config_.capture_extended.allFinite())
        throw std::invalid_argument("Invalid recovery reference or phase duration");
    if (!config_.root_axis_y.allFinite() || !config_.wheel_axis_y.allFinite()
        || (config_.root_axis_y.cwiseAbs().array() < 0.9).any()
        || (config_.root_axis_y.cwiseAbs().array() > 1.001).any()
        || (config_.wheel_axis_y.cwiseAbs().array() < 0.9).any()
        || (config_.wheel_axis_y.cwiseAbs().array() > 1.001).any()
        || std::abs(config_.root_axis_y[0] - config_.root_axis_y[1]) > 1e-3
        || std::abs(config_.root_axis_y[2] - config_.root_axis_y[3]) > 1e-3)
        throw std::invalid_argument("Recovery requires calibrated sagittal/coaxial joint axes");
}

void RecoveryController::reset() noexcept {
    phase_ = RecoveryPhase::kIdle;
    failure_ = RecoveryFailure::kNone;
    elapsed_ = phase_elapsed_ = ready_seconds_ = contact_seconds_ = orbit_angle_ = 0;
    inverted_seconds_ = 0.0;
    side_attempts_ = reroutes_ = 0;
}

bool RecoveryController::valid_(const RecoveryFeedback& feedback) const noexcept {
    return feedback.geometry_valid && feedback.spring_compensation_valid && feedback.q.allFinite()
        && feedback.dq.allFinite() && feedback.gravity.allFinite() && feedback.omega.allFinite()
        && std::abs(feedback.gravity.norm() - 1.0) < 0.1
        && std::ranges::all_of(
               feedback.spring_compensation_nm, [](double x) { return std::isfinite(x); })
        && std::isfinite(feedback.specific_force_norm_mps2)
        && (!feedback.world_wheel_omega_valid || feedback.world_wheel_omega.allFinite())
        && std::isfinite(feedback.height_if_grounded)
        && (!feedback.wheel_heights_valid || std::isfinite(feedback.wheel_height_difference));
}

bool RecoveryController::start(const RecoveryFeedback& feedback) {
    reset();
    if (!valid_(feedback))
        return false;
    pitch_ = last_pitch_ = signed_pitch(feedback.gravity);
    reference_ = feedback.q.head<4>();
    const double angle = tilt(feedback.gravity);
    if (angle >= 70.0 * kPi / 180.0 && std::abs(feedback.gravity.y()) > 0.7) {
        if (!feedback.wheel_heights_valid)
            return false;
        route_ = Route::kSide;
    } else if (angle < 15.0 * kPi / 180.0 && feedback.height_if_grounded > 0.20) {
        route_ = Route::kStand;
    } else if (angle < 70.0 * kPi / 180.0) {
        route_ = Route::kPlant;
    } else {
        route_ = pitch_ >= 0 ? Route::kOrbitPositive : Route::kOrbitNegative;
    }
    transition_(route_ == Route::kStand ? RecoveryPhase::kPrepare : RecoveryPhase::kFold);
    if (route_ == Route::kStand && !feedback.geometrically_supported
        && feedback.specific_force_norm_mps2 < 9.0) {
        route_ = Route::kStand;
        transition_(RecoveryPhase::kWaitGround);
    }
    return true;
}

void RecoveryController::transition_(RecoveryPhase next) noexcept {
    phase_ = next;
    phase_elapsed_ = ready_seconds_ = contact_seconds_ = 0;
}

void RecoveryController::fail_(RecoveryFailure reason) noexcept {
    failure_ = reason;
    transition_(RecoveryPhase::kFailed);
}

void RecoveryController::update_pitch_(const RecoveryFeedback& feedback) noexcept {
    const double raw_pitch = signed_pitch(feedback.gravity);
    pitch_ += std::remainder(raw_pitch - last_pitch_, 2 * kPi);
    last_pitch_ = raw_pitch;
}

Eigen::Vector4d RecoveryController::paired_delta(
    const Eigen::Vector4d& goal, const Eigen::Vector4d& reference) {
    Eigen::Vector4d result;
    for (int side = 0; side < 2; ++side) {
        const int hip = 2 * side;
        // nearbyint is ties-to-even, like the Torch reference's round().
        const double turns = std::nearbyint((goal[hip] - reference[hip]) / (2 * kPi));
        result.segment<2>(hip) = goal.segment<2>(hip) - reference.segment<2>(hip)
                               - Eigen::Vector2d::Constant(2 * kPi * turns);
    }
    return result;
}

bool RecoveryController::ready_for_blend_(const RecoveryFeedback& feedback) const noexcept {
    return feedback.support_confirmed && feedback.settled && feedback.body_clear
        && feedback.height_valid && feedback.height_if_grounded > 0.27
        && feedback.height_if_grounded < 0.36 && tilt(feedback.gravity) < 8.0 * kPi / 180.0
        && feedback.omega.norm() < 0.75;
}

RecoveryVector6 RecoveryController::efforts_(const RecoveryFeedback& feedback) const {
    RecoveryVector6 output = RecoveryVector6::Zero();
    if (phase_ == RecoveryPhase::kWaitGround)
        return output;
    const bool standing = route_ == Route::kStand;
    const double kp = standing ? 80.0 : 120.0;
    const double kd = standing ? 2.0 : 4.0;
    for (int i = 0; i < 4; ++i) {
        double torque = kp * (reference_[i] - feedback.q[i]) - kd * feedback.dq[i];
        torque = std::clamp(torque, -40.0, 40.0);
        if (phase_ == RecoveryPhase::kCapture)
            torque += 2.0 * feedback.omega.y() / config_.root_axis_y[i];
        output[i] = std::clamp(torque, -40.0, 40.0);
    }
    const bool rollover_route = route_ == Route::kOrbitPositive || route_ == Route::kOrbitNegative
                             || route_ == Route::kSide;
    if (rollover_route
        && (phase_ == RecoveryPhase::kFold || phase_ == RecoveryPhase::kOrbit
            || phase_ == RecoveryPhase::kSideSwing)) {
        for (int side = 0; side < 2; ++side) {
            output[2 * side] += feedback.spring_compensation_nm[side];
            output[2 * side + 1] -= feedback.spring_compensation_nm[side];
        }
        for (int i = 0; i < 4; ++i)
            output[i] = std::clamp(output[i], -40.0, 40.0);
    }
    const double balance_limit = phase_ == RecoveryPhase::kCapture ? 85.0
                               : route_ == Route::kPlant           ? 55.0
                                                                   : 70.0;
    const bool balancing = (phase_ == RecoveryPhase::kPlant || phase_ == RecoveryPhase::kPrepare
                            || phase_ == RecoveryPhase::kCapture || phase_ == RecoveryPhase::kBlend)
                        && tilt(feedback.gravity) < balance_limit * kPi / 180.0;
    const bool thrust_braking = phase_ == RecoveryPhase::kThrust;
    if ((balancing || thrust_braking) && feedback.contact_candidate) {
        const bool body_braking = feedback.body_contact_suspected && !standing;
        if (thrust_braking || body_braking) {
            // A relative wheel encoder rate omits base and linkage motion.
            // Leave the effort at zero until those axes are calibrated.
            if (feedback.world_wheel_omega_valid) {
                const double damping = body_braking ? 0.4 : 0.2;
                output.tail<2>() =
                    -damping * feedback.world_wheel_omega.cwiseProduct(config_.wheel_axis_y);
            }
        } else {
            const double effort = 8.0 * signed_pitch(feedback.gravity) + 1.5 * feedback.omega.y();
            output.tail<2>() = effort * config_.wheel_axis_y - 0.2 * feedback.dq.tail<2>();
        }
        output[4] = std::clamp(output[4], -1.5, 1.5);
        output[5] = std::clamp(output[5], -1.5, 1.5);
    }
    return output;
}

double RecoveryController::update_reference_(
    const RecoveryFeedback& feedback, double dt, double angle) {
    Eigen::Vector4d goal = route_ == Route::kStand ? config_.upright
                         : route_ == Route::kPlant ? config_.plant
                                                   : config_.stand;
    double speed = route_ == Route::kStand         ? config_.stand_speed
                 : phase_ == RecoveryPhase::kBlend ? config_.fold_speed
                                                   : config_.prepare_speed;
    bool directed = false;
    const auto phase = phase_;
    if (phase == RecoveryPhase::kFold) {
        goal = config_.fold;
        speed = config_.fold_speed;
    } else if (phase == RecoveryPhase::kPlant) {
        goal = config_.fold;
        // Match the V5 sensor-only PLANT domain: lateral poses keep the fold,
        // while sagittal placement uses principal pitch and sign(0) == 0.
        if (std::abs(feedback.gravity.y()) < 0.5) {
            const double pitch = signed_pitch(feedback.gravity);
            const double direction = (pitch > 0.0) - (pitch < 0.0);
            const double offset = std::clamp(-pitch - direction * (20.0 * kPi / 180.0), -2.2, 2.2);
            for (int i = 0; i < 4; ++i)
                goal[i] += offset / config_.root_axis_y[i];
        }
        speed = config_.fold_speed;
    } else if (phase == RecoveryPhase::kOrbit) {
        orbit_angle_ += config_.orbit_speed * dt;
        const double direction = route_ == Route::kOrbitPositive ? 1.0 : -1.0;
        for (int i = 0; i < 4; ++i)
            goal[i] = orbit_start_[i] + direction * orbit_angle_ / config_.root_axis_y[i];
        speed = config_.orbit_speed;
        directed = true;
    } else if (phase == RecoveryPhase::kThrust) {
        for (int i = 0; i < 4; ++i)
            goal[i] = config_.thrust[i]
                    + (planted_orbit_ - (pitch_ - planted_pitch_)) / config_.root_axis_y[i];
        speed = config_.rollover_speed;
        directed = true;
    } else if (phase == RecoveryPhase::kSideSwing) {
        const double duration = kPi / config_.side_speed;
        const double stroke = std::clamp(
            phase_elapsed_ < duration
                ? config_.side_speed * phase_elapsed_
                : kPi - config_.side_speed * std::max(0.0, phase_elapsed_ - duration - 0.08),
            0.0, kPi);
        for (int i = 0; i < 4; ++i)
            goal[i] = side_start_[i] + side_direction_[i] * stroke / config_.root_axis_y[i];
        speed = config_.side_speed;
        directed = true;
    } else if (
        phase == RecoveryPhase::kCapture || phase == RecoveryPhase::kPrepare
        || phase == RecoveryPhase::kBlend) {
        if (phase == RecoveryPhase::kCapture)
            speed = config_.capture_speed;
        const double alignment_limit = phase == RecoveryPhase::kCapture ? 85.0 : 65.0;
        const bool alignment =
            route_ == Route::kPlant ? feedback.alignment_candidate : feedback.contact_candidate;
        if (alignment && angle < alignment_limit * kPi / 180.0) {
            const double extension =
                std::clamp((angle - 12.0 * kPi / 180.0) / (43.0 * kPi / 180.0), 0.0, 1.0);
            const auto& extended = phase == RecoveryPhase::kCapture ? config_.capture_extended
                                 : route_ == Route::kStand ? config_.upright_support_extended
                                                           : config_.support_extended;
            const auto& neutral = route_ == Route::kStand ? config_.upright : config_.stand;
            goal = neutral + extension * (extended - neutral);
            const double pitch = signed_pitch(feedback.gravity);
            for (int i = 0; i < 4; ++i)
                goal[i] -= pitch / config_.root_axis_y[i];
        }
    }

    const Eigen::Vector4d difference =
        directed ? goal - reference_ : paired_delta(goal, reference_);
    const double max_step = speed * dt;
    if (route_ == Route::kStand
        && (phase == RecoveryPhase::kPrepare || phase == RecoveryPhase::kBlend)) {
        reference_ +=
            difference * std::min(1.0, max_step / std::max(1e-6, difference.cwiseAbs().maxCoeff()));
    } else {
        for (int i = 0; i < 4; ++i)
            reference_[i] += std::clamp(difference[i], -max_step, max_step);
    }
    return difference.cwiseAbs().maxCoeff();
}

void RecoveryController::update_phase_(
    const RecoveryFeedback& feedback, double dt, double angle, double reference_error) {
    const auto phase = phase_;
    const bool reference_ready = reference_error < 0.02;
    const auto tracking_error = (reference_ - feedback.q.head<4>()).eval();
    const double measured_error =
        tracking_error
            .unaryExpr([](double error) { return std::abs(std::remainder(error, 2 * kPi)); })
            .maxCoeff();
    const bool tracked = measured_error < 0.2;
    if (phase == RecoveryPhase::kFold && reference_ready && (route_ == Route::kPlant || tracked)
        && phase_elapsed_ >= 0.3) {
        if (route_ == Route::kSide) {
            side_start_ = reference_;
            side_direction_.setZero();
            const int lower = feedback.wheel_height_difference < 0 ? 0 : 1;
            const int upper = 1 - lower;
            side_direction_.segment<2>(2 * lower).setOnes();
            side_start_.segment<2>(2 * upper) +=
                (config_.side_extended - config_.fold).segment<2>(2 * upper);
            side_started_lateral_ = std::abs(feedback.gravity.y()) > 0.7;
            ++side_attempts_;
            transition_(RecoveryPhase::kSideSwing);
        } else if (route_ == Route::kPlant) {
            transition_(RecoveryPhase::kPlant);
        } else {
            route_ = pitch_ >= 0 ? Route::kOrbitPositive : Route::kOrbitNegative;
            orbit_start_ = reference_;
            orbit_angle_ = 0;
            transition_(RecoveryPhase::kOrbit);
        }
    } else if (phase == RecoveryPhase::kSideSwing) {
        if (phase_elapsed_ > 0.05
            && (side_started_lateral_
                    ? std::abs(feedback.gravity.y()) < 0.65
                    : std::abs(feedback.gravity.y()) > 0.7 || angle < 70.0 * kPi / 180.0)) {
            if (std::abs(feedback.gravity.y()) > 0.7 && side_attempts_ < 2)
                route_ = Route::kSide;
            else
                route_ = pitch_ >= 0 ? Route::kOrbitPositive : Route::kOrbitNegative;
            transition_(RecoveryPhase::kFold);
        } else if (phase_elapsed_ > 2 * kPi / config_.side_speed + 0.4) {
            fail_(RecoveryFailure::kNoReorientation);
        }
    } else if (phase == RecoveryPhase::kPlant) {
        contact_seconds_ = feedback.contact_candidate ? contact_seconds_ + dt : 0.0;
        if (reference_error < 0.15 && contact_seconds_ >= 0.06)
            transition_(RecoveryPhase::kPrepare);
        else if (phase_elapsed_ > 2.5)
            fail_(RecoveryFailure::kNoContact);
    } else if (phase == RecoveryPhase::kOrbit) {
        const bool supported = feedback.contact_candidate;
        if (supported && angle < 30.0 * kPi / 180.0 && feedback.height_if_grounded > 0.22) {
            transition_(RecoveryPhase::kPrepare);
        } else if (
            supported && angle < 145.0 * kPi / 180.0 && feedback.height_if_grounded > 0.12
            && orbit_angle_ > 3.7) {
            planted_pitch_ = pitch_;
            planted_orbit_ = (route_ == Route::kOrbitPositive ? 1.0 : -1.0) * orbit_angle_;
            transition_(RecoveryPhase::kThrust);
        } else if (orbit_angle_ >= 2 * kPi) {
            fail_(RecoveryFailure::kOrbitExhausted);
        }
    } else if (phase == RecoveryPhase::kThrust) {
        // CAPTURE arrests momentum; conditional geometry suffices to enter it.
        // Confirmed support is still mandatory before the RL handover.
        if (angle < 65.0 * kPi / 180.0 && feedback.height_if_grounded > 0.24
            && feedback.omega.norm() < 8.0)
            transition_(RecoveryPhase::kCapture);
    } else if (phase == RecoveryPhase::kPrepare || phase == RecoveryPhase::kCapture) {
        const bool reference_settled =
            route_ != Route::kPlant || (reference_ready && measured_error < 0.12);
        ready_seconds_ =
            ready_for_blend_(feedback) && reference_settled ? ready_seconds_ + dt : 0.0;
        if (ready_seconds_ >= 0.1)
            transition_(RecoveryPhase::kBlend);
    } else if (phase == RecoveryPhase::kBlend && phase_elapsed_ >= config_.blend_seconds) {
        transition_(RecoveryPhase::kComplete);
    }
}

RecoveryCommand RecoveryController::step(const RecoveryFeedback& feedback, double dt) {
    if (phase_ == RecoveryPhase::kIdle || phase_ == RecoveryPhase::kFailed
        || phase_ == RecoveryPhase::kComplete)
        return {.phase = phase_, .failure = failure_};
    if (!std::isfinite(dt) || dt <= 0 || dt > 0.02 || !valid_(feedback)) {
        fail_(RecoveryFailure::kInvalidFeedback);
        return {.phase = phase_, .failure = failure_};
    }
    elapsed_ += dt;
    phase_elapsed_ += dt;
    update_pitch_(feedback);
    const double angle = tilt(feedback.gravity);
    if (phase_ == RecoveryPhase::kWaitGround) {
        const bool landing =
            (feedback.contact_candidate || feedback.body_contact_suspected)
            && feedback.omega.norm() < 1.0 && feedback.dq.head<4>().cwiseAbs().maxCoeff() < 2.0
            && feedback.dq.tail<2>().cwiseAbs().maxCoeff() < 10.0
            && feedback.specific_force_norm_mps2 > 7.0 && feedback.specific_force_norm_mps2 < 12.0;
        contact_seconds_ = landing ? contact_seconds_ + dt : 0.0;
        if (contact_seconds_ >= 0.12) {
            reference_ = feedback.q.head<4>();
            if (feedback.height_if_grounded < 0.22) {
                route_ = Route::kPlant;
                transition_(RecoveryPhase::kFold);
            } else {
                transition_(RecoveryPhase::kPrepare);
            }
        }
        if (elapsed_ > config_.active_timeout)
            fail_(RecoveryFailure::kTimeout);
        return {
            .torque = phase_ == RecoveryPhase::kFailed ? RecoveryVector6::Zero().eval()
                                                       : efforts_(feedback),
            .phase = phase_,
            .failure = failure_};
    }
    const bool reroute_candidate =
        (phase_ == RecoveryPhase::kPrepare || phase_ == RecoveryPhase::kCapture)
        && (route_ == Route::kStand || route_ == Route::kPlant) && angle > 100.0 * kPi / 180.0
        && feedback.omega.norm() < 3.0 && reroutes_ == 0 && elapsed_ < config_.active_timeout - 3.0;
    inverted_seconds_ = reroute_candidate ? inverted_seconds_ + dt : 0.0;
    if (inverted_seconds_ >= 0.15) {
        route_ = pitch_ >= 0 ? Route::kOrbitPositive : Route::kOrbitNegative;
        reference_ = feedback.q.head<4>();
        ++reroutes_;
        transition_(RecoveryPhase::kFold);
    }
    if ((phase_ == RecoveryPhase::kBlend || phase_ == RecoveryPhase::kComplete)
        && angle > 45.0 * kPi / 180.0) {
        fail_(RecoveryFailure::kLostUpright);
        return {.phase = phase_, .failure = failure_};
    }

    // The current phase advances its reference before any transition. Efforts
    // below use the resulting phase, including its entry/exit torque rules.
    const double reference_error = update_reference_(feedback, dt, angle);
    update_phase_(feedback, dt, angle, reference_error);
    // The active script budget ends when BLEND begins. Let its 200 ms transfer
    // finish even if the script reached readiness near the budget boundary.
    if (phase_ != RecoveryPhase::kBlend && phase_ != RecoveryPhase::kComplete
        && phase_ != RecoveryPhase::kFailed && elapsed_ > config_.active_timeout)
        fail_(RecoveryFailure::kTimeout);

    RecoveryCommand command{.phase = phase_, .failure = failure_};
    if (phase_ != RecoveryPhase::kFailed && phase_ != RecoveryPhase::kComplete)
        command.torque = efforts_(feedback);
    if (phase_ == RecoveryPhase::kBlend)
        command.blend = std::clamp(phase_elapsed_ / config_.blend_seconds, 0.0, 1.0);
    return command;
}

} // namespace rmcs::rl
