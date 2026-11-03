#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>

#include "controller/gimbal/coordinated_yaw.hpp"

namespace {

using rmcs_core::controller::gimbal::CoordinatedYaw;
using rmcs_core::controller::gimbal::CoordinatedYawOutput;
using rmcs_core::controller::gimbal::CoordinatedYawParameters;

constexpr double kPi = std::numbers::pi;
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

void near(double actual, double expected, const char* message, double tolerance = 1e-9) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
        throw std::runtime_error(std::string{message} + ": actual=" + std::to_string(actual)
                                 + ", expected=" + std::to_string(expected));
}

template <typename Callable>
void rejects(Callable&& callable, const char* message) {
    try {
        std::forward<Callable>(callable)();
    } catch (const std::invalid_argument&) {
        return;
    }
    throw std::runtime_error(message);
}

double wrapped(double angle) { return std::remainder(angle, 2.0 * kPi); }

void stopped(const CoordinatedYawOutput& output, const char* message) {
    require(!output.active, message);
    near(output.small_velocity, 0.0, "inactive small-yaw reference must be zero");
    near(output.big_velocity, 0.0, "inactive big-yaw reference must be zero");
    near(output.small_acceleration, 0.0, "inactive small-yaw acceleration must be zero");
    near(output.big_acceleration, 0.0, "inactive big-yaw acceleration must be zero");
}

void reference_bounds(const CoordinatedYawOutput& output, const CoordinatedYawOutput& previous,
                      const CoordinatedYawParameters& parameters, double dt) {
    require(output.active, "valid kinematic execution must remain active");
    require(std::abs(output.small_velocity) <= parameters.max_small_velocity_rad_s + 1e-10,
            "small-yaw reference exceeds its speed capability");
    require(std::abs(output.big_velocity) <= parameters.max_big_velocity_rad_s + 1e-10,
            "big-yaw reference exceeds its speed capability");
    require(std::abs(output.small_acceleration) <= parameters.max_small_acceleration_rad_s2 + 1e-9,
            "small-yaw reference exceeds its acceleration capability");
    require(std::abs(output.big_acceleration) <= parameters.max_big_acceleration_rad_s2 + 1e-9,
            "big-yaw reference exceeds its acceleration capability");
    require(std::abs(output.small_velocity - previous.small_velocity)
                <= parameters.max_small_acceleration_rad_s2 * dt + 1e-10,
            "normal small-yaw deceleration/reversal must remain continuous");
    require(std::abs(output.big_velocity - previous.big_velocity)
                <= parameters.max_big_acceleration_rad_s2 * dt + 1e-10,
            "normal big-yaw deceleration/reversal must remain continuous");
}

// The plant here executes velocity references ideally. This checks allocation,
// capture, limits and geometry; it does not assert motor/friction performance.
void thirty_degree_handoff(double direction, double kinematic_sign, double dt) {
    CoordinatedYawParameters parameters;
    parameters.small_yaw_direction = kinematic_sign;
    CoordinatedYaw controller{parameters};
    const double big_origin = 2.8;
    const double small_center = kPi - .04;
    double big = big_origin, small = small_center, measured_big = 0.0;
    const double goal = direction * kPi / 6.0;
    double integrated_request = 0.0;
    auto output = controller.update(0.0, big, wrapped(small), 0.0, dt, true);
    require(output.active, "first permitted sample should capture a valid reference");
    near(output.target_angle, 0.0, "first enable must capture the current total heading");
    near(output.small_center_angle, wrapped(small_center), "enable must capture current small-yaw center");
    near(output.small_velocity, 0.0, "enable must not jump to a configured joint zero");
    near(output.big_velocity, 0.0, "enable must not move big yaw to a configured zero");
    bool big_started_early = false;
    bool counter_motion_after_release = false;
    bool input_finished = false;
    double held_target = 0.0;
    for (int tick = 0; tick < static_cast<int>(9.0 / dt); ++tick) {
        const double remaining = goal - integrated_request;
        const double rate = std::copysign(std::min(.5, std::abs(remaining) / dt), remaining);
        integrated_request += rate * dt;
        const auto previous = output;
        output = controller.update(rate, big, wrapped(small), measured_big, dt, true);
        reference_bounds(output, previous, parameters, dt);
        if (std::abs(output.big_velocity) > 1e-6 && std::abs(output.small_offset) < std::abs(goal) / 2.0)
            big_started_early = true;
        if (std::abs(remaining) <= 1e-10) {
            if (!input_finished) {
                input_finished = true;
                held_target = output.target_angle;
            }
            near(output.target_angle, held_target, "centered input must retain the accumulated angle target");
            if (output.big_velocity * (kinematic_sign * output.small_velocity) < 0.0)
                counter_motion_after_release = true;
        }
        measured_big = output.big_velocity;
        big += output.big_velocity * dt;
        small += output.small_velocity * dt;
    }
    require(big_started_early, "big yaw must follow before small yaw reaches the requested heading");
    require(counter_motion_after_release, "centered input must allow small yaw to counter big-yaw handoff");
    near(output.target_angle, goal, "thirty-degree command must retain its integrated goal", 1e-8);
    near((big - big_origin) + kinematic_sign * (small - small_center), goal,
         "total pointing must converge to thirty degrees after handoff", .004);
    near(small - small_center, 0.0, "small yaw must return inside its configured follow deadband",
         parameters.follow_deadband_rad + .002);
    near(big - big_origin, goal, "big yaw must take over the heading within the two configured deadbands",
         parameters.follow_deadband_rad + parameters.heading_error_deadband_rad + .002);
    require(!output.reference_limited, "settled, reachable target should not remain reference limited");
}

void capture_is_bumpless_and_wrap_is_continuous() {
    CoordinatedYaw controller;
    auto output = controller.update(.5, 9.0, kPi - .01, .0, .01, true);
    near(output.small_velocity, 0.0, "first call captures before applying an input increment");
    near(output.big_velocity, 0.0, "first call must not immediately follow a raw nonzero motor angle");
    output = controller.update(0.0, 9.0, -kPi + .01, 0.0, .01, true);
    require(output.active, "crossing a single-turn encoder wrap must not trip a false jump");
    near(output.small_offset, .02, "small yaw unwrap must use the short physical rotation");
    near(output.heading_angle, .02, "total heading must remain continuous across wrap");
    require(output.big_velocity > 0.0, "small-yaw deflection must immediately initiate big-yaw follow");
    controller.reset();
    output = controller.update(0.0, -15.0, -.7, 0.0, .01, true);
    near(output.heading_angle, 0.0, "reset must capture a new total origin");
    near(output.small_center_angle, -.7, "reset must capture the new small-yaw center");
}

void compensation_uses_actual_big_yaw_velocity() {
    for (const double sign : {-1.0, 1.0}) {
        CoordinatedYawParameters parameters;
        parameters.small_yaw_direction = sign;
        CoordinatedYaw stationary{parameters}, moving{parameters};
        stationary.update(0.0, 0.0, 0.0, 0.0, .01, true);
        moving.update(0.0, 0.0, 0.0, 0.0, .01, true);
        const auto still = stationary.update(0.0, 0.0, 0.0, 0.0, .01, true);
        const auto actual = moving.update(0.0, 0.0, 0.0, .4, .01, true);
        near(still.big_velocity, actual.big_velocity, "identical positions must produce identical big-yaw follow demand");
        near(still.small_velocity, 0.0, "stationary zero-error plant must not require compensation");
        require(sign * actual.small_velocity < 0.0,
                "actual big-yaw motion must induce opposite small-yaw compensation even with zero big target");
    }
}

void standstill_speed_quantization_does_not_start_small_yaw() {
    CoordinatedYaw controller;
    constexpr double dt = .005;
    constexpr std::array jitter{-.018, -.006, .006, .018};
    controller.update(0.0, 0.0, 0.0, 0.0, dt, true);
    for (int tick = 0; tick < 1000; ++tick) {
        const auto output = controller.update(0.0, 0.0, 0.0, jitter[tick % jitter.size()], dt, true);
        require(output.active, "finite standstill speed quantization must not cause a fault");
        near(output.small_velocity, 0.0, "stationary big-yaw speed bins must not start small-yaw motion");
        near(output.big_velocity, 0.0, "stationary encoder positions must not start big-yaw following");
        near(output.target_angle, 0.0, "standstill speed quantization must not accumulate an angle command");
    }
}

void genuine_subthreshold_motion_remains_visible_to_position_feedback() {
    CoordinatedYaw controller;
    constexpr double dt = .005;
    constexpr double physical_big_velocity = .01;
    double big = 0.0, small = 0.0;
    controller.update(0.0, big, small, physical_big_velocity, dt, true);
    bool compensated = false;
    for (int tick = 0; tick < 400; ++tick) {
        // An external disturbance moves big yaw below the feedforward noise
        // threshold. Its real encoder motion must still reach the heading loop.
        big += physical_big_velocity * dt;
        const auto output = controller.update(0.0, big, small, physical_big_velocity, dt, true);
        require(output.active, "real slow encoder motion should remain valid feedback");
        near(output.target_angle, 0.0, "unrequested base motion must not move the held pointing target");
        compensated |= output.small_velocity < -.001;
        small += output.small_velocity * dt;
    }
    require(compensated, "heading position error must correct true motion below the speed noise threshold");
    require(std::abs(big + small) < big / 2.0,
            "position feedback must reduce pointing drift despite suppressed speed feedforward");
}

void external_motion_cannot_drag_a_centered_user_target() {
    for (const double direction : {-1.0, 1.0}) {
        CoordinatedYaw controller;
        constexpr double dt = .01;
        controller.update(0.0, 0.0, 0.0, 0.0, dt, true);
        CoordinatedYawOutput output;
        for (int tick = 0; tick < 10; ++tick)
            output = controller.update(direction * .5, 0.0, 0.0, 0.0, dt, true);
        const double held_target = output.target_angle;
        for (int tick = 1; tick <= 40; ++tick) {
            output = controller.update(0.0, direction * .02 * tick, 0.0, direction * 2.0, dt, true);
            require(output.active, "continuous external big-yaw motion must remain valid angle feedback");
            near(output.target_angle, held_target,
                 "external motion beyond the lead window must not pull a held user target", 1e-12);
        }
        const double displaced_big = direction * .8;
        output = controller.update(-direction * .5, displaced_big, 0.0, 0.0, dt, true);
        require(output.active && output.reference_limited,
                "new input increasing excessive target lead must be limited without a fault");
        near(output.target_angle, held_target,
             "input must not add outward lead after external motion", 1e-12);
        output = controller.update(direction * .5, displaced_big, 0.0, 0.0, dt, true);
        require(output.active, "input reducing excessive target lead must remain permitted");
        near(output.target_angle, held_target + direction * .5 * dt,
             "input reducing excessive lead must be accepted at the unchanged actual pose", 1e-12);
    }
}

void large_quantized_speed_feedback_cannot_accumulate_a_user_angle() {
    for (const double dt : {.001, .005}) {
        for (const double speed : {.03, .3}) {
            for (const int hold_ticks : {1, 4}) {
                CoordinatedYawParameters parameters;
                CoordinatedYaw controller{parameters};
                auto output = controller.update(0.0, 0.0, 0.0, 0.0, dt, true);
                bool compensation_seen = false;
                // Angle feedback deliberately stays fixed. Velocity/angle
                // inconsistency isolates CAN speed-bin effects, not a plant.
                for (int tick = 0; tick < 2000; ++tick) {
                    const auto previous = output;
                    const double measured = (tick / hold_ticks) % 2 == 0 ? speed : -speed;
                    output = controller.update(0.0, 0.0, 0.0, measured, dt, true);
                    reference_bounds(output, previous, parameters, dt);
                    near(output.target_angle, 0.0,
                         "velocity measurement alone must never accumulate a user heading target", 1e-12);
                    near(output.big_velocity, 0.0,
                         "fixed small-yaw angle must not create a big-yaw follow command");
                    compensation_seen |= std::abs(output.small_velocity) > .001;
                }
                require(compensation_seen,
                        "out-of-gate measured motion must still reach the compensation path");
                for (int tick = 0; tick < 200; ++tick) {
                    const auto previous = output;
                    output = controller.update(0.0, 0.0, 0.0, 0.0, dt, true);
                    reference_bounds(output, previous, parameters, dt);
                }
                near(output.target_angle, 0.0, "speed-bin recovery must retain the zero user goal");
                near(output.small_velocity, 0.0,
                     "compensation must settle to zero when inconsistent velocity samples end");
            }
        }
    }
}

void heading_deadband_softens_only_centered_angle_feedback() {
    CoordinatedYawParameters parameters;
    constexpr double dt = .005;
    for (const double direction : {-1.0, 1.0}) {
        for (const double excess : {-.0001, .0001}) {
            CoordinatedYaw controller{parameters};
            controller.update(0.0, 0.0, 0.0, 0.0, dt, true);
            CoordinatedYawOutput output;
            for (int tick = 0; tick < 20; ++tick)
                output = controller.update(0.0,
                    direction * (parameters.heading_error_deadband_rad + excess), 0.0, 0.0, dt, true);
            require(output.active, "small settled angle errors must remain valid");
            if (excess < 0.0)
                near(output.small_velocity, 0.0,
                     "centered angle error inside the configured deadband must not move small yaw");
            else {
                require(direction * output.small_velocity < 0.0,
                        "angle error just outside the centered deadband must retain corrective authority");
                require(std::abs(output.small_velocity) < .001,
                        "crossing the centered deadband must introduce only a small continuous correction");
            }
        }
        auto unsoftened_parameters = parameters;
        unsoftened_parameters.heading_error_deadband_rad = 0.0;
        CoordinatedYaw softened{parameters}, unsoftened{unsoftened_parameters};
        softened.update(0.0, 0.0, 0.0, 0.0, dt, true);
        unsoftened.update(0.0, 0.0, 0.0, 0.0, dt, true);
        const auto with_deadband = softened.update(direction * .001,
            direction * .002, 0.0, 0.0, dt, true);
        const auto without_deadband = unsoftened.update(direction * .001,
            direction * .002, 0.0, 0.0, dt, true);
        require(std::abs(with_deadband.small_velocity) > .001,
                "small nonzero stick input must retain heading feedback inside the settled deadband");
        near(with_deadband.small_velocity, without_deadband.small_velocity,
             "configured settled deadband must have no effect while the stick requests motion");
    }
}

void actual_motion_compensation_survives_the_heading_deadband() {
    for (const double direction : {-1.0, 1.0}) {
        for (const double sign : {-1.0, 1.0}) {
            CoordinatedYawParameters parameters;
            parameters.small_yaw_direction = sign;
            CoordinatedYaw controller{parameters};
            constexpr double dt = .005;
            auto output = controller.update(0.0, 0.0, 0.0, 0.0, dt, true);
            for (int tick = 0; tick < 100; ++tick) {
                const auto previous = output;
                output = controller.update(0.0, direction * .002, 0.0, direction * .3, dt, true);
                reference_bounds(output, previous, parameters, dt);
                near(output.target_angle, 0.0, "compensation inside the deadband must retain the held goal");
            }
            require(std::abs(output.heading_error) < parameters.heading_error_deadband_rad,
                    "compensation regression must exercise an angle inside the centered deadband");
            near(sign * output.small_velocity, -direction * .3,
                 "actual big-yaw velocity must remain compensated inside the angle deadband");
        }
    }
}

void stiction_does_not_hide_growing_heading_error() {
    constexpr double dt = .005;
    constexpr double breakaway_reference = .08;
    for (const double direction : {-1.0, 1.0}) {
        for (const double speed : {.01, .03}) {
            CoordinatedYawParameters parameters;
            CoordinatedYaw controller{parameters};
            double big = 0.0, small = 0.0, maximum_error = 0.0;
            int stuck_ticks = 0;
            bool broke_away = false;
            auto output = controller.update(0.0, big, small, 0.0, dt, true);
            for (int tick = 0; tick < 1600; ++tick) {
                // External base motion is imposed. Small yaw executes no
                // reference below breakaway, so its actual angle remains still
                // until heading feedback grows enough to overcome this model.
                big += direction * speed * dt;
                const auto previous = output;
                output = controller.update(0.0, big, wrapped(small), direction * speed, dt, true);
                reference_bounds(output, previous, parameters, dt);
                near(output.target_angle, 0.0,
                     "stiction and imposed base motion must not drag the held user target", 1e-12);
                maximum_error = std::max(maximum_error, std::abs(output.heading_error));
                if (std::abs(output.small_velocity) < breakaway_reference) {
                    ++stuck_ticks;
                } else {
                    require(direction * output.small_velocity < 0.0,
                            "growing heading error must break small yaw away opposite the base motion");
                    broke_away = true;
                    small += output.small_velocity * dt;
                }
            }
            require(stuck_ticks > 20 && broke_away,
                    "stiction regression must include real stationary samples followed by breakaway");
            require(direction * small < -.02,
                    "small yaw must eventually produce substantial compensation despite stiction");
            require(maximum_error < .06,
                    "stiction must not make the held heading silently follow the imposed base motion");
        }
    }
}

// These are broad regression bounds for this synthetic plant, not a claim of
// good full-stick performance: the 80 ms case has substantial release overshoot.
void short_full_stick_stays_inside_its_known_range(double direction, double lag_s) {
    CoordinatedYawParameters parameters;
    CoordinatedYaw controller{parameters};
    constexpr double dt = .005;
    const double response = lag_s == 0.0 ? 1.0 : 1.0 - std::exp(-dt / lag_s);
    double big = 0.0, small = 0.0, actual_big = 0.0, actual_small = 0.0;
    double held_target = 0.0, maximum_offset = 0.0, maximum_overshoot = 0.0;
    auto output = controller.update(0.0, big, small, 0.0, dt, true);
    for (int tick = 0; tick < 2000; ++tick) {
        const auto previous = output;
        output = controller.update(tick < 160 ? direction : 0.0,
                                   big, wrapped(small), actual_big, dt, true);
        reference_bounds(output, previous, parameters, dt);
        if (tick == 159)
            held_target = output.target_angle;
        if (tick >= 160) {
            near(output.target_angle, held_target,
                 "full-stick release must exactly freeze the accepted user target", 1e-12);
            maximum_overshoot = std::max(maximum_overshoot, direction * (big + small - held_target));
        }
        actual_big += response * (output.big_velocity - actual_big);
        actual_small += response * (output.small_velocity - actual_small);
        big += actual_big * dt;
        small += actual_small * dt;
        maximum_offset = std::max(maximum_offset, std::abs(small));
    }
    near(held_target, direction * .8, "short full-stick input must accept its complete angular increment", 1e-8);
    require(maximum_offset < parameters.max_small_offset_rad,
            "known short full-stick plants must retain their small-yaw working range");
    require(maximum_overshoot < .8,
            "short full-stick overshoot must not exceed its documented broad synthetic bound");
    near(big + small, held_target, "short full-stick release must eventually settle at its held heading",
         parameters.heading_error_deadband_rad + .003);
    near(small, 0.0, "short full-stick release must eventually complete the handoff",
         parameters.follow_deadband_rad + .003);
}

void sustained_moderate_input_survives_asymmetric_lags(double direction, double big_lag_s,
                                                      double small_lag_s) {
    CoordinatedYawParameters parameters;
    CoordinatedYaw controller{parameters};
    constexpr double dt = .005;
    const double big_response = 1.0 - std::exp(-dt / big_lag_s);
    const double small_response = 1.0 - std::exp(-dt / small_lag_s);
    double big = 0.0, small = 0.0, actual_big = 0.0, actual_small = 0.0, maximum_offset = 0.0;
    auto output = controller.update(0.0, big, small, 0.0, dt, true);
    for (int tick = 0; tick < 3600; ++tick) {
        const auto previous = output;
        output = controller.update(tick < 2000 ? direction * .5 : 0.0,
                                   big, wrapped(small), actual_big, dt, true);
        reference_bounds(output, previous, parameters, dt);
        near(output.target_angle, direction * .5 * std::min(tick + 1, 2000) * dt,
             "moderate sustained input must accept every increment and freeze exactly on release", 1e-8);
        actual_big += big_response * (output.big_velocity - actual_big);
        actual_small += small_response * (output.small_velocity - actual_small);
        big += actual_big * dt;
        small += actual_small * dt;
        maximum_offset = std::max(maximum_offset, std::abs(small));
    }
    require(maximum_offset < .6, "moderate asymmetric plants must retain substantial small-yaw travel margin");
    near(big + small, direction * 5.0,
         "moderate asymmetric sustained motion must settle at its complete accepted heading", .01);
    near(small, 0.0, "moderate asymmetric motion must finish its big-yaw handoff",
         parameters.follow_deadband_rad + .005);
}

void sustained_full_stick_with_large_lag_trips_the_working_range() {
    for (const double direction : {-1.0, 1.0}) {
        CoordinatedYawParameters parameters;
        CoordinatedYaw controller{parameters};
        constexpr double dt = .005;
        const double response = 1.0 - std::exp(-dt / .08);
        double big = 0.0, small = 0.0, actual_big = 0.0, actual_small = 0.0;
        auto output = controller.update(0.0, big, small, 0.0, dt, true);
        bool tripped = false;
        // This intentionally records a known limitation rather than calling
        // ten seconds of full-stick tracking a successful synthetic test.
        for (int tick = 0; tick < 2000; ++tick) {
            const auto previous = output;
            output = controller.update(direction, big, wrapped(small), actual_big, dt, true);
            if (!output.active) {
                stopped(output, "large-lag full-stick travel exhaustion must fail closed");
                tripped = true;
                break;
            }
            reference_bounds(output, previous, parameters, dt);
            actual_big += response * (output.big_velocity - actual_big);
            actual_small += response * (output.small_velocity - actual_small);
            big += actual_big * dt;
            small += actual_small * dt;
        }
        require(tripped, "known sustained full-stick lag case must expose its working-range limitation");
        require(std::abs(small) > parameters.max_small_offset_rad
                    && std::abs(small) <= parameters.max_small_offset_rad
                        + parameters.max_small_velocity_rad_s * dt + 1e-9,
                "working-range fault must occur within one sampled small-yaw motion interval");
    }
}

void normal_reversal_and_stop_respect_acceleration() {
    for (const double dt : {.001, .007, .02}) {
        CoordinatedYawParameters parameters;
        CoordinatedYaw controller{parameters};
        double big = 0.0, small = 0.0, measured_big = 0.0;
        auto output = controller.update(0.0, big, small, measured_big, dt, true);
        for (int tick = 0; tick < static_cast<int>(4.0 / dt); ++tick) {
            const double time = tick * dt;
            const double rate = time < 1.0 ? 1.0 : time < 2.0 ? -1.0 : 0.0;
            const auto previous = output;
            output = controller.update(rate, big, wrapped(small), measured_big, dt, true);
            reference_bounds(output, previous, parameters, dt);
            big += output.big_velocity * dt;
            small += output.small_velocity * dt;
            measured_big = output.big_velocity;
        }
    }
}

void blocked_plant_cannot_accumulate_unbounded_target() {
    for (const double direction : {-1.0, 1.0}) {
        CoordinatedYawParameters parameters;
        CoordinatedYaw controller{parameters};
        controller.update(0.0, 0.0, 0.0, 0.0, .005, true);
        bool limited = false;
        for (int tick = 0; tick < 2000; ++tick) {
            const auto output = controller.update(direction, 0.0, 0.0, 0.0, .005, true);
            require(std::abs(output.heading_error) <= parameters.max_target_error_rad + 1e-10,
                    "blocked plant must not accumulate an unbounded pointing error");
            require(std::abs(output.target_angle) <= parameters.max_target_error_rad + 1e-10,
                    "blocked plant must bound its accumulated angle command");
            limited |= output.reference_limited;
        }
        require(limited, "target anti-accumulation must be visible in telemetry");
    }
}

void stalled_big_yaw_cannot_exhaust_small_yaw_travel() {
    for (const double direction : {-1.0, 1.0}) {
        CoordinatedYawParameters parameters;
        CoordinatedYaw controller{parameters};
        const double dt = .005;
        double small = .2;
        controller.update(0.0, 0.0, small, 0.0, dt, true);
        bool tripped = false;
        for (int tick = 0; tick < 1000; ++tick) {
            // Big yaw is mechanically blocked; only small yaw executes its
            // reference. The caller latches this fault and stops at the first
            // inactive sample, rather than silently re-enabling the helper.
            const auto output = controller.update(direction * .6, 0.0, wrapped(small), 0.0, dt, true);
            if (!output.active) {
                stopped(output, "small-yaw travel exhaustion must disable both references");
                tripped = true;
                break;
            }
            small += output.small_velocity * dt;
        }
        require(tripped, "stalled big yaw must trip the captured small-yaw working range");
        require(std::abs(small - .2) <= parameters.max_small_offset_rad
                    + parameters.max_small_velocity_rad_s * dt + 1e-10,
                "small-yaw travel trip must occur within one control interval of its soft bound");
        const auto rearmed = controller.update(0.0, 0.0, wrapped(small), 0.0, dt, true);
        require(rearmed.active, "external rearming may establish a new working center");
        near(rearmed.target_angle, 0.0, "range fault rearming must discard its accumulated target");
        near(rearmed.small_offset, 0.0, "range fault rearming must capture a new center");
        near(rearmed.small_velocity, 0.0, "range fault rearming must not start old motion");
    }
}

void sustained_input_is_bounded_by_big_yaw_takeover_speed() {
    for (const double direction : {-1.0, 1.0}) {
        CoordinatedYawParameters parameters;
        CoordinatedYaw controller{parameters};
        const double dt = .01;
        controller.update(0.0, 0.0, 0.0, 0.0, dt, true);
        for (int tick = 1; tick <= 5; ++tick) {
            const auto output = controller.update(
                direction * 4.0 * parameters.max_big_velocity_rad_s, 0.0, 0.0, 0.0, dt, true);
            require(output.active, "a finite excessive input should be limited rather than faulted");
            require(output.reference_limited, "input-rate limiting must be visible in telemetry");
            near(output.target_angle, direction * parameters.max_big_velocity_rad_s * dt * tick,
                 "angle integration must use the rate big yaw can eventually take over");
        }
    }
}

CoordinatedYawParameters directional_envelope_parameters(double sign = 1.0) {
    CoordinatedYawParameters parameters;
    parameters.small_yaw_direction = sign;
    parameters.small_speed_zero_offset_rad = kPi / 6.0;
    parameters.max_small_velocity_rad_s = 1.771875;
    parameters.max_big_velocity_rad_s = 1.5;
    parameters.follow_gain = 3.2;
    return parameters;
}

CoordinatedYawParameters high_speed_envelope_parameters(double sign = 1.0) {
    auto parameters = directional_envelope_parameters(sign);
    parameters.small_speed_zero_offset_rad = kPi / 4.0;
    parameters.max_small_velocity_rad_s = 2.6578125;
    parameters.max_big_velocity_rad_s = 2.25;
    parameters.follow_gain = 3.2;
    parameters.max_small_acceleration_rad_s2 = 3.0;
    parameters.max_big_acceleration_rad_s2 = 1.5;
    parameters.small_braking_response_time_s = .32;
    return parameters;
}

CoordinatedYawParameters delayed_follow_parameters(double sign = 1.0) {
    auto parameters = high_speed_envelope_parameters(sign);
    parameters.max_small_velocity_rad_s = 8.0;
    parameters.max_big_velocity_rad_s = 4.5;
    parameters.follow_gain = 6.4;
    parameters.centered_follow_gain = 3.2;
    parameters.follow_start_offset_rad = kPi / 36.0;
    parameters.max_small_acceleration_rad_s2 = 6.0;
    parameters.max_big_acceleration_rad_s2 = 3.0;
    parameters.counterrotation_speed_fraction = .85;
    return parameters;
}

// Move joint and base feedback together so the composed heading stays zero.
// This isolates relative-angle limits without adding an unrelated goal error.
CoordinatedYawOutput place_at_relative_offset(CoordinatedYaw& controller,
                                             const CoordinatedYawParameters& parameters,
                                             double offset, double dt) {
    auto output = controller.update(0.0, 0.0, 0.0, 0.0, dt, true);
    const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(offset) / .01)));
    for (int tick = 1; tick <= steps; ++tick) {
        const double position = offset * tick / steps;
        output = controller.update(0.0, -parameters.small_yaw_direction * position,
                                   wrapped(position), 0.0, dt, true);
        require(output.active, "placing a valid relative offset must retain active references");
        near(output.heading_angle, 0.0, "relative-limit setup must preserve zero composed heading", 1e-12);
    }
    return output;
}

void directional_envelope_slows_outward_motion_and_retains_inward_authority() {
    constexpr double dt = .005;
    for (const double sign : {-1.0, 1.0}) {
        const auto parameters = directional_envelope_parameters(sign);
        for (const double side : {-1.0, 1.0}) {
            double previous_outward_speed = parameters.max_small_velocity_rad_s + 1.0;
            for (const double fraction : {0.0, .25, .6, .9, 1.0}) {
                const double offset = side * fraction * parameters.small_speed_zero_offset_rad;
                CoordinatedYaw controller{parameters};
                auto output = place_at_relative_offset(controller, parameters, offset, dt);
                for (int tick = 0; tick < 400; ++tick) {
                    output = controller.update(sign * side * parameters.max_big_velocity_rad_s,
                        -sign * offset, wrapped(offset), 0.0, dt, true);
                    require(output.active, "strong outward demand at a valid offset must be limited rather than faulted");
                }
                const double outward_speed = side * output.small_velocity;
                require(outward_speed >= -1e-12 && outward_speed < previous_outward_speed - 1e-5,
                        "available outward speed must decrease strictly as relative deflection grows");
                require(output.reference_limited, "directional limiting must be visible to the caller");
                if (fraction < 1.0)
                    require(outward_speed > .01, "positions inside the zero-speed boundary must retain outward authority");
                else {
                    near(outward_speed, 0.0, "outward demand must be zero at the configured relative boundary", 1e-12);
                    for (int tick = 0; tick < 600; ++tick) {
                        const auto previous = output;
                        output = controller.update(-sign * side * parameters.max_big_velocity_rad_s,
                            -sign * offset, wrapped(offset), 0.0, dt, true);
                        reference_bounds(output, previous, parameters, dt);
                    }
                    near(side * output.small_velocity, -parameters.max_small_velocity_rad_s,
                         "reversing input at the boundary must retain full inward authority", 1e-9);
                }
                previous_outward_speed = outward_speed;
            }
        }
    }
}

void actual_big_velocity_and_heading_error_cannot_bypass_the_directional_boundary() {
    constexpr double dt = .005;
    for (const double sign : {-1.0, 1.0}) {
        const auto parameters = directional_envelope_parameters(sign);
        for (const double side : {-1.0, 1.0}) {
            const double offset = side * parameters.small_speed_zero_offset_rad;
            CoordinatedYaw controller{parameters};
            auto output = place_at_relative_offset(controller, parameters, offset, dt);
            // A base motion that needs inward small-yaw compensation must
            // remain compensated even at the relative-angle boundary.
            for (int tick = 0; tick < 100; ++tick) {
                const auto previous = output;
                output = controller.update(0.0, -sign * offset, wrapped(offset), sign * side * .3, dt, true);
                reference_bounds(output, previous, parameters, dt);
                near(output.target_angle, 0.0, "boundary compensation must retain the held zero user goal");
            }
            near(side * output.small_velocity, -.3,
                 "inward actual-big compensation must survive the directional speed envelope");
            for (int tick = 0; tick < 100; ++tick) {
                const auto previous = output;
                output = controller.update(0.0, -sign * offset, wrapped(offset), -sign * side * .3, dt, true);
                reference_bounds(output, previous, parameters, dt);
                require(side * output.small_velocity <= 1e-12,
                        "opposite actual-big compensation must never create outward motion at the boundary");
            }
            near(output.small_velocity, 0.0,
                 "actual-big compensation demanding outward travel must stop at the boundary");
            for (int tick = 0; tick < 400; ++tick) {
                output = controller.update(sign * side * parameters.max_big_velocity_rad_s,
                    -sign * offset, wrapped(offset), -sign * side * 2.0, dt, true);
                require(output.active && output.reference_limited,
                        "combined strong feedback and velocity compensation must be limited without a false fault");
                near(output.small_velocity, 0.0,
                     "neither stored heading error nor actual-big feedforward may bypass the outward stop");
            }
            require(std::abs(output.heading_error) > .2,
                    "boundary regression must exercise substantial stored heading error");
            const double held_target = output.target_angle;
            for (int tick = 0; tick < 100; ++tick) {
                output = controller.update(0.0, -sign * offset, wrapped(offset), -sign * side * 2.0, dt, true);
                require(output.active, "centered boundary compensation must remain valid");
                near(output.target_angle, held_target,
                     "directional limiting and strong compensation must not move a centered user goal", 1e-12);
                near(output.small_velocity, 0.0,
                     "centered strong compensation must still obey the outward stop");
            }
        }
    }
}

void external_boundary_projection_preserves_bounded_feedforward_and_recovery() {
    constexpr double dt = .005;
    for (const double sign : {-1.0, 1.0}) {
        const auto parameters = directional_envelope_parameters(sign);
        for (const double side : {-1.0, 1.0}) {
            CoordinatedYaw controller{parameters};
            const double near_boundary = side * .5;
            auto output = place_at_relative_offset(controller, parameters, near_boundary, dt);
            for (int tick = 0; tick < 300; ++tick)
                output = controller.update(sign * side * parameters.max_big_velocity_rad_s,
                    -sign * near_boundary, wrapped(near_boundary), 0.0, dt, true);
            require(side * output.small_velocity > .1,
                    "projection regression must begin with a meaningful outward velocity reference");
            const auto previous = output;
            const double boundary = side * parameters.small_speed_zero_offset_rad;
            // This is a legal encoder step but insufficient braking distance.
            // The directional boundary takes priority over reference slew.
            output = controller.update(sign * side * parameters.max_big_velocity_rad_s,
                -sign * boundary, wrapped(boundary), 0.0, dt, true);
            require(output.active && output.reference_limited,
                    "boundary projection must stay active and report limiting, preserving hardware permission");
            near(output.small_velocity, 0.0, "external arrival at the boundary must immediately remove outward demand");
            require(std::abs(output.small_velocity - previous.small_velocity)
                        > parameters.max_small_acceleration_rad_s2 * dt,
                    "projection regression must actually exercise the infeasible acceleration intersection");
            require(std::abs(output.small_acceleration) <= parameters.max_small_acceleration_rad_s2,
                    "projection must keep inertial feedforward within the hardware validity bound");
            for (int tick = 0; tick < 600; ++tick) {
                const auto prior = output;
                output = controller.update(-sign * side * parameters.max_big_velocity_rad_s,
                    -sign * boundary, wrapped(boundary), 0.0, dt, true);
                reference_bounds(output, prior, parameters, dt);
            }
            require(side * output.small_velocity < -.5,
                    "inward recovery must remain possible after emergency outward projection");
        }
    }
}

void directional_envelope_retains_sustained_handoff(double sign, double direction, double lag_s) {
    const auto parameters = directional_envelope_parameters(sign);
    CoordinatedYaw controller{parameters};
    constexpr double dt = .005;
    const double response = lag_s == 0.0 ? 1.0 : 1.0 - std::exp(-dt / lag_s);
    double big = 0.0, small = 0.0, actual_big = 0.0, actual_small = 0.0;
    double maximum_offset = 0.0, held_target = 0.0;
    bool boundary_visited = false;
    auto output = controller.update(0.0, big, small, 0.0, dt, true);
    for (int tick = 0; tick < 4000; ++tick) {
        const auto previous = output;
        output = controller.update(tick < 2000 ? direction * parameters.max_big_velocity_rad_s : 0.0,
                                   big, wrapped(small), actual_big, dt, true);
        require(output.active, "enabled envelope must retain the known ideal and mild-lag sustained handoffs");
        require(std::abs(output.small_velocity) <= parameters.max_small_velocity_rad_s + 1e-10,
                "enabled envelope must retain the global small-yaw velocity bound");
        require(std::abs(output.small_acceleration) <= parameters.max_small_acceleration_rad_s2 + 1e-10,
                "enabled envelope must retain bounded hardware acceleration feedforward");
        if (lag_s == 0.0)
            reference_bounds(output, previous, parameters, dt);
        if (std::abs(output.small_offset) >= parameters.small_speed_zero_offset_rad - 1e-10) {
            boundary_visited = true;
            require(output.small_offset * output.small_velocity <= 1e-9,
                    "at or beyond the relative boundary the command must not continue outward");
        }
        if (tick == 1999) {
            held_target = output.target_angle;
            require(direction * held_target > 12.0,
                    "long enabled full-stick input must retain substantial big-yaw takeover progress");
        }
        if (tick >= 2000)
            near(output.target_angle, held_target,
                 "centered input must exactly freeze the target throughout envelope-limited handoff", 1e-12);
        actual_big += response * (output.big_velocity - actual_big);
        actual_small += response * (output.small_velocity - actual_small);
        big += actual_big * dt;
        small += actual_small * dt;
        maximum_offset = std::max(maximum_offset, std::abs(small));
    }
    require(boundary_visited, "sustained full-stick regression must actually exercise the relative boundary");
    require(maximum_offset < parameters.max_small_offset_rad,
            "enabled mild-lag plants must avoid exhausting the independent hard working range");
    if (lag_s == 0.0)
        require(maximum_offset <= parameters.small_speed_zero_offset_rad + 1e-9,
                "ideal reference execution must stop within the configured relative boundary");
    near(big + sign * small, held_target,
         "envelope-limited release must eventually settle at its accepted heading", .01);
    near(small, 0.0, "envelope-limited release must return small yaw to the captured center",
         parameters.follow_deadband_rad + .005);
}

void enabled_envelope_does_not_replace_the_hard_working_range_fault() {
    for (const double sign : {-1.0, 1.0}) {
        const auto parameters = directional_envelope_parameters(sign);
        for (const double side : {-1.0, 1.0}) {
            CoordinatedYaw controller{parameters};
            constexpr double dt = .005;
            controller.update(0.0, 0.0, 0.0, 0.0, dt, true);
            bool tripped = false;
            double position = 0.0;
            for (int tick = 1; tick <= 100; ++tick) {
                position = side * .01 * tick;
                const auto output = controller.update(0.0, -sign * position,
                                                       wrapped(position), 0.0, dt, true);
                if (!output.active) {
                    stopped(output, "external motion beyond the hard range must still fail closed with the envelope enabled");
                    tripped = true;
                    break;
                }
            }
            require(tripped && std::abs(position) > parameters.max_small_offset_rad,
                    "directional speed limiting must not suppress the independent hard-angle fault");
            const auto rearmed = controller.update(0.0, -sign * position, wrapped(position), 0.0, dt, true);
            require(rearmed.active, "external owner may rearm after the hard-angle fault");
            near(rearmed.small_velocity, 0.0, "hard-angle rearm must not restart the old command");
            near(rearmed.target_angle, 0.0, "hard-angle rearm must clear the old user target");
        }
    }
}

void high_speed_boundary_retains_inward_authority_and_bounded_feedforward() {
    constexpr double dt = .005;
    for (const double sign : {-1.0, 1.0}) {
        const auto parameters = high_speed_envelope_parameters(sign);
        for (const double side : {-1.0, 1.0}) {
            CoordinatedYaw controller{parameters};
            const double near_boundary = side * (parameters.small_speed_zero_offset_rad - .03);
            auto output = place_at_relative_offset(controller, parameters, near_boundary, dt);
            for (int tick = 0; tick < 300; ++tick)
                output = controller.update(sign * side * parameters.max_big_velocity_rad_s,
                    -sign * near_boundary, wrapped(near_boundary), -sign * side * .3, dt, true);
            require(side * output.small_velocity > .05,
                    "high-speed projection must begin with an outward reference inside the braking boundary");
            const auto previous = output;
            const double boundary = side * parameters.small_speed_zero_offset_rad;
            output = controller.update(sign * side * parameters.max_big_velocity_rad_s,
                -sign * boundary, wrapped(boundary), -sign * side * .3, dt, true);
            require(output.active && output.reference_limited,
                    "high-speed boundary projection must retain a valid limited control reference");
            near(output.small_velocity, 0.0, "45-degree boundary must stop all combined outward demand");
            require(std::abs(output.small_velocity - previous.small_velocity)
                        > parameters.max_small_acceleration_rad_s2 * dt,
                    "high-speed boundary regression must exercise acceleration-infeasible projection");
            require(std::abs(output.small_acceleration) <= parameters.max_small_acceleration_rad_s2,
                    "high-speed projection inertial feedforward must stay within 3 rad/s squared");
            const double held_target = output.target_angle;
            for (int tick = 0; tick < 300; ++tick) {
                output = controller.update(0.0, -sign * boundary, wrapped(boundary),
                                           -sign * side * .3, dt, true);
                require(output.active, "centered high-speed boundary hold must remain valid");
                near(output.target_angle, held_target,
                     "high-speed directional projection must not drag the centered user target", 1e-12);
                near(output.small_velocity, 0.0,
                     "centered error and outward base compensation must obey the 45-degree stop");
            }
            for (int tick = 0; tick < 400; ++tick) {
                const auto prior = output;
                output = controller.update(-sign * side * parameters.max_big_velocity_rad_s,
                    -sign * boundary, wrapped(boundary), sign * side * .3, dt, true);
                reference_bounds(output, prior, parameters, dt);
            }
            require(side * output.small_velocity < -.5,
                    "45-degree limit must preserve inward recovery under opposite-side braking");
            require(std::abs(output.small_velocity) <= parameters.max_small_velocity_rad_s,
                    "inward recovery must retain the configured high-speed bound");
        }
    }
}

void high_speed_response_margin_retains_sustained_handoff(double direction, double lag_s) {
    const auto parameters = high_speed_envelope_parameters();
    CoordinatedYaw controller{parameters};
    constexpr double dt = .005;
    const double response = lag_s == 0.0 ? 1.0 : 1.0 - std::exp(-dt / lag_s);
    double big = 0.0, small = 0.0, actual_big = 0.0, actual_small = 0.0;
    double maximum_offset = 0.0, held_target = 0.0;
    auto output = controller.update(0.0, big, small, 0.0, dt, true);
    for (int tick = 0; tick < 4000; ++tick) {
        const auto previous = output;
        output = controller.update(tick < 2000 ? direction * parameters.max_big_velocity_rad_s : 0.0,
                                   big, wrapped(small), actual_big, dt, true);
        require(output.active,
                "high-speed response margin must retain ideal and 40/80 ms full-stick handoffs");
        require(std::abs(output.small_velocity) <= parameters.max_small_velocity_rad_s + 1e-10
                    && std::abs(output.big_velocity) <= parameters.max_big_velocity_rad_s + 1e-10,
                "high-speed handoff references must retain both configured velocity bounds");
        require(std::abs(output.small_acceleration) <= parameters.max_small_acceleration_rad_s2 + 1e-10
                    && std::abs(output.big_acceleration) <= parameters.max_big_acceleration_rad_s2 + 1e-10,
                "high-speed handoff inertial feedforward must remain valid for hardware permission");
        if (lag_s == 0.0)
            reference_bounds(output, previous, parameters, dt);
        if (std::abs(output.small_offset) >= parameters.small_speed_zero_offset_rad - 1e-10)
            require(output.small_offset * output.small_velocity <= 1e-9,
                    "high-speed handoff must never command outward motion beyond 45 degrees");
        if (tick == 1999) {
            held_target = output.target_angle;
            require(direction * held_target > 18.0,
                    "high-speed sustained input must retain substantial progress at the new mapped speed");
        }
        if (tick >= 2000)
            near(output.target_angle, held_target,
                 "high-speed release must exactly freeze the accepted target", 1e-12);
        actual_big += response * (output.big_velocity - actual_big);
        actual_small += response * (output.small_velocity - actual_small);
        big += actual_big * dt;
        small += actual_small * dt;
        maximum_offset = std::max(maximum_offset, std::abs(small));
    }
    require(maximum_offset < parameters.max_small_offset_rad,
            "high-speed response margin must preserve the narrow remaining hard-range margin");
    near(big + small, held_target, "high-speed handoff must settle at its frozen accepted heading", .01);
    near(small, 0.0, "high-speed handoff must return small yaw to its captured center",
         parameters.follow_deadband_rad + .005);
}

void high_speed_asynchronous_big_angles_remain_valid_and_jumps_still_fault() {
    constexpr double dt = .001;
    for (const double direction : {-1.0, 1.0}) {
        const auto parameters = high_speed_envelope_parameters();
        CoordinatedYaw controller{parameters};
        double big = 0.0;
        auto output = controller.update(0.0, big, 0.0, 0.0, dt, true);
        for (int tick = 1; tick <= 200; ++tick) {
            if (tick % 20 == 0)
                big += direction * parameters.max_big_velocity_rad_s * .02;
            output = controller.update(0.0, big, 0.0,
                                       direction * parameters.max_big_velocity_rad_s, dt, true);
            require(output.active,
                    "legal 2.25 rad/s big-yaw feedback at 20 ms CAN intervals must survive a 1 kHz loop");
            near(output.target_angle, 0.0,
                 "asynchronous high-speed feedback must not move a centered user target", 1e-12);
            require(std::abs(output.small_velocity) <= parameters.max_small_velocity_rad_s
                        && std::abs(output.small_acceleration) <= parameters.max_small_acceleration_rad_s2,
                    "asynchronous feedback must retain valid small-yaw velocity and inertial feedforward");
        }
        stopped(controller.update(0.0, big + direction * .08, 0.0, 0.0, dt, true),
                "an impossible jump immediately after a refreshed angle must still fault");
        controller.reset();
        controller.update(0.0, 0.0, 0.0, 0.0, dt, true);
        for (int tick = 0; tick < 100; ++tick)
            require(controller.update(0.0, 0.0, 0.0, 0.0, dt, true).active,
                    "unchanged angle alone must remain valid input");
        stopped(controller.update(0.0, direction * .16, 0.0, 0.0, dt, true),
                "unchanged-angle accumulation must not allow unlimited future discontinuity");
        CoordinatedYaw legacy;
        legacy.update(0.0, 0.0, 0.0, 0.0, dt, true);
        require(legacy.update(0.0, direction * .031, 0.0, 0.0, dt, true).active,
                "legacy immediate angle-step allowance must remain valid");
        legacy.reset();
        legacy.update(0.0, 0.0, 0.0, 0.0, dt, true);
        stopped(legacy.update(0.0, direction * .033, 0.0, 0.0, dt, true),
                "legacy immediate discontinuity bound must not be globally widened by high-speed support");
    }
}

void high_speed_asynchronous_small_angles_preserve_wrap_and_reject_jumps() {
    constexpr double dt = .001;
    for (const double direction : {-1.0, 1.0}) {
        const auto parameters = high_speed_envelope_parameters();
        CoordinatedYaw controller{parameters};
        const double center = direction * (kPi - .02);
        double offset = 0.0;
        auto output = controller.update(0.0, 0.0, wrapped(center), 0.0, dt, true);
        // Twelve real 20 ms CAN intervals cross the encoder wrap while staying
        // inside the relative-angle boundary. The control loop runs at 1 kHz.
        for (int tick = 1; tick <= 240; ++tick) {
            if (tick % 20 == 0)
                offset += direction * parameters.max_small_velocity_rad_s * .02;
            output = controller.update(0.0, 0.0, wrapped(center + offset), 0.0, dt, true);
            require(output.active,
                    "legal high-speed small-yaw CAN steps must survive asynchronous control and encoder wrap");
            near(output.small_center_angle, wrapped(center),
                 "asynchronous small-yaw updates must not recapture or drag the mechanical center", 1e-12);
            near(output.small_offset, offset,
                 "asynchronous small-yaw unwrap must retain the physical accumulated relative angle", 1e-10);
            near(output.target_angle, 0.0,
                 "asynchronous external small-yaw motion must preserve the frozen user goal", 1e-12);
        }
        require(std::abs(offset) < parameters.small_speed_zero_offset_rad,
                "small-yaw async regression must remain within the intended angle envelope");
        stopped(controller.update(0.0, 0.0, wrapped(center + offset + direction * .04), 0.0, dt, true),
                "fresh impossible small-yaw angle jump must still fault after a legal CAN update");
        controller.reset();
        controller.update(0.0, 0.0, wrapped(center), 0.0, dt, true);
        for (int tick = 0; tick < 100; ++tick)
            require(controller.update(0.0, 0.0, wrapped(center), 0.0, dt, true).active,
                    "unchanged small-yaw feedback alone must remain valid");
        stopped(controller.update(0.0, 0.0, wrapped(center + direction * .18), 0.0, dt, true),
                "small-yaw elapsed allowance must remain capped at fifty milliseconds");
        CoordinatedYaw legacy;
        legacy.update(0.0, 0.0, wrapped(center), 0.0, dt, true);
        require(legacy.update(0.0, 0.0, wrapped(center + direction * .031), 0.0, dt, true).active,
                "legacy immediate small-yaw angle allowance must remain valid");
        legacy.reset();
        legacy.update(0.0, 0.0, wrapped(center), 0.0, dt, true);
        stopped(legacy.update(0.0, 0.0, wrapped(center + direction * .032), 0.0, dt, true),
                "legacy immediate small-yaw discontinuity bound must not widen with high-speed support");
    }
}

void zero_response_margin_disables_only_the_added_braking_lookahead() {
    constexpr double dt = .005;
    near(CoordinatedYawParameters{}.small_braking_response_time_s, 0.0,
         "legacy default must leave response lookahead disabled");
    auto without_response = high_speed_envelope_parameters();
    without_response.small_braking_response_time_s = 0.0;
    CoordinatedYaw baseline{without_response};
    const auto with_response = high_speed_envelope_parameters();
    CoordinatedYaw conservative{with_response};
    baseline.update(0.0, 0.0, 0.0, 0.0, dt, true);
    conservative.update(0.0, 0.0, 0.0, 0.0, dt, true);
    CoordinatedYawOutput baseline_output, conservative_output;
    for (int tick = 0; tick < 400; ++tick) {
        baseline_output = baseline.update(without_response.max_big_velocity_rad_s, 0.0, 0.0, 0.0, dt, true);
        conservative_output = conservative.update(with_response.max_big_velocity_rad_s, 0.0, 0.0, 0.0, dt, true);
        require(baseline_output.active && conservative_output.active,
                "both disabled and enabled response-margin settings must be valid");
    }
    require(baseline_output.small_velocity > conservative_output.small_velocity + .1,
            "zero response margin must remove only the added early-braking restriction");
}

void follow_start_threshold_latches_until_the_joint_has_recentered() {
    constexpr double dt = .005;
    for (const double sign : {-1.0, 1.0}) {
        const auto parameters = delayed_follow_parameters(sign);
        for (const double side : {-1.0, 1.0}) {
            CoordinatedYaw controller{parameters};
            auto output = controller.update(0.0, 0.0, 0.0, 0.0, dt, true);
            for (const double magnitude : {.01, .04, .07, .08,
                                           parameters.follow_start_offset_rad - .0001}) {
                const double offset = side * magnitude;
                output = controller.update(0.0, -sign * offset, wrapped(offset), 0.0, dt, true);
                require(output.active, "valid displacement below the follow threshold must remain active");
                near(output.big_velocity, 0.0,
                     "big yaw must remain stopped before small yaw first crosses five degrees");
            }
            const double triggered = side * (parameters.follow_start_offset_rad + .0001);
            for (int tick = 0; tick < 100; ++tick)
                output = controller.update(0.0, -sign * triggered, wrapped(triggered), 0.0, dt, true);
            require(sign * side * output.big_velocity > .1,
                    "crossing the follow threshold must start big yaw in the matching physical direction");
            const double returning = side * .04;
            for (int tick = 0; tick < 100; ++tick)
                output = controller.update(0.0, -sign * returning, wrapped(returning), 0.0, dt, true);
            require(sign * side * output.big_velocity > .05,
                    "latched big-yaw following must continue inside five degrees until true recentering");
            near(sign * side * output.big_velocity,
                 parameters.centered_follow_gain * (.04 - parameters.follow_deadband_rad),
                 "centered input must use the configured lower recentering gain", 1e-9);
            for (int tick = 0; tick < 100; ++tick)
                output = controller.update(0.0, 0.0, 0.0, 0.0, dt, true);
            for (int tick = 0; tick < 100; ++tick)
                output = controller.update(0.0, sign * returning, wrapped(-returning), 0.0, dt, true);
            near(output.big_velocity, 0.0,
                 "after stationary recentering a reverse displacement below five degrees must not restart big yaw");
            const double reversed = -triggered;
            for (int tick = 0; tick < 100; ++tick)
                output = controller.update(0.0, -sign * reversed, wrapped(reversed), 0.0, dt, true);
            require(sign * side * output.big_velocity < -.1,
                    "crossing the opposite threshold must restart following in the reverse direction");
            controller.reset();
            controller.update(0.0, 0.0, 0.0, 0.0, dt, true);
            output = controller.update(0.0, -sign * returning, wrapped(returning), 0.0, dt, true);
            near(output.big_velocity, 0.0, "reset must clear the delayed-follow latch");
        }
    }
}

void delayed_follow_waits_for_actual_and_reference_big_yaw_to_stop() {
    constexpr double dt = .005;
    for (const double sign : {-1.0, 1.0}) {
        const auto parameters = delayed_follow_parameters(sign);
        for (const double side : {-1.0, 1.0}) {
            for (const bool base_still_moving : {false, true}) {
                CoordinatedYaw controller{parameters};
                auto output = place_at_relative_offset(controller, parameters, side * .2, dt);
                for (int tick = 0; tick < 100; ++tick)
                    output = controller.update(0.0, -sign * side * .2, side * .2, 0.0, dt, true);
                require(sign * side * output.big_velocity > .5,
                        "latch-clear regression must begin with substantial big-yaw following");
                for (int step = 19; step >= 0; --step) {
                    const double offset = side * .01 * step;
                    output = controller.update(0.0, -sign * offset, wrapped(offset),
                        base_still_moving ? sign * side * .3 : 0.0, dt, true);
                }
                if (base_still_moving) {
                    for (int tick = 0; tick < 100; ++tick)
                        output = controller.update(0.0, 0.0, 0.0, sign * side * .3, dt, true);
                    near(output.big_velocity, 0.0,
                         "measured-motion latch case must finish decelerating the big-yaw reference");
                } else {
                    require(std::abs(output.big_velocity) > parameters.big_velocity_noise_threshold_rad_s,
                            "reference-motion latch case must reach center before the reference has stopped");
                }
                // Leave center before the relevant stop condition can clear:
                // actual motion for one case, previous reference for the other.
                for (int step = 1; step <= 4; ++step) {
                    const double offset = side * .01 * step;
                    output = controller.update(0.0, -sign * offset, wrapped(offset),
                        base_still_moving ? sign * side * .3 : 0.0, dt, true);
                }
                for (int tick = 0; tick < 100; ++tick)
                    output = controller.update(0.0, -sign * side * .04, side * .04,
                        base_still_moving ? sign * side * .3 : 0.0, dt, true);
                require(sign * side * output.big_velocity > .05,
                        "actual or reference motion at center must prevent prematurely clearing the follow latch");
            }
        }
    }
}

void moving_input_retains_main_follow_gain_and_zero_centered_gain_falls_back() {
    constexpr double dt = .005;
    auto parameters = delayed_follow_parameters();
    CoordinatedYaw centered{parameters}, moving{parameters};
    auto centered_output = place_at_relative_offset(centered, parameters, .2, dt);
    auto moving_output = place_at_relative_offset(moving, parameters, .2, dt);
    for (int tick = 0; tick < 150; ++tick) {
        centered_output = centered.update(0.0, -.2, .2, 0.0, dt, true);
        moving_output = moving.update(.1, -.2, .2, 0.0, dt, true);
    }
    require(moving_output.big_velocity > centered_output.big_velocity + .5,
            "nonzero user input must use the stronger main follow gain");
    parameters.centered_follow_gain = 0.0;
    CoordinatedYaw fallback{parameters};
    auto output = place_at_relative_offset(fallback, parameters, .2, dt);
    for (int tick = 0; tick < 150; ++tick)
        output = fallback.update(0.0, -.2, .2, 0.0, dt, true);
    near(output.big_velocity, moving_output.big_velocity,
         "zero centered follow gain must preserve the original main-gain behavior");
    parameters.follow_start_offset_rad = 0.0;
    CoordinatedYaw always_follow{parameters};
    output = place_at_relative_offset(always_follow, parameters, .04, dt);
    for (int tick = 0; tick < 100; ++tick)
        output = always_follow.update(0.0, -.04, .04, 0.0, dt, true);
    require(output.big_velocity > .1,
            "zero follow-start offset must preserve always-follow behavior below five degrees");
}

void fast_motor_feedback_at_fifty_millisecond_intervals_is_valid() {
    constexpr double dt = .001;
    for (const double direction : {-1.0, 1.0}) {
        const auto parameters = delayed_follow_parameters();
        CoordinatedYaw big_controller{parameters}, small_controller{parameters};
        double big = 0.0, small = 0.0;
        big_controller.update(0.0, big, 0.0, 0.0, dt, true);
        small_controller.update(0.0, 0.0, small, 0.0, dt, true);
        for (int tick = 1; tick <= 100; ++tick) {
            if (tick % 50 == 0) {
                big += direction * parameters.max_big_velocity_rad_s * .05;
                small = tick == 50 ? direction * parameters.max_small_velocity_rad_s * .05 : 0.0;
            }
            const auto big_output = big_controller.update(0.0, big, 0.0,
                direction * parameters.max_big_velocity_rad_s, dt, true);
            require(big_output.active, "4.5 rad/s big-yaw CAN steps at fifty milliseconds must remain valid");
            near(big_output.target_angle, 0.0, "fast big-yaw CAN steps must preserve a centered user goal");
            const auto small_output = small_controller.update(0.0, 0.0, small, 0.0, dt, true);
            require(small_output.active, "8 rad/s small-yaw CAN steps at fifty milliseconds must remain valid");
            near(small_output.target_angle, 0.0, "fast small-yaw CAN steps must preserve a centered user goal");
        }
        stopped(big_controller.update(0.0, big + direction * .3, 0.0, 0.0, dt, true),
                "fast big-yaw continuity allowance must still reject a fresh impossible jump");
        stopped(small_controller.update(0.0, 0.0, direction * .1, 0.0, dt, true),
                "fast small-yaw continuity allowance must still reject a fresh impossible jump");
    }
}

void big_following_respects_directional_small_yaw_counterrotation_capacity() {
    constexpr double dt = .005;
    near(CoordinatedYawParameters{}.counterrotation_speed_fraction, 0.0,
         "counterrotation limiting must remain disabled by default for legacy configurations");
    for (const double sign : {-1.0, 1.0}) {
        auto parameters = delayed_follow_parameters(sign);
        // Use the main gain for centered input here to expose saturation
        // without accumulating a separate user angle target.
        parameters.centered_follow_gain = 0.0;
        auto unrestricted_parameters = parameters;
        unrestricted_parameters.counterrotation_speed_fraction = 0.0;
        for (const double side : {-1.0, 1.0}) {
            const double boundary = side * parameters.small_speed_zero_offset_rad;
            CoordinatedYaw limited{parameters}, unrestricted{unrestricted_parameters};
            auto limited_output = place_at_relative_offset(limited, parameters, boundary, dt);
            auto unrestricted_output = place_at_relative_offset(unrestricted, unrestricted_parameters, boundary, dt);
            for (int tick = 0; tick < 400; ++tick) {
                const auto previous = limited_output;
                limited_output = limited.update(0.0, -sign * boundary, wrapped(boundary), 0.0, dt, true);
                unrestricted_output = unrestricted.update(0.0, -sign * boundary, wrapped(boundary), 0.0, dt, true);
                reference_bounds(limited_output, previous, parameters, dt);
            }
            require(sign * side * limited_output.big_velocity > 1.0,
                    "boundary follow must use inward small-yaw capacity rather than the zero outward capacity");
            require(std::abs(limited_output.big_velocity) + .5 < std::abs(unrestricted_output.big_velocity),
                    "enabled counterrotation reserve must limit big-yaw follow speed before the nominal speed cap");
            near(std::abs(unrestricted_output.big_velocity), parameters.max_big_velocity_rad_s,
                 "zero counterrotation fraction must preserve unrestricted legacy following");
            require(limited_output.reference_limited,
                    "counterrotation-capacity limiting must be visible in reference telemetry");
            const double physical_big_velocity = limited_output.big_velocity;
            for (int tick = 0; tick < 600; ++tick) {
                const auto previous = limited_output;
                limited_output = limited.update(0.0, -sign * boundary, wrapped(boundary),
                                                physical_big_velocity, dt, true);
                reference_bounds(limited_output, previous, parameters, dt);
            }
            require(side * limited_output.small_velocity < -1.0,
                    "capacity-limited big-yaw motion must remain compensable in the inward joint direction");
            near(sign * limited_output.small_velocity, -physical_big_velocity,
                 "capacity-limited big-yaw speed must leave enough inward compensation authority", 1e-8);
            near(limited_output.target_angle, 0.0,
                 "counterrotation limiting and actual velocity compensation must not move the centered goal");
        }
    }
}

void delayed_follow_capacity_reserve_retains_full_speed_release(double direction, double lag_s) {
    const auto parameters = delayed_follow_parameters();
    CoordinatedYaw controller{parameters};
    constexpr double dt = .005;
    const double response = lag_s == 0.0 ? 1.0 : 1.0 - std::exp(-dt / lag_s);
    double big = 0.0, small = 0.0, actual_big = 0.0, actual_small = 0.0;
    double held_target = 0.0, maximum_release_error = 0.0, maximum_offset = 0.0;
    auto output = controller.update(0.0, big, small, 0.0, dt, true);
    for (int tick = 0; tick < 4000; ++tick) {
        const auto previous = output;
        output = controller.update(tick < 2000 ? direction * parameters.max_big_velocity_rad_s : 0.0,
                                   big, wrapped(small), actual_big, dt, true);
        require(output.active,
                "delayed follow with counterrotation reserve must retain full-input ideal and lagged operation");
        require(std::abs(output.small_velocity) <= parameters.max_small_velocity_rad_s + 1e-10
                    && std::abs(output.big_velocity) <= parameters.max_big_velocity_rad_s + 1e-10,
                "capacity-reserved handoff must preserve both actuator velocity contracts");
        require(std::abs(output.small_acceleration) <= parameters.max_small_acceleration_rad_s2 + 1e-10
                    && std::abs(output.big_acceleration) <= parameters.max_big_acceleration_rad_s2 + 1e-10,
                "capacity-reserved handoff must preserve both hardware inertial feedforward contracts");
        if (lag_s == 0.0)
            reference_bounds(output, previous, parameters, dt);
        if (tick == 1999) {
            held_target = output.target_angle;
            require(direction * held_target > 20.0,
                    "counterrotation reserve must retain sustained physical heading progress");
        }
        if (tick >= 2000) {
            near(output.target_angle, held_target,
                 "capacity-reserved release must exactly freeze the accepted goal", 1e-12);
            maximum_release_error = std::max(maximum_release_error, std::abs(big + small - held_target));
        }
        actual_big += response * (output.big_velocity - actual_big);
        actual_small += response * (output.small_velocity - actual_small);
        big += actual_big * dt;
        small += actual_small * dt;
        maximum_offset = std::max(maximum_offset, std::abs(small));
    }
    require(maximum_release_error <= .36,
            "counterrotation reserve must bound full-input release pointing error in the documented lag plants");
    require(maximum_offset < parameters.max_small_offset_rad,
            "full-input counterrotation-reserved plants must not exhaust the hard small-yaw working range");
    near(big + small, held_target,
         "capacity-reserved handoff must settle to its frozen heading within ten seconds of release", .015);
    near(small, 0.0,
         "capacity-reserved handoff must recenter small yaw within ten seconds of release", .015);
}

void faults_clear_references_and_require_fresh_capture() {
    const auto check_bad = [](double rate, double big, double small, double measured,
                              double dt, bool permitted) {
        CoordinatedYaw controller;
        controller.update(0.0, 1.0, .5, 0.0, .01, true);
        controller.update(.5, 1.0, .5, 0.0, .01, true);
        stopped(controller.update(rate, big, small, measured, dt, permitted),
                "bad input or permission loss must immediately disable both references");
        const auto restored = controller.update(0.0, 2.0, -.4, 0.0, .01, true);
        require(restored.active, "owner may restore permission after its external rearming sequence");
        near(restored.target_angle, 0.0, "fault recovery must discard the old target");
        near(restored.heading_angle, 0.0, "fault recovery must capture a new origin");
        near(restored.small_velocity, 0.0, "fault recovery must not restore an old small-yaw reference");
        near(restored.big_velocity, 0.0, "fault recovery must not restore an old big-yaw reference");
    };
    check_bad(.5, 1.0, .5, 0.0, .01, false);
    for (const double invalid : {kNan, kInf, -kInf}) {
        check_bad(invalid, 1.0, .5, 0.0, .01, true);
        check_bad(.5, invalid, .5, 0.0, .01, true);
        check_bad(.5, 1.0, invalid, 0.0, .01, true);
        check_bad(.5, 1.0, .5, invalid, .01, true);
        check_bad(.5, 1.0, .5, 0.0, invalid, true);
    }
    for (const double invalid_dt : {0.0, -.01, .020001})
        check_bad(.5, 1.0, .5, 0.0, invalid_dt, true);
    check_bad(.5, 1.5, .5, 0.0, .01, true);
    check_bad(.5, 1.0, 1.0, 0.0, .01, true);
}

void invalid_parameters_are_rejected() {
    auto parameters = CoordinatedYawParameters{};
    parameters.max_big_velocity_rad_s = parameters.max_small_velocity_rad_s + .01;
    rejects([&] { CoordinatedYaw controller{parameters}; },
            "big-yaw follow speed must be within small-yaw compensation capability");
    parameters = {};
    parameters.max_big_acceleration_rad_s2 = parameters.max_small_acceleration_rad_s2 + .01;
    rejects([&] { CoordinatedYaw controller{parameters}; },
            "big-yaw follow acceleration must be within small-yaw compensation capability");
    for (const double invalid : {0.0, .5, kNan, kInf}) {
        parameters = {};
        parameters.small_yaw_direction = invalid;
        rejects([&] { CoordinatedYaw controller{parameters}; }, "kinematic direction must be finite +/-1");
    }
    parameters = {};
    parameters.follow_deadband_rad = -.001;
    rejects([&] { CoordinatedYaw controller{parameters}; }, "negative follow deadband must be rejected");
    parameters = {};
    parameters.max_target_error_rad = 0.0;
    rejects([&] { CoordinatedYaw controller{parameters}; }, "zero target lead bound must be rejected");
    for (const double invalid : {-.001, .35, kNan, kInf}) {
        parameters = {};
        parameters.heading_error_deadband_rad = invalid;
        rejects([&] { CoordinatedYaw controller{parameters}; },
                "heading deadband must be finite, nonnegative and smaller than the lead bound");
    }
    for (const double invalid : {-.001, kNan, kInf}) {
        parameters = {};
        parameters.big_velocity_noise_threshold_rad_s = invalid;
        rejects([&] { CoordinatedYaw controller{parameters}; },
                "big-yaw speed noise threshold must be finite and nonnegative");
    }
    parameters = {};
    const double minimum_offset = parameters.follow_deadband_rad
        + parameters.max_big_velocity_rad_s / parameters.follow_gain;
    for (const double invalid : {minimum_offset - .001, minimum_offset, kNan}) {
        parameters = {};
        parameters.max_small_offset_rad = invalid;
        rejects([&] { CoordinatedYaw controller{parameters}; },
                "small-yaw working range must be finite and exceed the full-speed follow offset");
    }
    parameters = directional_envelope_parameters();
    const double minimum_zero_offset = parameters.follow_deadband_rad
        + parameters.max_big_velocity_rad_s / parameters.follow_gain;
    for (const double invalid : {-.001, minimum_zero_offset, parameters.max_small_offset_rad, kNan, kInf}) {
        parameters = directional_envelope_parameters();
        parameters.small_speed_zero_offset_rad = invalid;
        rejects([&] { CoordinatedYaw controller{parameters}; },
                "enabled speed-zero offset must be finite, beyond steady follow offset and inside the hard range");
    }
    parameters = directional_envelope_parameters();
    parameters.small_speed_zero_offset_rad = 0.0;
    CoordinatedYaw disabled{parameters};
    require(disabled.update(0.0, 0.0, 0.0, 0.0, .005, true).active,
            "zero speed-offset setting must preserve the disabled legacy behavior");
    for (const double invalid : {-.001, kNan, kInf}) {
        parameters = high_speed_envelope_parameters();
        parameters.small_braking_response_time_s = invalid;
        rejects([&] { CoordinatedYaw controller{parameters}; },
                "braking response time must be finite and nonnegative");
    }
    for (const double invalid : {-.001, kNan, kInf}) {
        parameters = delayed_follow_parameters();
        parameters.centered_follow_gain = invalid;
        rejects([&] { CoordinatedYaw controller{parameters}; },
                "optional centered follow gain must be finite and nonnegative");
    }
    for (const double invalid : {-.001, .01, kPi / 4.0, .8, kNan, kInf}) {
        parameters = delayed_follow_parameters();
        parameters.follow_start_offset_rad = invalid;
        rejects([&] { CoordinatedYaw controller{parameters}; },
                "enabled follow-start offset must lie beyond the center deadband and inside the valid joint range");
    }
    for (const double invalid : {-.001, 1.001, kNan, kInf}) {
        parameters = delayed_follow_parameters();
        parameters.counterrotation_speed_fraction = invalid;
        rejects([&] { CoordinatedYaw controller{parameters}; },
                "counterrotation speed fraction must be finite and inside the closed unit interval");
    }
    parameters = delayed_follow_parameters();
    parameters.counterrotation_speed_fraction = 1.0;
    CoordinatedYaw full_capacity{parameters};
    require(full_capacity.update(0.0, 0.0, 0.0, 0.0, .005, true).active,
            "a counterrotation fraction of one must remain a valid full-capacity setting");
}

} // namespace

int main() {
    try {
        for (const double direction : {-1.0, 1.0})
            for (const double sign : {-1.0, 1.0})
                for (const double dt : {.001, .005})
                    thirty_degree_handoff(direction, sign, dt);
        capture_is_bumpless_and_wrap_is_continuous();
        compensation_uses_actual_big_yaw_velocity();
        standstill_speed_quantization_does_not_start_small_yaw();
        genuine_subthreshold_motion_remains_visible_to_position_feedback();
        external_motion_cannot_drag_a_centered_user_target();
        large_quantized_speed_feedback_cannot_accumulate_a_user_angle();
        heading_deadband_softens_only_centered_angle_feedback();
        actual_motion_compensation_survives_the_heading_deadband();
        stiction_does_not_hide_growing_heading_error();
        for (const double direction : {-1.0, 1.0}) {
            for (const double lag_s : {0.0, .04, .08})
                short_full_stick_stays_inside_its_known_range(direction, lag_s);
            for (const auto& [big_lag_s, small_lag_s] : {std::pair{.04, .08}, std::pair{.08, .04}})
                sustained_moderate_input_survives_asymmetric_lags(direction, big_lag_s, small_lag_s);
        }
        sustained_full_stick_with_large_lag_trips_the_working_range();
        normal_reversal_and_stop_respect_acceleration();
        blocked_plant_cannot_accumulate_unbounded_target();
        stalled_big_yaw_cannot_exhaust_small_yaw_travel();
        sustained_input_is_bounded_by_big_yaw_takeover_speed();
        faults_clear_references_and_require_fresh_capture();
        invalid_parameters_are_rejected();
        directional_envelope_slows_outward_motion_and_retains_inward_authority();
        actual_big_velocity_and_heading_error_cannot_bypass_the_directional_boundary();
        external_boundary_projection_preserves_bounded_feedforward_and_recovery();
        for (const double sign : {-1.0, 1.0})
            for (const double direction : {-1.0, 1.0})
                for (const double lag_s : {0.0, .04, .08})
                    directional_envelope_retains_sustained_handoff(sign, direction, lag_s);
        enabled_envelope_does_not_replace_the_hard_working_range_fault();
        high_speed_boundary_retains_inward_authority_and_bounded_feedforward();
        for (const double direction : {-1.0, 1.0})
            for (const double lag_s : {0.0, .04, .08})
                high_speed_response_margin_retains_sustained_handoff(direction, lag_s);
        high_speed_asynchronous_big_angles_remain_valid_and_jumps_still_fault();
        high_speed_asynchronous_small_angles_preserve_wrap_and_reject_jumps();
        zero_response_margin_disables_only_the_added_braking_lookahead();
        follow_start_threshold_latches_until_the_joint_has_recentered();
        delayed_follow_waits_for_actual_and_reference_big_yaw_to_stop();
        moving_input_retains_main_follow_gain_and_zero_centered_gain_falls_back();
        fast_motor_feedback_at_fifty_millisecond_intervals_is_valid();
        big_following_respects_directional_small_yaw_counterrotation_capacity();
        for (const double direction : {-1.0, 1.0})
            for (const double lag_s : {0.0, .04, .08, .12})
                delayed_follow_capacity_reserve_retains_full_speed_release(direction, lag_s);
        std::cout << "coordinated yaw tests passed\n";
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << '\n';
        return 1;
    }
}
