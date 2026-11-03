#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "controller/gimbal/velocity_reference.hpp"
#include "controller/robust/robust_calculator.hpp"
#include "hardware/gimbal_torque_limiter.hpp"

namespace {

using rmcs_core::controller::gimbal::VelocityReference;
using rmcs_core::controller::robust::AuxiliaryLaw;
using rmcs_core::controller::robust::RobustVelocityCalculator;
using rmcs_core::controller::robust::RobustVelocityParameters;
using rmcs_core::controller::robust::parse_auxiliary_law;
using rmcs_core::hardware::limit_gimbal_torque;
using rmcs_core::hardware::gimbal_velocity_stopping;

constexpr std::array kLaws{AuxiliaryLaw::Sliding, AuxiliaryLaw::HighGain, AuxiliaryLaw::Smooth};
constexpr std::array kTorqueLimits{3.2, 10.0};
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr std::array kInvalid{kNan, kInf, -kInf};

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

void near(double actual, double expected, const char* message, double tolerance = 1e-10) {
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

RobustVelocityParameters deployed_parameters(double torque_limit = 3.2) {
    RobustVelocityParameters parameters;
    parameters.inertia = 0.24;
    parameters.coulomb_friction = 3.2;
    parameters.static_friction_compensation = 3.2;
    parameters.tracking_gain = 2.0;
    parameters.robust_gain = 1.0;
    parameters.boundary_width = 0.1;
    parameters.friction_velocity_scale = 0.05;
    parameters.standstill_velocity_threshold = 0.02;
    parameters.output_min = -torque_limit;
    parameters.output_max = torque_limit;
    return parameters;
}

void zero_target_brakes() {
    for (const auto law : kLaws) {
        for (const double torque_limit : kTorqueLimits) {
            for (const double bias : {-3.0, 0.0, 3.0}) {
                auto parameters = deployed_parameters(torque_limit);
                parameters.auxiliary_law = law;
                parameters.viscous_friction = 7.0;
                parameters.torque_bias = bias;
                RobustVelocityCalculator calculator{parameters};
                for (const double velocity : {-100.0, -1.0, -0.35, -0.02, -1e-6, 0.0,
                                              1e-6, 0.02, 0.35, 1.0, 100.0}) {
                    const double baseline = calculator.update(0.0, velocity, 0.0);
                    for (const double acceleration : {-100.0, 0.0, 100.0}) {
                        const double torque = calculator.update(0.0, velocity, acceleration);
                        require(std::isfinite(torque), "stopping torque must be finite");
                        require(torque * velocity <= 0.0, "zero target must never propel existing motion");
                        require(std::abs(torque) <= torque_limit, "stopping torque must respect limits");
                        near(torque, baseline, "zero target must ignore acceleration feedforward");
                        if (velocity == 0.0)
                            near(torque, 0.0, "zero target at rest must not start motion");
                        else
                            require(torque * velocity < 0.0, "nonzero speed must receive braking torque");
                    }
                }
            }
        }
    }
    RobustVelocityCalculator calculator{deployed_parameters()};
    near(calculator.update(0.0, 0.35, 0.0), -0.3987657474337975,
         "reported positive-speed runaway must become braking", 1e-8);
    near(calculator.update(0.0, -0.35, 0.0), 0.3987657474337975,
         "reported negative-speed runaway must become braking", 1e-8);
}

void same_direction_drive_is_preserved() {
    for (const auto law : kLaws) {
        auto parameters = deployed_parameters();
        parameters.auxiliary_law = law;
        RobustVelocityCalculator calculator{parameters};
        for (const double direction : {-1.0, 1.0}) {
            near(calculator.update(direction * 0.35, 0.0, 0.0), direction * 3.2,
                 "normal breakaway must retain the configured 3.2 Nm capability");
            near(calculator.update(direction * 0.35, direction * 0.35, 0.0), direction * 3.2,
                 "normal cruising compensation must be preserved", 1e-5);
        }
    }
}

void quantized_standstill_starts_both_directions() {
    // DM8009 velocity is 12-bit over [-25, 25] rad/s. Zero lies between bins;
    // the protected, stationary motor reported these negative and positive bins.
    constexpr double half_velocity_bin = 25.0 / 4095.0;
    for (const auto law : kLaws) {
        auto parameters = deployed_parameters();
        parameters.auxiliary_law = law;
        RobustVelocityCalculator calculator{parameters};
        for (const double measured : {-3.0 * half_velocity_bin, -half_velocity_bin,
                                     0.0, half_velocity_bin, 3.0 * half_velocity_bin}) {
            for (const double target : {0.005, 0.05, 0.35}) {
                const double positive = calculator.update(target, measured, 0.0);
                const double negative = calculator.update(-target, measured, 0.0);
                near(positive, calculator.update(target, 0.0, 0.0),
                     "standstill noise must not remove positive breakaway compensation");
                near(negative, calculator.update(-target, 0.0, 0.0),
                     "standstill noise must not remove negative breakaway compensation");
                near(positive, -negative,
                     "the same stationary velocity bin must permit symmetric starting torque");
            }
        }
    }
}

void reversal_finishes_at_quantized_standstill() {
    constexpr double half_velocity_bin = 25.0 / 4095.0;
    for (const auto law : kLaws) {
        auto parameters = deployed_parameters();
        parameters.auxiliary_law = law;
        RobustVelocityCalculator calculator{parameters};
        for (const double direction : {-1.0, 1.0}) {
            double applied = direction * 3.2;
            for (const double speed : {0.35, 0.1, 0.05, 0.03}) {
                const double measured = direction * speed;
                const double requested = calculator.update(-direction * 0.35, measured, 0.0);
                require(requested * measured < 0.0,
                        "a real reversal must brake before resuming friction compensation");
                applied = limit_gimbal_torque(requested, applied, 0.005, false, measured);
                require(applied * measured <= 0.0,
                        "reversal must immediately withdraw the old propulsive torque");
            }
            const double measured = direction * half_velocity_bin;
            const double requested = calculator.update(-direction * 0.35, measured, 0.0);
            near(requested, -direction * 3.2,
                 "reversal must regain full breakaway torque after reaching standstill");
            for (int cycle = 0; cycle < 700; ++cycle) {
                const double previous = applied;
                applied = limit_gimbal_torque(requested, applied, 0.005, false, measured);
                require(std::abs(applied - previous) <= 0.005 + 1e-10,
                        "restored reversal torque must still obey the hardware slew limit");
            }
            near(applied, -direction * 3.2,
                 "hardware must build the restored breakaway torque in either direction");
        }
    }
}

void standstill_threshold_is_configurable() {
    constexpr double half_velocity_bin = 25.0 / 4095.0;
    for (const auto law : kLaws) {
        auto parameters = deployed_parameters();
        parameters.auxiliary_law = law;
        parameters.standstill_velocity_threshold = 0.0;
        RobustVelocityCalculator unfiltered{parameters};
        require(unfiltered.update(0.35, -half_velocity_bin, 0.0)
                    < unfiltered.update(0.35, 0.0, 0.0),
                "zero threshold must retain the original reversal behavior");
        parameters.standstill_velocity_threshold = 0.02;
        RobustVelocityCalculator filtered{parameters};
        for (const double direction : {-1.0, 1.0}) {
            near(filtered.update(direction * 0.35, -direction * 0.02, 0.0), direction * 3.2,
                 "the configured standstill boundary must retain breakaway compensation");
            near(filtered.update(direction * 0.35, -direction * 0.020001, 0.0),
                 unfiltered.update(direction * 0.35, -direction * 0.020001, 0.0),
                 "measured reversal outside the threshold must still brake without friction");
        }
    }
}

void reversed_target_brakes() {
    for (const auto law : kLaws) {
        for (const double bias : {-3.0, 0.0, 3.0}) {
            auto parameters = deployed_parameters();
            parameters.auxiliary_law = law;
            parameters.viscous_friction = 7.0;
            parameters.torque_bias = bias;
            RobustVelocityCalculator calculator{parameters};
            for (const double velocity : {-0.35, -0.02, -1e-6, 1e-6, 0.02, 0.35}) {
                const double target = velocity > 0.0 ? -0.35 : 0.35;
                const double torque = calculator.update(target, velocity, 0.0);
                require(std::isfinite(torque) && torque * velocity < 0.0,
                        "reversing the target must brake the old direction");
            }
        }
    }
    RobustVelocityCalculator calculator{deployed_parameters()};
    near(calculator.update(-0.35, 0.35, 0.0), -0.57358787847868,
         "reverse command must no longer produce +2.626 Nm", 1e-8);
}

void calculator_limits_and_invalid_inputs() {
    for (const auto law : kLaws) {
        auto parameters = deployed_parameters();
        parameters.auxiliary_law = law;
        parameters.output_min = -1.1;
        parameters.output_max = 0.7;
        RobustVelocityCalculator calculator{parameters};
        for (const double target : {-100.0, -0.35, 0.0, 0.35, 100.0}) {
            for (const double velocity : {-100.0, -0.35, 0.0, 0.35, 100.0}) {
                for (const double acceleration : {-10000.0, 0.0, 10000.0}) {
                    const double torque = calculator.update(target, velocity, acceleration);
                    require(std::isfinite(torque) && torque >= -1.1 && torque <= 0.7,
                            "all laws must respect asymmetric output limits");
                }
            }
        }
        for (const double invalid : kInvalid) {
            near(calculator.update(invalid, 0.35, 0.0), 0.0, "invalid target must fail closed");
            near(calculator.update(0.35, invalid, 0.0), 0.0, "invalid speed must fail closed");
            near(calculator.update(0.35, 0.0, invalid), 0.0, "invalid feedforward must fail closed");
        }
    }
}

void parameter_validation() {
    using Parameters = RobustVelocityParameters;
    const std::array<double Parameters::*, 12> fields{
        &Parameters::inertia, &Parameters::viscous_friction, &Parameters::coulomb_friction,
        &Parameters::static_friction_compensation, &Parameters::torque_bias,
        &Parameters::tracking_gain, &Parameters::robust_gain, &Parameters::boundary_width,
        &Parameters::friction_velocity_scale, &Parameters::standstill_velocity_threshold,
        &Parameters::output_min, &Parameters::output_max};
    for (const auto field : fields) {
        for (const double invalid : kInvalid) {
            auto parameters = deployed_parameters();
            parameters.*field = invalid;
            rejects([&] { RobustVelocityCalculator calculator{parameters}; },
                    "nonfinite parameters must be rejected");
        }
    }
    for (const auto field : {&Parameters::inertia, &Parameters::tracking_gain,
                            &Parameters::robust_gain, &Parameters::boundary_width,
                            &Parameters::friction_velocity_scale}) {
        auto parameters = deployed_parameters();
        parameters.*field = 0.0;
        rejects([&] { RobustVelocityCalculator calculator{parameters}; },
                "strictly positive parameters must reject zero");
    }
    for (const auto field : {&Parameters::viscous_friction, &Parameters::coulomb_friction,
                            &Parameters::static_friction_compensation,
                            &Parameters::standstill_velocity_threshold}) {
        auto parameters = deployed_parameters();
        parameters.*field = -1.0;
        rejects([&] { RobustVelocityCalculator calculator{parameters}; },
                "friction coefficients and the standstill threshold must reject negative values");
    }
    auto parameters = deployed_parameters();
    parameters.output_min = parameters.output_max;
    rejects([&] { RobustVelocityCalculator calculator{parameters}; },
            "empty torque range must be rejected");
    require(parse_auxiliary_law("sliding") == AuxiliaryLaw::Sliding, "sliding law parsing");
    require(parse_auxiliary_law("high_gain") == AuxiliaryLaw::HighGain, "high-gain law parsing");
    require(parse_auxiliary_law("smooth") == AuxiliaryLaw::Smooth, "smooth law parsing");
    rejects([] { parse_auxiliary_law("unknown"); }, "unknown auxiliary law must be rejected");
}

void reference_release_and_reversal() {
    for (const double direction : {-1.0, 1.0}) {
        VelocityReference reference;
        for (int i = 0; i < 100; ++i)
            reference.update(direction * 0.35, 0.5, 0.01, true);
        near(reference.velocity(), direction * 0.35, "reference must reach the joystick target");
        reference.update(0.0, 0.5, 0.01, true);
        near(reference.velocity(), 0.0, "stick release must zero reference in the same update");
        near(reference.acceleration(), 0.0, "stick release must clear reference feedforward");
        reference.update(direction * 0.35, 0.5, 0.01, true);
        near(reference.velocity(), direction * 0.005, "restart must ramp from zero");
        near(reference.acceleration(), direction * 0.5, "restart must obey acceleration limit");
        reference.update(-direction * 0.35, 0.5, 0.01, true);
        near(reference.velocity(), -direction * 0.005, "reversal must discard old-sign reference");
        near(reference.acceleration(), -direction * 0.5, "reversal must ramp in the new direction");
        reference.update(direction * 0.35, 0.5, 0.01, false);
        near(reference.velocity(), 0.0, "inactive control must reset reference");
        near(reference.acceleration(), 0.0, "inactive control must reset acceleration");
    }
}

void reference_invalid_inputs() {
    for (const double invalid : kInvalid) {
        for (int field = 0; field < 3; ++field) {
            VelocityReference reference;
            reference.update(0.35, 0.5, 0.01, true);
            reference.update(field == 0 ? invalid : 0.35, field == 1 ? invalid : 0.5,
                             field == 2 ? invalid : 0.01, true);
            near(reference.velocity(), 0.0, "invalid reference input must clear velocity");
            near(reference.acceleration(), 0.0, "invalid reference input must clear acceleration");
        }
    }
    for (const double invalid : {-1.0, 0.0}) {
        for (int field = 0; field < 2; ++field) {
            VelocityReference reference;
            reference.update(0.35, 0.5, 0.01, true);
            reference.update(0.35, field == 0 ? invalid : 0.5, field == 1 ? invalid : 0.01, true);
            near(reference.velocity(), 0.0, "nonpositive reference limits must clear velocity");
            near(reference.acceleration(), 0.0, "nonpositive reference limits must clear acceleration");
        }
    }
}

void torque_limiter_removes_propulsion_tail() {
    for (const double torque_limit : kTorqueLimits) {
        for (const double direction : {-1.0, 1.0}) {
            const double velocity = direction * 0.35;
            const double previous = direction * torque_limit;
            near(limit_gimbal_torque(-direction * 0.4, previous, 0.005, true, velocity),
                 -direction * 0.005, "centering must remove the old propulsion tail immediately");
            near(limit_gimbal_torque(direction * torque_limit, previous, 0.005, true, velocity), 0.0,
                 "neutral hardware guard must reject a propulsive controller output");
            near(limit_gimbal_torque(-direction * 0.4, -direction * 0.1, 0.005, true, velocity),
                 -direction * 0.105, "established braking must retain its slew limit");
            near(limit_gimbal_torque(-direction * 0.4, previous, 0.005, false, velocity),
                 -direction * 0.005, "reverse braking must discard old propulsion before ramping");
            near(limit_gimbal_torque(direction * torque_limit, direction * 0.1, 0.005, false, velocity),
                 direction * 0.105, "normal propulsion must retain its slew limit");
            near(limit_gimbal_torque(direction * 0.4, -direction * 0.2, 0.005, true, -velocity),
                 direction * 0.005, "velocity crossing zero must not turn old braking into propulsion");
            near(limit_gimbal_torque(direction * 0.4, previous, 0.005, true, 0.0), 0.0,
                 "neutral at rest must clear both request and torque history");
        }
    }
}

void torque_limiter_invalid_inputs() {
    for (const double invalid : kInvalid) {
        for (const bool stopping : {false, true}) {
            near(limit_gimbal_torque(invalid, 3.2, 0.005, stopping, 0.35), 0.0,
                 "invalid torque request must fail closed");
            near(limit_gimbal_torque(-0.4, invalid, 0.005, stopping, 0.35), 0.0,
                 "invalid torque history must fail closed");
            near(limit_gimbal_torque(-0.4, 3.2, invalid, stopping, 0.35), 0.0,
                 "invalid torque slew step must fail closed");
            near(limit_gimbal_torque(-0.4, 3.2, 0.005, stopping, invalid), 0.0,
                 "invalid velocity must fail closed");
        }
    }
    near(limit_gimbal_torque(-0.4, 3.2, -0.005, true, 0.35), 0.0,
         "negative slew step must fail closed");
}

void complete_release_path() {
    // Check dissipative release both with the original slow ramp and the
    // responsive 2 rad/s configuration, where the limiter can change faster.
    for (const auto& [target_speed, acceleration, torque_step] :
         {std::array{0.35, 0.5, 0.005}, std::array{2.0, 40.0, 0.2}}) {
        for (const double torque_limit : kTorqueLimits) {
            RobustVelocityCalculator calculator{deployed_parameters(torque_limit)};
            for (const double direction : {-1.0, 1.0}) {
                VelocityReference reference;
                for (int i = 0; i < 100; ++i)
                    reference.update(direction * target_speed, acceleration, 0.01, true);
                double applied = direction * torque_limit;
                reference.update(0.0, acceleration, 0.001, true);
                // These speeds represent deceleration and a small zero crossing;
                // they do not claim a physical stop time.
                for (const double speed : {target_speed, 0.3, 0.1, 0.02, 0.001, 0.0, -0.001, 0.0}) {
                    const double velocity = direction * speed;
                    const double requested = calculator.update(reference.velocity(), velocity,
                                                               reference.acceleration());
                    applied = limit_gimbal_torque(requested, applied, torque_step, true, velocity);
                    require(std::isfinite(applied) && applied * velocity <= 0.0,
                            "the complete released-stick path must not add rotational energy");
                    require(std::abs(applied) <= torque_limit,
                            "the complete released-stick path must respect its configured torque limit");
                    if (velocity == 0.0)
                        near(applied, 0.0, "the complete path must not restart a stopped motor");
                }
            }
        }
    }
}

void coordinated_heading_hold_can_finish_following() {
    for (const double direction : {-1.0, 1.0}) {
        RobustVelocityCalculator calculator{deployed_parameters()};
        const double target = direction * 0.35;
        const double drive = calculator.update(target, 0.0, 0.0);
        const double following = limit_gimbal_torque(
            drive, 0.0, 0.2, gimbal_velocity_stopping(true, 0.0, target), 0.0);
        require(following * direction > 0.0,
                "centered stick must allow a coordinated nonzero follow target to start motion");
        near(limit_gimbal_torque(drive, 0.0, 0.2,
                                gimbal_velocity_stopping(false, 0.0, target), 0.0),
             0.0, "legacy centered-stick mode must preserve braking-only behavior");
        const double brake = calculator.update(0.0, direction * 0.1, 0.0);
        const double stopping = limit_gimbal_torque(
            brake, following, 0.2, gimbal_velocity_stopping(true, 0.5, 0.0), direction * 0.1);
        require(stopping * direction <= 0.0,
                "completed follow reference must brake even if physical stick is displaced");
    }
    require(gimbal_velocity_stopping(true, 0.0, kNan),
            "invalid coordinated velocity must select braking-only mode");
}

} // namespace

int main() {
    const std::array tests{
        std::pair{"zero target brakes for all auxiliary laws", &zero_target_brakes},
        std::pair{"normal 3.2 Nm drive remains available", &same_direction_drive_is_preserved},
        std::pair{"quantized standstill permits symmetric starting torque", &quantized_standstill_starts_both_directions},
        std::pair{"reversal regains breakaway torque at standstill", &reversal_finishes_at_quantized_standstill},
        std::pair{"standstill threshold is configurable and bounded", &standstill_threshold_is_configurable},
        std::pair{"opposite target brakes the old direction", &reversed_target_brakes},
        std::pair{"calculator limits and invalid inputs", &calculator_limits_and_invalid_inputs},
        std::pair{"parameter validation", &parameter_validation},
        std::pair{"reference release and reversal", &reference_release_and_reversal},
        std::pair{"reference rejects nonfinite inputs", &reference_invalid_inputs},
        std::pair{"torque limiter removes propulsion tails", &torque_limiter_removes_propulsion_tail},
        std::pair{"torque limiter rejects nonfinite inputs", &torque_limiter_invalid_inputs},
        std::pair{"complete released-stick control path", &complete_release_path},
        std::pair{"coordinated heading hold finishes following", &coordinated_heading_hold_can_finish_following}};
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try {
            test();
            std::cout << "PASS: " << name << '\n';
        } catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL: " << name << ": " << error.what() << '\n';
        }
    }
    std::cout << tests.size() - failures << '/' << tests.size() << " cases passed\n";
    return failures == 0 ? 0 : 1;
}
