#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "controller/robust/robust_calculator.hpp"

namespace {

using rmcs_core::controller::robust::AuxiliaryLaw;
using rmcs_core::controller::robust::FrictionModel;
using rmcs_core::controller::robust::RobustVelocityCalculator;
using rmcs_core::controller::robust::RobustVelocityParameters;

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

RobustVelocityParameters model_parameters() {
    RobustVelocityParameters parameters;
    parameters.inertia = 0.460762;
    parameters.viscous_friction = 0.299774;
    parameters.coulomb_friction = 3.15532;
    parameters.static_friction_compensation = 3.15532;
    parameters.torque_bias = -0.134329;
    parameters.tracking_gain = 2.0;
    parameters.robust_gain = 1.0;
    parameters.boundary_width = 0.1;
    parameters.friction_velocity_scale = 0.05;
    parameters.standstill_velocity_threshold = 0.02;
    parameters.output_min = -3.6;
    parameters.output_max = 3.6;
    parameters.auxiliary_law = AuxiliaryLaw::Smooth;
    return parameters;
}

void invalid_inputs_fail_closed() {
    RobustVelocityCalculator calculator{model_parameters()};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    require(calculator.update(nan, 0.0, 0.0) == 0.0, "invalid setpoint must produce zero");
    require(calculator.update(0.0, nan, 0.0) == 0.0, "invalid measurement must produce zero");
    require(calculator.update(0.0, 0.0, nan) == 0.0, "invalid acceleration must produce zero");
}

void model_output_is_bounded_in_both_directions() {
    RobustVelocityCalculator calculator{model_parameters()};
    const double positive = calculator.update(0.4, 0.0, 0.0);
    const double negative = calculator.update(-0.4, 0.0, 0.0);
    require(std::abs(positive - 3.6) < 1e-12, "positive output must respect the configured limit");
    require(std::abs(negative + 3.6) < 1e-12, "negative output must respect the configured limit");

    for (const double limit : {7.0, 10.0}) {
        auto parameters = model_parameters();
        parameters.output_min = -limit;
        parameters.output_max = limit;
        RobustVelocityCalculator calculator{parameters};
        require(calculator.update(100.0, 0.0, 0.0) == limit,
                "an explicit higher controller limit must permit positive output above 6 Nm");
        require(calculator.update(-100.0, 0.0, 0.0) == -limit,
                "an explicit higher controller limit must permit negative output above 6 Nm");
    }
}

void zero_reference_only_brakes_and_reversal_opposes_motion() {
    RobustVelocityCalculator calculator{model_parameters()};
    const double positive_braking = calculator.update(0.0, 0.5, 0.0);
    const double negative_braking = calculator.update(0.0, -0.5, 0.0);
    require(positive_braking < 0.0, "zero reference must brake positive motion");
    require(negative_braking > 0.0, "zero reference must brake negative motion");
    require(calculator.update(0.4, -0.1, 0.0) >= 0.0,
            "a positive reference must not drive farther into negative motion");
    require(calculator.update(-0.4, 0.1, 0.0) <= 0.0,
            "a negative reference must not drive farther into positive motion");
}

void invalid_configuration_is_rejected() {
    auto parameters = model_parameters();
    parameters.inertia = 0.0;
    try {
        const RobustVelocityCalculator calculator{parameters};
        (void)calculator;
    } catch (const std::invalid_argument&) {
        return;
    }
    throw std::runtime_error("nonpositive inertia must be rejected");
}

RobustVelocityParameters integral_parameters() {
    RobustVelocityParameters parameters;
    parameters.inertia = 1.0;
    parameters.tracking_gain = 1.0;
    parameters.robust_gain = 0.01;
    parameters.boundary_width = 1.0;
    parameters.integral_gain = 1.0;
    parameters.integral_torque_limit = 5.0;
    parameters.output_min = -10.0;
    parameters.output_max = 10.0;
    return parameters;
}

void integral_rejects_constant_opposing_load_in_both_directions() {
    auto parameters = integral_parameters();
    parameters.inertia = 0.3;
    parameters.viscous_friction = 0.2;
    parameters.tracking_gain = 2.0;
    parameters.robust_gain = 0.5;
    parameters.boundary_width = 0.2;
    parameters.integral_gain = 10.0;
    parameters.integral_torque_limit = 2.0;
    constexpr double dt = 0.001;
    for (double direction : {-1.0, 1.0}) {
        RobustVelocityCalculator calculator{parameters};
        double velocity = 0.0;
        for (int iteration = 0; iteration < 15000; ++iteration) {
            const double torque = calculator.update(direction * 0.5, velocity, 0.0, dt);
            // A constant unmodelled 1 Nm opposing load, including static
            // friction at rest. Viscous drag and inertia match the nominal model.
            if (velocity == 0.0 && std::abs(torque) <= 1.0)
                continue;
            const double opposing_load = std::copysign(1.0, velocity != 0.0 ? velocity : torque);
            const double next = velocity + dt * (torque - 0.2 * velocity - opposing_load) / 0.3;
            velocity = velocity * next < 0.0 && std::abs(torque) <= 1.0 ? 0.0 : next;
        }
        require(std::abs(velocity - direction * 0.5) < 0.005,
                "bounded integral must remove the constant-load tracking error");
        require(std::abs(calculator.integral_torque() - direction) < 0.02,
                "the persistent torque correction must converge to the omitted load");
        calculator.update(direction * 0.5, direction * 0.5, 0.0, dt);
        require(std::abs(calculator.integral_torque() - direction) < 0.02,
                "reaching target speed must retain the load correction");
    }
}

void integral_saturation_freezes_and_can_unwind() {
    auto parameters = integral_parameters();
    parameters.output_min = -1.0;
    parameters.output_max = 1.0;
    RobustVelocityCalculator calculator{parameters};
    for (double direction : {-1.0, 1.0}) {
        calculator.reset();
        calculator.update(direction * 0.5, 0.0, 0.0, 0.4);
        const double accumulated = calculator.integral_torque();
        require(std::abs(accumulated - direction * 0.2) < 1e-12,
                "unsaturated load error must accumulate");
        for (int iteration = 0; iteration < 100; ++iteration) {
            const double torque = calculator.update(direction, 0.0, 0.0, 0.1);
            require(std::abs(torque - direction) < 1e-12,
                    "integral saturation must respect both output limits");
        }
        require(calculator.integral_torque() == accumulated,
                "outward integration must freeze at an output limit");
        calculator.update(direction, direction * 2.0, direction * 3.0, 0.1);
        require(std::abs(calculator.integral_torque() - direction * 0.1) < 1e-12,
                "opposite error must unwind even while feedforward keeps output saturated");
    }
    parameters.output_min = -10.0;
    parameters.output_max = 10.0;
    parameters.integral_torque_limit = 0.3;
    RobustVelocityCalculator limited{parameters};
    for (int iteration = 0; iteration < 100; ++iteration)
        limited.update(0.4, 0.0, 0.0, 1.0);
    require(std::abs(limited.integral_torque() - 0.3) < 1e-12,
            "the separate load-integral torque cap must hold below the motor output limit");
}

void integral_clears_on_center_reversal_invalid_input_and_reset() {
    RobustVelocityCalculator calculator{integral_parameters()};
    calculator.update(0.5, 0.0, 0.0, 0.2);
    require(calculator.integral_torque() > 0.0, "test must establish a prior load correction");
    const double braking = calculator.update(0.0, 0.4, 0.0, 0.1);
    require(calculator.integral_torque() == 0.0 && braking <= 0.0,
            "centering must remove integral and retain dissipative braking");
    calculator.update(0.5, 0.0, 0.0, 0.2);
    calculator.update(-0.5, 0.0, 0.0, 0.2);
    require(calculator.integral_torque() == 0.0,
            "a reversing reference must not inherit the prior directional load");
    calculator.update(-0.5, 0.0, 0.0, 0.2);
    require(calculator.integral_torque() < 0.0, "the new direction must accumulate independently");
    calculator.update(-0.5, 0.2, 0.0, 0.2);
    require(calculator.integral_torque() == 0.0,
            "measured opposite motion must clear the load integral before reversal braking");
    calculator.update(0.5, 0.0, 0.0, 0.2);
    calculator.reset();
    require(calculator.integral_torque() == 0.0,
            "an owner's disable or permission loss must be able to clear the integral");
    RobustVelocityCalculator fresh{integral_parameters()};
    require(calculator.update(0.5, 0.0, 0.0, 0.0) == fresh.update(0.5, 0.0, 0.0, 0.0),
            "re-enabling must behave like a fresh calculator without a retained load");
    calculator.update(0.5, 0.0, 0.0, 0.2);
    calculator.update(0.5, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.1);
    require(calculator.integral_torque() == 0.0,
            "invalid measurement must not retain a load correction");
}

void invalid_dt_skips_integration_and_default_mode_stays_stateless() {
    RobustVelocityCalculator calculator{integral_parameters()};
    calculator.update(0.5, 0.0, 0.0, 0.2);
    const double stored = calculator.integral_torque();
    for (double dt : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::infinity()}) {
        require(std::isfinite(calculator.update(0.5, 0.1, 0.0, dt)),
                "invalid/nonpositive dt must leave the inverse-model torque finite");
        require(calculator.integral_torque() == stored,
                "invalid/nonpositive dt must skip accumulation without poisoning the stored load");
    }
    auto parameters = model_parameters();
    RobustVelocityCalculator without_dt{parameters}, with_dt{parameters};
    for (double target : {0.4, -0.4, 0.0, 0.2}) {
        for (double measured : {-0.2, 0.0, 0.02, 0.2}) {
            require(without_dt.update(target, measured, 0.1)
                        == with_dt.update(target, measured, 0.1, 0.01),
                    "zero integral defaults must preserve the previous stateless output");
            require(with_dt.integral_torque() == 0.0,
                    "disabled integral must not accumulate state");
        }
    }
}

void invalid_integral_configuration_is_rejected() {
    for (bool invalid_gain : {false, true}) {
        auto parameters = integral_parameters();
        if (invalid_gain)
            parameters.integral_gain = -1.0;
        else
            parameters.integral_torque_limit = std::numeric_limits<double>::infinity();
        bool rejected = false;
        try {
            RobustVelocityCalculator calculator{parameters};
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        require(rejected, "invalid integral gain/limit must be rejected");
    }
}

RobustVelocityParameters continuous_parameters() {
    RobustVelocityParameters parameters;
    parameters.inertia = 0.317375134;
    parameters.viscous_friction = 0.080399177;
    parameters.coulomb_friction = 1.912103711;
    parameters.static_friction_compensation = 3.6;
    parameters.torque_bias = -0.007155586;
    parameters.tracking_gain = 12.0;
    parameters.robust_gain = 4.2;
    parameters.boundary_width = 0.5;
    parameters.friction_velocity_scale = 0.01;
    parameters.standstill_velocity_threshold = 0.02;
    parameters.friction_model = FrictionModel::Continuous;
    parameters.friction_transition_velocity_scale = 0.22;
    parameters.output_min = -10.0;
    parameters.output_max = 10.0;
    return parameters;
}

void zero_reference_boundary_selects_only_the_centered_reference() {
    for (const auto law : {AuxiliaryLaw::Smooth, AuxiliaryLaw::HighGain, AuxiliaryLaw::Sliding}) {
        auto motion_parameters = continuous_parameters();
        motion_parameters.static_friction_compensation = 3.2;
        motion_parameters.boundary_width = 0.4;
        motion_parameters.auxiliary_law = law;
        auto centered_parameters = motion_parameters;
        centered_parameters.boundary_width = 0.6;
        auto split_parameters = motion_parameters;
        split_parameters.zero_reference_boundary_width = 0.6;
        RobustVelocityCalculator motion{motion_parameters};
        RobustVelocityCalculator centered{centered_parameters};
        RobustVelocityCalculator split{split_parameters};

        for (double target : {-0.0, 0.0}) {
            for (double velocity : {-100.0, -0.2, -0.006, 0.0, 0.006, 0.2, 100.0}) {
                for (double acceleration : {-100.0, 0.0, 100.0}) {
                    require(split.update(target, velocity, acceleration)
                                == centered.update(target, velocity, acceleration),
                            "both signed zeros must use the separate centered-reference boundary");
                }
            }
        }
        for (double target : {-0.1, -0.001, -1e-12, 1e-12, 0.001, 0.1}) {
            for (double velocity : {-0.2, -0.018315, 0.0, 0.018315, 0.2}) {
                for (double acceleration : {-100.0, 0.0, 0.3, 100.0}) {
                    require(split.update(target, velocity, acceleration)
                                == motion.update(target, velocity, acceleration),
                            "nonzero targets, including breakaway and reversal, must retain the motion boundary");
                }
            }
        }
    }
}

void zero_reference_boundary_default_and_zero_retain_the_shared_boundary() {
    require(RobustVelocityParameters{}.zero_reference_boundary_width == 0.0,
            "the separate centered-reference boundary must be opt-in");
    for (const auto law : {AuxiliaryLaw::Smooth, AuxiliaryLaw::HighGain, AuxiliaryLaw::Sliding}) {
        auto parameters = continuous_parameters();
        parameters.boundary_width = 0.4;
        parameters.auxiliary_law = law;
        RobustVelocityCalculator unset{parameters};
        parameters.zero_reference_boundary_width = parameters.boundary_width;
        RobustVelocityCalculator shared_boundary{parameters};
        for (double disabled : {-0.0, 0.0}) {
            parameters.zero_reference_boundary_width = disabled;
            RobustVelocityCalculator explicit_zero{parameters};
            for (double target : {-0.1, -1e-12, -0.0, 0.0, 1e-12, 0.1}) {
                for (double velocity : {-1.0, -0.006, 0.0, 0.006, 1.0}) {
                    const double expected = shared_boundary.update(target, velocity, 0.3);
                    require(unset.update(target, velocity, 0.3) == expected
                                && explicit_zero.update(target, velocity, 0.3) == expected,
                            "unset or zero centered-reference boundary must retain the shared-boundary output");
                }
            }
        }
    }
}

void zero_reference_boundary_keeps_center_reset_passivity_and_limits() {
    auto parameters = integral_parameters();
    parameters.boundary_width = 0.4;
    parameters.zero_reference_boundary_width = 0.6;
    parameters.output_min = -0.5;
    parameters.output_max = 0.5;
    RobustVelocityCalculator calculator{parameters};
    for (double direction : {-1.0, 1.0}) {
        for (double speed : {0.0, 0.006, 0.2, 100.0}) {
            calculator.reset();
            calculator.update(direction * 0.25, 0.0, 0.0, 0.1);
            require(std::abs(calculator.integral_torque()) > 0.0,
                    "the reset check must first establish a retained load correction");
            const double velocity = direction * speed;
            const double braking = calculator.update(0.0, velocity, direction * 100.0, 0.1);
            require(calculator.integral_torque() == 0.0 && braking * velocity <= 0.0,
                    "the separate boundary must retain integral reset and dissipative zero-reference braking");
            require(std::abs(braking) <= 0.5 && (speed != 0.0 || braking == 0.0),
                    "centered braking must retain its output limits and zero output at rest");
            if (speed == 100.0)
                require(braking == -direction * 0.5,
                        "centered braking must clamp in both directions with the separate boundary");
            RobustVelocityCalculator fresh{parameters};
            require(calculator.update(direction * 0.25, 0.0, 0.0)
                        == fresh.update(direction * 0.25, 0.0, 0.0),
                    "motion after centering must not inherit a prior load correction");
        }
    }
}

void invalid_zero_reference_boundary_is_rejected() {
    for (double boundary : {-0.1, std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity(),
                            -std::numeric_limits<double>::infinity()}) {
        auto parameters = continuous_parameters();
        parameters.zero_reference_boundary_width = boundary;
        bool rejected = false;
        try { RobustVelocityCalculator calculator{parameters}; }
        catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "the centered-reference boundary must be finite and nonnegative");
    }
}

void continuous_model_has_no_same_direction_standstill_step() {
    RobustVelocityCalculator calculator{continuous_parameters()};
    for (double direction : {-1.0, 1.0}) {
        for (double point : {0.0, 0.01, 0.02, 0.22}) {
            const double before = calculator.update(direction * 0.1, direction * (point - 1e-9), 0.0);
            const double after = calculator.update(direction * 0.1, direction * (point + 1e-9), 0.0);
            require(std::abs(after - before) < 1e-6,
                    "same-direction output must remain continuous across the standstill threshold");
        }
        const double lower_bin = calculator.update(direction * 0.1, direction * 0.018315018315, 0.0);
        const double higher_bin = calculator.update(direction * 0.1, direction * 0.030525030525, 0.0);
        require(std::abs(higher_bin - lower_bin) < 0.25,
                "the first moving DM speed bin must not discard the low-speed compensation");
    }
}

void continuous_friction_preserves_endpoints_and_direction_symmetry() {
    auto parameters = continuous_parameters();
    parameters.inertia = 1e-9; // Make feedback negligible to observe the friction endpoints.
    parameters.viscous_friction = 0.0;
    parameters.torque_bias = 0.0;
    RobustVelocityCalculator calculator{parameters};
    for (double direction : {-1.0, 1.0}) {
        const double starting = direction * calculator.update(direction, 0.0, 0.0);
        const double moving = direction * calculator.update(direction * 2.0, direction * 2.0, 0.0);
        require(std::abs(starting - 3.6) < 1e-6,
                "rest compensation must retain the existing 3.6 Nm magnitude");
        require(std::abs(moving - 1.912103711) < 1e-9,
                "large-speed compensation must retain the identified moving friction");
        double previous = starting;
        for (double speed : {0.01, 0.02, 0.05, 0.1, 0.22, 0.5, 1.0}) {
            const double magnitude = direction * calculator.update(direction, direction * speed, 0.0);
            require(magnitude <= previous && magnitude >= 1.912103711 - 1e-6,
                    "continuous friction must decay monotonically between its two endpoints");
            previous = magnitude;
        }
    }
    parameters.inertia = 0.317375134;
    parameters.viscous_friction = 0.080399177;
    RobustVelocityCalculator symmetric{parameters};
    for (double target : {0.001, 0.1, 0.3, 2.0}) {
        for (double velocity : {0.0, 0.018315, 0.030525, 0.22, 1.0}) {
            require(std::abs(symmetric.update(target, velocity, 0.2)
                           + symmetric.update(-target, -velocity, -0.2)) < 1e-12,
                    "the unbiased continuous model must be symmetric in the two directions");
        }
    }
}

void continuous_model_retains_braking_and_ignores_standstill_sign_noise() {
    auto parameters = continuous_parameters();
    RobustVelocityCalculator calculator{parameters};
    for (double direction : {-1.0, 1.0}) {
        for (double speed : {0.006105, 0.018315, 0.030525, 0.5}) {
            require(calculator.update(0.0, direction * speed, direction * 100.0) * direction <= 0.0,
                    "a centered reference must brake even with a nonzero feedforward");
        }
        require(calculator.update(direction * 0.1, -direction * 0.018315, 0.0) * direction > 0.0,
                "opposite-sign standstill quantization must not suppress breakaway");
        // A large static compensation/bias must not leak into the reversal branch.
        require(std::abs(calculator.update(direction * 0.1, -direction * 0.03, 0.0)) < 2.0,
                "opposite motion beyond the standstill range must use braking alone");
        require(calculator.update(-direction * 0.1, direction * 0.2, direction * 100.0)
                    * direction <= 0.0,
                "reversal braking must oppose motion even with conflicting acceleration");
        require(calculator.update(direction * 100.0, 0.0, 0.0) == direction * 10.0,
                "continuous friction must retain the configured output bound");
    }
    require(calculator.update(0.0, 0.0, 100.0) == 0.0,
            "centered and stopped must produce zero despite feedforward and bias");
}

void continuous_model_without_integral_has_no_retained_load() {
    const auto parameters = continuous_parameters();
    RobustVelocityCalculator calculator{parameters};
    for (int i = 0; i < 1000; ++i) {
        const double target = i % 3 == 0 ? -0.2 : i % 3 == 1 ? 0.0 : 0.1;
        const double velocity = i % 2 == 0 ? 0.018315 : -0.1;
        RobustVelocityCalculator fresh{parameters};
        require(calculator.update(target, velocity, 0.3, 1.0)
                    == fresh.update(target, velocity, 0.3, 0.0),
                "Ki=0 must give the same output after any prior target/direction history");
        require(calculator.integral_torque() == 0.0,
                "the native continuous-model trial must not accumulate an error integral");
    }
}

void default_blend_retains_recorded_operating_points() {
    auto parameters = continuous_parameters();
    parameters.friction_model = RobustVelocityParameters{}.friction_model;
    require(parameters.friction_model == FrictionModel::Blend, "the legacy model must remain the default");
    RobustVelocityCalculator calculator{parameters};
    require(std::abs(calculator.update(0.1, 0.018315018315, 0.0) - 4.831054) < 1e-6,
            "legacy standstill behavior must retain the previously audited operating point");
    require(std::abs(calculator.update(0.1, 0.030525030525, 0.0) - 2.835350) < 1e-6,
            "legacy moving blend must retain the previously audited operating point");
}

void continuous_friction_configuration_is_validated() {
    using rmcs_core::controller::robust::parse_friction_model;
    require(parse_friction_model("blend") == FrictionModel::Blend, "legacy model parsing");
    require(parse_friction_model("continuous") == FrictionModel::Continuous, "continuous model parsing");
    bool rejected = false;
    try { parse_friction_model("unknown"); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "unknown friction model must be rejected");
    for (double scale : {0.0, -0.1, std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity()}) {
        auto parameters = continuous_parameters();
        parameters.friction_transition_velocity_scale = scale;
        rejected = false;
        try { RobustVelocityCalculator calculator{parameters}; }
        catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "invalid continuous-friction transition scale must be rejected");
    }
}

} // namespace

int main() {
    invalid_inputs_fail_closed();
    model_output_is_bounded_in_both_directions();
    zero_reference_only_brakes_and_reversal_opposes_motion();
    invalid_configuration_is_rejected();
    integral_rejects_constant_opposing_load_in_both_directions();
    integral_saturation_freezes_and_can_unwind();
    integral_clears_on_center_reversal_invalid_input_and_reset();
    invalid_dt_skips_integration_and_default_mode_stays_stateless();
    invalid_integral_configuration_is_rejected();
    zero_reference_boundary_selects_only_the_centered_reference();
    zero_reference_boundary_default_and_zero_retain_the_shared_boundary();
    zero_reference_boundary_keeps_center_reset_passivity_and_limits();
    invalid_zero_reference_boundary_is_rejected();
    continuous_model_has_no_same_direction_standstill_step();
    continuous_friction_preserves_endpoints_and_direction_symmetry();
    continuous_model_retains_braking_and_ignores_standstill_sign_noise();
    continuous_model_without_integral_has_no_retained_load();
    default_blend_retains_recorded_operating_points();
    continuous_friction_configuration_is_validated();
    return 0;
}
