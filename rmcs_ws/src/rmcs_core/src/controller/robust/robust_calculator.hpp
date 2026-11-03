#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string_view>

namespace rmcs_core::controller::robust {

enum class AuxiliaryLaw { Sliding, HighGain, Smooth };
enum class FrictionModel { Blend, Continuous };

inline AuxiliaryLaw parse_auxiliary_law(std::string_view name) {
    if (name == "sliding")
        return AuxiliaryLaw::Sliding;
    if (name == "high_gain")
        return AuxiliaryLaw::HighGain;
    if (name == "smooth")
        return AuxiliaryLaw::Smooth;
    throw std::invalid_argument("robust auxiliary_law must be sliding, high_gain, or smooth");
}

inline FrictionModel parse_friction_model(std::string_view name) {
    if (name == "blend")
        return FrictionModel::Blend;
    if (name == "continuous")
        return FrictionModel::Continuous;
    throw std::invalid_argument("robust friction_model must be blend or continuous");
}

struct RobustVelocityParameters {
    double inertia = 0.24;
    double viscous_friction = 0.0;
    double coulomb_friction = 0.0;
    double static_friction_compensation = 0.0;
    double torque_bias = 0.0;
    double tracking_gain = 1.0;
    double robust_gain = 1.0;
    double boundary_width = 0.1;
    // Optional velocity-error integral, stored directly as a torque correction.
    // Both values must be positive to enable load compensation.
    double integral_gain = 0.0; // Nm / rad
    double integral_torque_limit = 0.0; // Nm
    double friction_velocity_scale = 0.05;
    double standstill_velocity_threshold = 0.0;
    double output_min = -3.6;
    double output_max = 3.6;
    AuxiliaryLaw auxiliary_law = AuxiliaryLaw::Smooth;
    // Preserve the existing blend for callers that do not opt into the new model.
    FrictionModel friction_model = FrictionModel::Blend;
    // Continuous-model amplitude transition; independent of direction smoothing.
    double friction_transition_velocity_scale = 0.22;
    // Optional boundary for an exactly zero reference; zero follows boundary_width.
    double zero_reference_boundary_width = 0.0;
};

class RobustVelocityCalculator {
public:
    explicit RobustVelocityCalculator(RobustVelocityParameters parameters)
        : parameters_(parameters) {
        validate();
    }

    void reset() noexcept {
        integral_torque_ = 0.0;
        previous_desired_direction_ = 0;
    }

    double integral_torque() const noexcept { return integral_torque_; }

    // Tracks angular velocity and returns the inverse-model torque command:
    // tau = J * (d(omega_d)/dt + k*e + u_aux) + B*omega + friction + bias + tau_I.
    // Optional tau_I integrates velocity error in torque units, with its own
    // bound and output anti-windup. Centering and reversal clear its load state.
    // The optional continuous friction model uses raw speed for the model and error:
    // [Fc + (Fs-Fc)*exp(-(omega/ws)^2)] * tanh(omega_d/direction_scale).
    // The legacy blend retains its original standstill handling for other callers.
    // A zero reference requests dissipative braking without model compensation. A
    // reversing reference also brakes before compensation resumes in the new direction.
    // Here e = omega_d - omega. The smooth law is a bounded smooth approximation
    // to sign(e), with the same near-zero slope rho^2 / epsilon as the high-gain law.
    double update(
        double desired_velocity, double measured_velocity, double desired_acceleration,
        double dt = 0.0) {
        if (!std::isfinite(desired_velocity) || !std::isfinite(measured_velocity)
            || !std::isfinite(desired_acceleration)) {
            reset();
            return 0.0;
        }

        if (desired_velocity == 0.0) {
            reset();
            // Neither feedforward nor friction/bias may keep driving a stopped reference.
            // Retain the raw speed here so braking remains dissipative even near zero.
            const double error = -measured_velocity;
            const double boundary = parameters_.zero_reference_boundary_width > 0.0
                ? parameters_.zero_reference_boundary_width : parameters_.boundary_width;
            const double aux = auxiliary(error, boundary);
            const double braking = parameters_.inertia * (parameters_.tracking_gain * error + aux);
            return braking_torque(braking, measured_velocity);
        }
        const int desired_direction = desired_velocity > 0.0 ? 1 : -1;
        const bool reference_reversed = previous_desired_direction_ != 0
            && desired_direction != previous_desired_direction_;
        if (reference_reversed)
            integral_torque_ = 0.0;
        previous_desired_direction_ = desired_direction;
        // Quantized zero-speed feedback can have either sign. It must not select the
        // reversing branch and suppress breakaway torque in just one direction.
        const double direction_velocity =
            std::abs(measured_velocity) <= parameters_.standstill_velocity_threshold
                ? 0.0 : measured_velocity;
        const double model_velocity = parameters_.friction_model == FrictionModel::Continuous
            ? measured_velocity : direction_velocity;
        const double error = desired_velocity - model_velocity;
        const double aux = auxiliary(error, parameters_.boundary_width);
        const double acceleration = desired_acceleration + parameters_.tracking_gain * error + aux;
        if ((desired_velocity > 0.0 && direction_velocity < 0.0)
            || (desired_velocity < 0.0 && direction_velocity > 0.0)) {
            integral_torque_ = 0.0;
            return braking_torque(parameters_.inertia * acceleration, measured_velocity);
        }
        double friction;
        if (parameters_.friction_model == FrictionModel::Continuous) {
            const double ratio = model_velocity / parameters_.friction_transition_velocity_scale;
            const double magnitude = parameters_.coulomb_friction
                + (parameters_.static_friction_compensation - parameters_.coulomb_friction)
                    * std::exp(-ratio * ratio);
            friction = magnitude * std::tanh(desired_velocity / parameters_.friction_velocity_scale);
        } else {
            const double friction_blend = std::clamp(
                1.0 - std::abs(model_velocity) / parameters_.friction_velocity_scale, 0.0, 1.0);
            const double dynamic_friction = parameters_.coulomb_friction
                                          * std::tanh(model_velocity / parameters_.friction_velocity_scale);
            const double static_friction = parameters_.static_friction_compensation
                                         * std::tanh(desired_velocity / parameters_.friction_velocity_scale);
            friction = (1.0 - friction_blend) * dynamic_friction
                     + friction_blend * static_friction;
        }
        const double model_torque = parameters_.inertia * acceleration
                            + parameters_.viscous_friction * model_velocity + friction
                            + parameters_.torque_bias;

        if (!std::isfinite(model_torque)) {
            reset();
            return 0.0;
        }
        if (parameters_.integral_gain > 0.0 && parameters_.integral_torque_limit > 0.0) {
            // dt=0 inhibits accumulation while retaining the inverse-model
            // torque, allowing an owner to handle its own arming/reference ramp.
            if (!reference_reversed && std::isfinite(dt) && dt > 0.0) {
                const double candidate = std::clamp(
                    integral_torque_ + parameters_.integral_gain * error * dt,
                    -parameters_.integral_torque_limit, parameters_.integral_torque_limit);
                const double increment = candidate - integral_torque_;
                const double candidate_torque = model_torque + candidate;
                // Freeze only integration that drives farther into an output
                // limit; an opposite increment can still unwind saturation.
                if (!((candidate_torque > parameters_.output_max && increment > 0.0)
                      || (candidate_torque < parameters_.output_min && increment < 0.0)))
                    integral_torque_ = candidate;
            }
        } else {
            integral_torque_ = 0.0;
        }
        return std::clamp(
            model_torque + integral_torque_, parameters_.output_min, parameters_.output_max);
    }

private:
    double braking_torque(double torque, double measured_velocity) const {
        if (!std::isfinite(torque) || measured_velocity == 0.0)
            return 0.0;
        const double bounded = std::clamp(torque, parameters_.output_min, parameters_.output_max);
        return measured_velocity > 0.0 ? std::min(bounded, 0.0) : std::max(bounded, 0.0);
    }

    double auxiliary(double error, double epsilon) const {
        const double rho = parameters_.robust_gain;
        switch (parameters_.auxiliary_law) {
        case AuxiliaryLaw::Sliding:
            return rho * (error > 0.0 ? 1.0 : error < 0.0 ? -1.0 : 0.0);
        case AuxiliaryLaw::HighGain:
            return (rho * rho / epsilon) * error;
        case AuxiliaryLaw::Smooth:
            return (rho * rho * error) / std::hypot(rho * error, epsilon);
        }
        return 0.0;
    }

    void validate() const {
        const auto finite = [](double value) { return std::isfinite(value); };
        if (!finite(parameters_.inertia) || parameters_.inertia <= 0.0
            || !finite(parameters_.viscous_friction) || parameters_.viscous_friction < 0.0
            || !finite(parameters_.coulomb_friction) || parameters_.coulomb_friction < 0.0
            || !finite(parameters_.static_friction_compensation)
            || parameters_.static_friction_compensation < 0.0
            || !finite(parameters_.torque_bias) || !finite(parameters_.tracking_gain)
            || parameters_.tracking_gain <= 0.0 || !finite(parameters_.robust_gain)
            || parameters_.robust_gain <= 0.0 || !finite(parameters_.boundary_width)
            || parameters_.boundary_width <= 0.0
            || !finite(parameters_.zero_reference_boundary_width)
            || parameters_.zero_reference_boundary_width < 0.0
            || !finite(parameters_.integral_gain) || parameters_.integral_gain < 0.0
            || !finite(parameters_.integral_torque_limit) || parameters_.integral_torque_limit < 0.0
            || !finite(parameters_.friction_velocity_scale)
            || parameters_.friction_velocity_scale <= 0.0
            || !finite(parameters_.friction_transition_velocity_scale)
            || parameters_.friction_transition_velocity_scale <= 0.0
            || (parameters_.friction_model != FrictionModel::Blend
                && parameters_.friction_model != FrictionModel::Continuous)
            || !finite(parameters_.standstill_velocity_threshold)
            || parameters_.standstill_velocity_threshold < 0.0 || !finite(parameters_.output_min)
            || !finite(parameters_.output_max) || parameters_.output_min >= parameters_.output_max)
            throw std::invalid_argument("invalid robust velocity controller parameters");
    }

    RobustVelocityParameters parameters_;
    double integral_torque_{0.0};
    int previous_desired_direction_{0};
};

} // namespace rmcs_core::controller::robust
