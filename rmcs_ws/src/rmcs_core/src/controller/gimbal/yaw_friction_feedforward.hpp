#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rmcs_core::controller::gimbal {

// Shared by the bench command guard and the production torque combiner.
class YawFrictionCompensation {
public:
    struct Config {
        double coulomb = 0.;
        double static_friction = 0.;
        double viscous = 0.;
        double stribeck_velocity = .003;
        double smoothing_velocity = .05;
        double scale = 0.;
        double torque_limit = .8;
        double slew_rate = 100.;

        void validate() const {
            for (double value : {coulomb, static_friction, viscous, scale})
                if (!std::isfinite(value) || value < 0.)
                    throw std::invalid_argument("Invalid Yaw friction compensation coefficient");
            // The bench identification chain allowed up to 1.2; keep the same
            // bound here. The output is still clamped by torque_limit (.8 Nm).
            if ((static_friction > 0. && static_friction < coulomb) || scale > 1.2)
                throw std::invalid_argument("Invalid Yaw friction compensation range");
            for (double value : {stribeck_velocity, smoothing_velocity, torque_limit, slew_rate})
                if (!std::isfinite(value) || value <= 0.)
                    throw std::invalid_argument("Invalid Yaw friction compensation limit");
        }
    };

    double update(double velocity_reference, double dt, const Config& config) {
        if (!std::isfinite(velocity_reference) || !std::isfinite(dt) || dt <= 0. || dt > .02
            || config.scale == 0.) {
            reset();
            return 0.;
        }
        const double magnitude =
            config.static_friction > 0.
                ? config.coulomb
                      + (config.static_friction - config.coulomb)
                            * std::exp(-std::pow(velocity_reference / config.stribeck_velocity, 2))
                : config.coulomb;
        const double desired = std::clamp(
            config.scale
                * (config.viscous * velocity_reference
                   + magnitude * std::tanh(velocity_reference / config.smoothing_velocity)),
            -config.torque_limit, config.torque_limit);
        last_ = std::clamp(desired, last_ - config.slew_rate * dt, last_ + config.slew_rate * dt);
        return last_;
    }

    void reset() { last_ = 0.; }

private:
    double last_ = 0.;
};

} // namespace rmcs_core::controller::gimbal
