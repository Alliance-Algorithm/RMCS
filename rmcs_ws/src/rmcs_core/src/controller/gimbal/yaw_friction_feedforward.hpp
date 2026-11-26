#pragma once

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace rmcs_core::controller::gimbal {

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
                    throw std::invalid_argument("Invalid yaw friction compensation coefficient");
            if ((static_friction > 0. && static_friction < coulomb) || scale > 1.2)
                throw std::invalid_argument("Invalid yaw friction compensation range");
            for (double value : {stribeck_velocity, smoothing_velocity, torque_limit, slew_rate})
                if (!std::isfinite(value) || value <= 0.)
                    throw std::invalid_argument("Invalid yaw friction compensation limit");
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
        last_torque_ = std::clamp(
            desired, last_torque_ - config.slew_rate * dt, last_torque_ + config.slew_rate * dt);
        return last_torque_;
    }

    void reset() { last_torque_ = 0.; }

private:
    double last_torque_ = 0.;
};

} // namespace rmcs_core::controller::gimbal
