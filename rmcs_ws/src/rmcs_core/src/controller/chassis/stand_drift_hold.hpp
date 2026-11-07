#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rmcs_core::controller::chassis {

class StandDriftHold {
public:
    struct Config {
        bool enabled = false;
        double position_kp = 0.;
        double velocity_kp = 0.;
        double command_limit = 0.;
        double command_rate = 0.;
        double velocity_filter_hz = 0.;
    };

    void configure(Config config) {
        if (config.position_kp < 0. || !std::isfinite(config.position_kp) || config.velocity_kp < 0.
            || !std::isfinite(config.velocity_kp) || config.command_limit <= 0.
            || !std::isfinite(config.command_limit) || config.command_rate <= 0.
            || !std::isfinite(config.command_rate) || config.velocity_filter_hz <= 0.
            || !std::isfinite(config.velocity_filter_hz))
            throw std::invalid_argument("Invalid stand drift hold configuration");
        config_ = config;
        reset();
    }

    void reset() {
        active_ = false;
        hold_distance_ = 0.;
        distance_error_ = 0.;
        filtered_velocity_ = 0.;
        command_ = 0.;
    }

    double update(bool requested, double distance, double velocity, double dt) {
        if (!config_.enabled || !requested || !std::isfinite(distance) || !std::isfinite(velocity)
            || !std::isfinite(dt) || dt <= 0. || dt > .25) {
            reset();
            return 0.;
        }
        if (!active_) {
            active_ = true;
            hold_distance_ = distance;
            filtered_velocity_ = velocity;
        }
        const double filter_dt = std::min(dt, .1);
        const double alpha =
            1. - std::exp(-6.2831853071795864769 * config_.velocity_filter_hz * filter_dt);
        filtered_velocity_ += alpha * (velocity - filtered_velocity_);
        distance_error_ = hold_distance_ - distance;
        const double desired = std::clamp(
            config_.position_kp * distance_error_ - config_.velocity_kp * filtered_velocity_,
            -config_.command_limit, config_.command_limit);
        const double maximum_delta = config_.command_rate * dt;
        command_ += std::clamp(desired - command_, -maximum_delta, maximum_delta);
        command_ = std::clamp(command_, -config_.command_limit, config_.command_limit);
        return command_;
    }

    bool active() const { return active_; }
    double distance_error() const { return distance_error_; }
    double filtered_velocity() const { return filtered_velocity_; }
    double command() const { return command_; }

private:
    Config config_;
    bool active_ = false;
    double hold_distance_ = 0.;
    double distance_error_ = 0.;
    double filtered_velocity_ = 0.;
    double command_ = 0.;
};

} // namespace rmcs_core::controller::chassis
