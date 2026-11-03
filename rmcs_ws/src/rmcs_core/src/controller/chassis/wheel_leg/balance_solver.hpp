#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <optional>
namespace rmcs_core::controller::chassis {
class BalanceSolver {
public:
    struct Settings {
        double dt = .001, pitch_cutoff_hz = 30.;
        double pitch_gain = -17.6, pitch_rate_gain = -3.6, velocity_gain = -3., distance_gain = 0.;
        double balance_limit = 3.69, yaw_gain = -10.8, yaw_rate_gain = -4.3, yaw_limit = 8.,
               yaw_sign = -1., wheel_limit = 4.88;
        double roll_kp = -1., roll_kd = -50., roll_limit = .5;
        bool yaw_enabled = false, trim_enabled = false;
        double trim_sign = 1., trim_ki = .02, trim_tau = 2., trim_deadband = .01, trim_limit = .4,
               trim_pitch_limit = std::numbers::pi / 3., trim_velocity_limit = .5;
    } config;
    struct Output {
        std::array<double, 2> wheel;
        double roll_bias, trim;
    };
    void reset() {
        pitch_.reset();
        roll_error_.reset();
        trim_torque_ = trim_ = 0.;
    }
    Output step(
        double pitch, double pitch_rate, double velocity, double distance, double yaw,
        double yaw_rate, double roll, double yaw_reference, double bank, double roll_ff,
        bool balance_enabled, double feedforward = 0.) {
        const double alpha =
            config.pitch_cutoff_hz > 0.
                ? 1. - std::exp(-2. * std::numbers::pi * config.pitch_cutoff_hz * config.dt)
                : 1.;
        pitch_ = pitch_ ? alpha * pitch + (1. - alpha) * *pitch_ : pitch;
        const double common =
            balance_enabled
                ? std::clamp(
                      config.pitch_gain * *pitch_ + config.pitch_rate_gain * pitch_rate
                          + config.velocity_gain * velocity + config.distance_gain * distance,
                      -config.balance_limit, config.balance_limit)
                : 0.;
        double diff =
            balance_enabled && config.yaw_enabled
                ? std::clamp(
                      config.yaw_gain * (yaw - yaw_reference) + config.yaw_rate_gain * yaw_rate,
                      -config.yaw_limit, config.yaw_limit)
                : 0.;
        const double limited = std::clamp(
                         common + (balance_enabled ? feedforward : 0.), -config.wheel_limit,
                         config.wheel_limit),
                     headroom = config.wheel_limit - std::abs(limited);
        diff = std::clamp(config.yaw_sign * diff, -headroom, headroom);
        const double error = roll - bank;
        const double bias = roll_ff
                          + std::clamp(
                                config.roll_kp * error
                                    + (roll_error_ ? config.roll_kd * (error - *roll_error_) : 0.),
                                -config.roll_limit, config.roll_limit);
        roll_error_ = error;
        if (!config.trim_enabled)
            trim_torque_ = trim_ = 0.;
        else {
            trim_torque_ += std::min(1., config.dt / config.trim_tau) * (common - trim_torque_);
            if (std::abs(pitch) <= config.trim_pitch_limit
                && std::abs(velocity) <= config.trim_velocity_limit) {
                const double e = std::abs(trim_torque_) < config.trim_deadband ? 0. : trim_torque_;
                trim_ = std::clamp(
                    trim_ - config.trim_sign * config.trim_ki * e * config.dt, -config.trim_limit,
                    config.trim_limit);
            }
        }
        return {{limited + diff, limited - diff}, bias, trim_};
    }

private:
    double trim_torque_ = 0., trim_ = 0.;
    std::optional<double> pitch_;
    std::optional<double> roll_error_;
};
} // namespace rmcs_core::controller::chassis
