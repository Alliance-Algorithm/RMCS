#pragma once
#include "model/mechanics.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
namespace rmcs_core::controller::chassis {
class MotionPlanner {
public:
    struct Settings {
        double dt = .001, max_velocity = 5., max_yaw_rate = 12., channel_kp = 10.;
        double velocity_accel = 1.5, velocity_jerk = 4., yaw_accel = 6.28, yaw_jerk = 40.;
        double velocity_sign = -1., gravity = 9.81, bank_to_roll_bias = 1.06;
        double friction_mu = 2., track_width = .3966, com_height = .26, lat_accel_safety = .7;
    } config;
    struct Reference {
        double velocity, yaw_rate, yaw, bank, roll_feedforward;
    };
    void reset() { velocity_ = accel_ = yaw_rate_ = yaw_accel_ = yaw_ = 0.; }
    Reference step(double vx, double wz) {
        step_channel(
            velocity_, accel_, std::clamp(vx, -config.max_velocity, config.max_velocity),
            config.velocity_accel, config.velocity_jerk);
        step_channel(
            yaw_rate_, yaw_accel_, std::clamp(wz, -config.max_yaw_rate, config.max_yaw_rate),
            config.yaw_accel, config.yaw_jerk);
        yaw_ += yaw_rate_ * config.dt;
        const double limit = config.lat_accel_safety
                           * std::min(
                                 config.friction_mu * config.gravity,
                                 config.gravity * config.track_width / (2. * config.com_height));
        const double bank =
            std::atan(std::clamp(velocity_ * yaw_rate_, -limit, limit) / config.gravity);
        return {
            config.velocity_sign * velocity_, yaw_rate_, yaw_, bank,
            config.bank_to_roll_bias * bank};
    }

private:
    void step_channel(
        double& value, double& rate, double goal, double acceleration, double jerk) const {
        const double desired =
            std::clamp(config.channel_kp * (goal - value), -acceleration, acceleration);
        rate += std::clamp(desired - rate, -jerk * config.dt, jerk * config.dt);
        value += rate * config.dt;
    }
    double velocity_ = 0., accel_ = 0., yaw_rate_ = 0., yaw_accel_ = 0., yaw_ = 0.;
};

class TrajectoryPlanner {
public:
    struct Settings {
        double dt = 0.001;
        static constexpr double thigh_length = wheel_leg::mechanics::kThighLength;
        static constexpr double shank_length = wheel_leg::mechanics::kShankLength;
        static constexpr double wheel_radius = wheel_leg::mechanics::kWheelRadius;
        double nominal_leg_length = 0.334;
        static constexpr double body_mass = wheel_leg::mechanics::kBodyMass;
        static constexpr double thigh_mass = wheel_leg::mechanics::kThighMass;
        static constexpr double shank_mass = wheel_leg::mechanics::kShankMass;
        static constexpr double thigh_com_offset = wheel_leg::mechanics::kThighComOffset;
        static constexpr double shank_com_offset = wheel_leg::mechanics::kShankComOffset;
        static constexpr double body_com_offset = wheel_leg::mechanics::kBodyComForward;
        double transition_time_pitch = 0.15;
        double transition_time_shank = 4.0;
        double transition_time_knee = 4.0;
        double knee_branch_sign = 1.0;
        double knee_angle_min = 0.6981317007977318;
        double knee_angle_max = 1.9198621771937625;
        double singularity_epsilon = 0.001;
        double leg_length_epsilon = 0.01;
        double startup_settle_s = 0.0;
        double command_accel_limit = 1.5;
        double command_jerk_limit = 4.0;
        double command_vloop_kp = 10.0;
        double command_vloop_kd = 2.0;
        double leg_motion_coupling = 0.0;
        double outer_kx = 0.5;
        double outer_kv = 0.5;
        double com_height = 0.26;
        double accel_leg_ff = 0.0;
        double accel_d_kp = 0.0;
        double accel_theta_ff = 0.0;
        double accel_kv = 0.0;
        double accel_gate_min = 0.1;
        double d_max = 100.0;
        double gravity = 9.81;
        double balance_velocity_kp = 0.0;
        double balance_position_kp = 0.0;
        double balance_integral_limit = 5.0;
    } config;
    struct State {
        double wheel_velocity, pitch, pitch_velocity, com_offset, leg_length, velocity, distance;
        std::array<double, 2> knee, shank, knee_rate, shank_rate;
    };
    struct Reference {
        double pitch = 0., wheel_velocity = 0., balance_feedforward = 0.;
        std::array<double, 2> hip{}, knee{}, shank{};
    };
    void reset() {
        initialized_ = false;
        elapsed_ = since_enable_ = velocity_ref_ = accel_ref_ = distance_ref_ = wheel_integral_ =
            0.;
    }
    Reference step(const State& state, double velocity, double length) {
        const double dt = config.dt;
        const double acceleration = std::clamp(
            config.command_vloop_kp * (velocity - velocity_ref_)
                - config.command_vloop_kd * accel_ref_,
            -config.command_accel_limit, config.command_accel_limit);
        accel_ref_ += std::clamp(
            acceleration - accel_ref_, -config.command_jerk_limit * dt,
            config.command_jerk_limit * dt);
        velocity_ref_ += accel_ref_ * dt;
        distance_ref_ += velocity_ref_ * dt;
        const double a = accel_ref_ + config.outer_kv * (velocity_ref_ - state.velocity)
                       + config.outer_kx * (distance_ref_ - state.distance);
        length = std::clamp(length - config.leg_motion_coupling * velocity, .22, .45);
        const auto balance = solve_target_(velocity, length);
        auto target = balance;
        if (since_enable_ < config.startup_settle_s) {
            target.leg_length = state.leg_length;
            target.beta = .5 * (state.knee[0] + state.knee[1]);
            target.shank = .5 * (state.shank[0] + state.shank[1]);
        }
        const double dref = a * config.com_height / config.gravity, derr = dref - state.com_offset;
        const double gait = std::abs(a) > config.accel_gate_min ? 1. : 0.;
        const double dhip = gait * (config.accel_theta_ff * dref + config.accel_kv * derr);
        const auto gait_target = solve_target_(
            velocity,
            std::clamp(
                length + gait * (config.accel_leg_ff * dref + config.accel_d_kp * derr), .22, .45));
        const double dknee = gait_target.beta - balance.beta,
                     dshank = gait_target.shank - balance.shank;
        const bool guard = std::abs(state.com_offset) > config.d_max;
        if (guard)
            target.leg_length = state.leg_length;
        if (!initialized_
            || (!guard
                && std::abs(target.leg_length - planned_length_) > config.leg_length_epsilon)) {
            pitch_.reset(state.pitch, state.pitch_velocity, 0., 0., config.transition_time_pitch);
            for (int side = 0; side < 2; ++side) {
                const double knee_rate = .5 * (state.knee_rate[0] + state.knee_rate[1]);
                const double shank_rate = .5 * (state.shank_rate[0] + state.shank_rate[1]);
                knee_[side].reset(
                    state.knee[side], knee_rate, target.beta, 0., config.transition_time_knee);
                shank_[side].reset(
                    state.shank[side], shank_rate, target.shank, 0., config.transition_time_shank);
            }
            initialized_ = true;
            elapsed_ = 0.;
            planned_length_ = target.leg_length;
        } else
            elapsed_ += dt;
        Reference result;
        result.pitch = pitch_.value(elapsed_);
        double dx = 0.;
        for (int side = 0; side < 2; ++side) {
            const double knee = knee_[side].value(elapsed_), shank = shank_[side].value(elapsed_);
            result.knee[side] = knee + dknee;
            result.shank[side] = shank + dshank;
            result.hip[side] =
                std::numbers::pi - result.pitch - result.knee[side] - result.shank[side] + dhip;
            const double thigh = config.thigh_length * std::sin(knee + shank);
            dx +=
                (-thigh + config.shank_length * std::sin(shank)) * shank_[side].derivative(elapsed_)
                - thigh * knee_[side].derivative(elapsed_);
        }
        result.wheel_velocity = (velocity_ref_ + .5 * dx) / config.wheel_radius;
        wheel_integral_ = std::clamp(
            wheel_integral_ + state.wheel_velocity * dt, -config.balance_integral_limit,
            config.balance_integral_limit);
        result.balance_feedforward = -config.balance_velocity_kp * state.wheel_velocity
                                   - config.balance_position_kp * wheel_integral_;
        since_enable_ += dt;
        return result;
    }

private:
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN(), kEpsilon = 1e-9;
    struct Target {
        double leg_length = kNaN, beta = kNaN, shank = kNaN, hip = kNaN, wheel_velocity = kNaN;
        bool feasible = false;
    };
    struct Quintic {
        void reset(double y0, double dy0, double y1, double dy1, double duration) {
            duration_ = std::max(duration, kEpsilon);
            a0_ = y0;
            a1_ = duration_ * dy0;

            const double delta = y1 - y0;
            a3_ = 10.0 * delta - 6.0 * duration_ * dy0 - 4.0 * duration_ * dy1;
            a4_ = -15.0 * delta + 8.0 * duration_ * dy0 + 7.0 * duration_ * dy1;
            a5_ = 6.0 * delta - 3.0 * duration_ * dy0 - 3.0 * duration_ * dy1;
        }

        [[nodiscard]] double value(double t) const {
            const double u = std::clamp(t / duration_, 0.0, 1.0);
            const double u2 = u * u;
            const double u3 = u2 * u;
            const double u4 = u3 * u;
            const double u5 = u4 * u;
            return a0_ + a1_ * u + a3_ * u3 + a4_ * u4 + a5_ * u5;
        }

        [[nodiscard]] double derivative(double t) const {
            const double u = std::clamp(t / duration_, 0.0, 1.0);
            const double u2 = u * u;
            const double u3 = u2 * u;
            const double u4 = u3 * u;
            return (a1_ + 3.0 * a3_ * u2 + 4.0 * a4_ * u3 + 5.0 * a5_ * u4) / duration_;
        }

    private:
        double a0_ = 0.0, a1_ = 0.0, a3_ = 0.0, a4_ = 0.0, a5_ = 0.0;
        double duration_ = 1.0;
    };

    [[nodiscard]] Target solve_target_(double velocity_command, double leg_length_command) const {
        Target target;

        double leg_length =
            std::isfinite(leg_length_command) ? leg_length_command : config.nominal_leg_length;

        const double reach_min = std::abs(config.thigh_length - config.shank_length);
        const double reach_max = config.thigh_length + config.shank_length;
        const bool reachable = leg_length >= reach_min && leg_length <= reach_max;
        leg_length = std::clamp(leg_length, reach_min, reach_max);

        // §6.3: h_leg* = L0.
        const double cos_beta = std::clamp(
            (config.thigh_length * config.thigh_length + config.shank_length * config.shank_length
             - leg_length * leg_length)
                / (2.0 * config.thigh_length * config.shank_length),
            -1.0, 1.0);

        double beta = config.knee_branch_sign * std::acos(cos_beta);
        bool within_limits = beta >= config.knee_angle_min && beta <= config.knee_angle_max;
        if (!within_limits) {
            beta = std::clamp(beta, config.knee_angle_min, config.knee_angle_max);
            leg_length = std::sqrt(
                config.thigh_length * config.thigh_length
                + config.shank_length * config.shank_length
                - 2.0 * config.thigh_length * config.shank_length * std::cos(beta));
        }

        // Static balance coefficients, 执行路线1_数学解析.md §6.2.
        const double a1 = config.body_mass * config.thigh_length
                        + config.thigh_mass * (config.thigh_length - config.thigh_com_offset);
        const double a2 = (config.body_mass + config.thigh_mass) * config.shank_length
                        + config.shank_mass * (config.shank_length - config.shank_com_offset);

        // §6.4: P sin(phi) + Q cos(phi) = m_b c_b, Gamma = sqrt(P^2 + Q^2).
        const double p = -a1 * std::sin(beta);
        const double q = a1 * std::cos(beta) - a2;
        const double gamma = std::sqrt(p * p + q * q);

        double shank = kNaN;
        if (gamma > kEpsilon) {
            // P sin(phi) + Q cos(phi) = m_b c_b has TWO branches. The physically
            // correct one (matching the real-robot encoder / bench convention,
            // axle_line(beta, hip) ~ -90 deg at the nominal stance) is
            //   phi = pi - asin(m_b c_b / Gamma) - atan2(Q, P),
            // NOT asin(...) - atan2(...) (that is the pi-opposite branch, which
            // drove the real leg "straight up"). Wrap into (-pi, pi] so the
            // per-leg plan does not take the long way.
            shank = std::remainder(
                std::numbers::pi
                    - std::asin(
                        std::clamp(config.body_mass * config.body_com_offset / gamma, -1.0, 1.0))
                    - std::atan2(q, p),
                2.0 * std::numbers::pi);
        }

        const bool nonsingular = std::abs(std::sin(beta)) > config.singularity_epsilon;

        target.leg_length = leg_length;
        target.beta = beta;
        target.shank = shank;
        // §6.5: alpha* = pi - beta* - phi*.
        target.hip = std::numbers::pi - beta - shank;
        target.wheel_velocity = velocity_command / config.wheel_radius;
        target.feasible = reachable && within_limits && nonsingular;
        return target;
    }

    Quintic pitch_;
    std::array<Quintic, 2> knee_, shank_;
    bool initialized_ = false;
    double elapsed_ = 0., since_enable_ = 0., velocity_ref_ = 0., accel_ref_ = 0.,
           distance_ref_ = 0., wheel_integral_ = 0., planned_length_ = 0.;
};
} // namespace rmcs_core::controller::chassis
