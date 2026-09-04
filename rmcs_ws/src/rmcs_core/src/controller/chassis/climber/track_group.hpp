#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <eigen3/Eigen/Dense>
#include <rmcs_executor/component.hpp>

#include "controller/pid/matrix_pid_calculator.hpp"

namespace rmcs_core::controller::chassis::climber {

struct TrackGroup {
    static constexpr auto kNaN = std::numeric_limits<double>::quiet_NaN();

    template <typename T>
    using InputInterface = rmcs_executor::Component::InputInterface<T>;

    template <typename T>
    using OutputInterface = rmcs_executor::Component::OutputInterface<T>;

    enum class State {
        kFree,
        kHold,
        kRush,
    } state = State::kFree;

    struct Config {
        double speed_rush;

        double kp;
        double ki;
        double kd;
        double sync_coefficient;

        double power_estimate_bias;
        double power_estimate_k_tau2;
        double power_estimate_k_mech;

        auto get_speed(State state) const noexcept {
            switch (state) {
            case State::kFree: return kNaN;
            case State::kHold: return 0.0;
            case State::kRush: return speed_rush;
            }
            std::unreachable();
        }
    } config;

    rmcs_executor::Component& command;

    // Interfaces
    InputInterface<double> l_velocity;
    InputInterface<double> r_velocity;
    InputInterface<double> l_max_torque;
    InputInterface<double> r_max_torque;
    InputInterface<double> control_power_limit;

    OutputInterface<double> l_control_torque;
    OutputInterface<double> r_control_torque;

    // PID
    pid::MatrixPidCalculator<2> velocity_pid;

    explicit TrackGroup(rmcs_executor::Component& command, const Config& config)
        : config{config}
        , command{command}
        , velocity_pid{config.kp, config.ki, config.kd} {

        command.register_input("/chassis/climber/left_front_motor/velocity", l_velocity);
        command.register_input("/chassis/climber/right_front_motor/velocity", r_velocity);
        command.register_input("/chassis/climber/left_front_motor/max_torque", l_max_torque);
        command.register_input("/chassis/climber/right_front_motor/max_torque", r_max_torque);
        command.register_input("/chassis/climber/front/control_power_limit", control_power_limit);

        command.register_output(
            "/chassis/climber/left_front_motor/control_torque", l_control_torque, kNaN);
        command.register_output(
            "/chassis/climber/right_front_motor/control_torque", r_control_torque, kNaN);
    }

    auto spin_once() {
        const auto target_speed = config.get_speed(state);

        if (std::isnan(target_speed)) {
            *l_control_torque = kNaN;
            *r_control_torque = kNaN;
            return;
        }

        auto torque = Eigen::Vector2d{};
        {
            const auto setpoint_error = Eigen::Vector2d{
                target_speed - *l_velocity,
                target_speed - *r_velocity,
            };
            const auto relative_speed = Eigen::Vector2d{
                *l_velocity - *r_velocity,
                *r_velocity - *l_velocity,
            };

            torque = velocity_pid.update(setpoint_error - config.sync_coefficient * relative_speed);
        }

        {
            const auto power_limit = *control_power_limit;
            if (power_limit <= 0.0) {
                *l_control_torque = 0.0;
                *r_control_torque = 0.0;
                return;
            }

            auto l_torque = std::clamp(torque[0], -*l_max_torque, *l_max_torque);
            auto r_torque = std::clamp(torque[1], -*r_max_torque, *r_max_torque);

            const auto estimated_power =
                config.power_estimate_bias
                + config.power_estimate_k_tau2 * (std::pow(l_torque, 2) + std::pow(r_torque, 2))
                + config.power_estimate_k_mech
                      * (std::abs(l_torque * *l_velocity) + std::abs(r_torque * *r_velocity));

            if (estimated_power > power_limit && estimated_power > 0.0) {
                const auto scale = std::clamp(power_limit / estimated_power, 0.0, 1.0);
                l_torque *= scale;
                r_torque *= scale;
            }

            *l_control_torque = l_torque;
            *r_control_torque = r_torque;
        }
    }

    auto set_state(State target) {
        if (state == target)
            return;

        state = target;
        velocity_pid.reset();
    }

    auto get_state() const noexcept { return state; }
};

} // namespace rmcs_core::controller::chassis::climber
