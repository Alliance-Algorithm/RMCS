#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

#include <eigen3/Eigen/Dense>
#include <rmcs_executor/component.hpp>

#include "controller/pid/matrix_pid_calculator.hpp"

namespace rmcs_core::controller::chassis::climber {

struct StickGroup {
    static constexpr auto kNaN = std::numeric_limits<double>::quiet_NaN();

    template <typename T>
    using InputInterface = rmcs_executor::Component::InputInterface<T>;

    template <typename T>
    using OutputInterface = rmcs_executor::Component::OutputInterface<T>;

    enum class State {
        kFree,
        kHold,
        kDrop,
        kRise,
        kLand,
        kKeep,
    } state = State::kFree;

    struct Config {
        double speed_drop;
        double speed_rise;

        double rise_torque_limit;

        // kLand：begin→final 速度变化时间(s)；越小越快贴到 final；t≥T 后恒 final 缓收
        double land_speed_begin;
        double land_speed_final;
        double land_duration;
        double land_torque_limit;

        double blocked_torque_threshold;
        double blocked_speed_threshold;

        double kp;
        double ki;
        double kd;
        double sync_coefficient;
        double hold_torque;

        auto get_speed(State state, double land_elapsed = 0.0) const noexcept {
            switch (state) {
            case State::kFree: return kNaN;
            case State::kHold: return 0.0;
            case State::kDrop: return +speed_drop;
            case State::kRise: [[fallthrough]];
            case State::kKeep: return -speed_rise;
            case State::kLand: {
                // 指数进度 α：t=0 → begin，t≥T → final，前快后慢减震
                constexpr auto kShape = 3.0;
                const auto T = std::max(land_duration, 1e-3);
                if (land_elapsed >= T)
                    return -land_speed_final;

                const auto u = land_elapsed / T;
                const auto alpha = (1.0 - std::exp(-kShape * u)) / (1.0 - std::exp(-kShape));
                return -(land_speed_begin + (land_speed_final - land_speed_begin) * alpha);
            }
            }
            std::unreachable();
        }

        auto get_torque_limit(State state) const noexcept {
            switch (state) {
            case State::kRise: [[fallthrough]];
            case State::kKeep: return rise_torque_limit;
            case State::kLand: return land_torque_limit;
            default: return std::numeric_limits<double>::infinity();
            }
        }
    } config;

    rmcs_executor::Component& command;

    // Interfaces
    InputInterface<double> l_velocity;
    InputInterface<double> r_velocity;
    InputInterface<double> l_torque;
    InputInterface<double> r_torque;

    OutputInterface<double> l_control_torque;
    OutputInterface<double> r_control_torque;

    // PID
    pid::MatrixPidCalculator<2> velocity_pid;

    std::chrono::steady_clock::time_point land_start_timestamp;

    explicit StickGroup(rmcs_executor::Component& command, const Config& config)
        : config{config}
        , command{command}
        , velocity_pid{config.kp, config.ki, config.kd} {

        command.register_input("/chassis/climber/left_back_motor/velocity", l_velocity);
        command.register_input("/chassis/climber/right_back_motor/velocity", r_velocity);
        command.register_input("/chassis/climber/left_back_motor/torque", l_torque);
        command.register_input("/chassis/climber/right_back_motor/torque", r_torque);

        command.register_output(
            "/chassis/climber/left_back_motor/control_torque", l_control_torque, kNaN);
        command.register_output(
            "/chassis/climber/right_back_motor/control_torque", r_control_torque, kNaN);
    }

    auto spin_once() {
        if (state == State::kKeep && get_block()) {
            *l_control_torque = -config.hold_torque;
            *r_control_torque = -config.hold_torque;
            return;
        }

        const auto land_elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - land_start_timestamp)
                .count();
        const auto target_speed = config.get_speed(state, land_elapsed);

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
            const auto relative_velocity = Eigen::Vector2d{
                *l_velocity - *r_velocity,
                *r_velocity - *l_velocity,
            };

            torque =
                velocity_pid.update(setpoint_error - config.sync_coefficient * relative_velocity);
        }

        {
            const auto torque_limit = config.get_torque_limit(state);

            if (target_speed < 0.0 && std::isfinite(torque_limit) && torque_limit > 0.0) {
                const auto peak = std::max(std::abs(torque[0]), std::abs(torque[1]));
                if (peak > torque_limit) {
                    torque *= torque_limit / peak;
                }
            }

            *l_control_torque = torque[0];
            *r_control_torque = torque[1];
        }
    }

    auto set_state(State target) {
        if (state == target)
            return;

        state = target;
        velocity_pid.reset();

        if (target == State::kLand) {
            land_start_timestamp = std::chrono::steady_clock::now();
        }
    }

    auto get_state() const noexcept { return state; }

    auto get_block() const noexcept -> bool {
        const auto is_blocked = [](double torque, double velocity, double torque_threshold,
                                   double velocity_threshold) {
            return std::abs(torque) > torque_threshold && std::abs(velocity) < velocity_threshold;
        };

        return is_blocked(
                   *l_torque, *l_velocity, config.blocked_torque_threshold,
                   config.blocked_speed_threshold)
            || is_blocked(
                   *r_torque, *r_velocity, config.blocked_torque_threshold,
                   config.blocked_speed_threshold);
    }
};

} // namespace rmcs_core::controller::chassis::climber
