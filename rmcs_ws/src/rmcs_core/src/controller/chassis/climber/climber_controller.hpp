#pragma once

#include <cmath>
#include <cstdlib>
#include <limits>
#include <rclcpp/logging.hpp>

namespace rmcs_core::controller::chassis::climber {

class ClimberController {
public:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    enum class Mode { OneStair, TwoStairs };

    enum class State { IDLE, APPROACH, SUPPORT_DEPLOY, DASH, SUPPORT_RETRACT };

    struct Config {
        double track_velocity_max                = 0.0;
        double climber_back_control_velocity_abs = 0.0;
        double support_retract_velocity_abs      = 0.0;
        double approach_chassis_velocity         = 0.0;
        double support_deploy_chassis_velocity   = 0.0;
        double dash_chassis_velocity             = 0.0;
        double leveled_pitch_threshold           = 0.0;
        double first_stair_approach_pitch        = 0.0;
        double second_stair_approach_pitch       = 0.0;
        double back_blocked_torque_threshold     = 0.1;
        double back_blocked_velocity_threshold   = 0.1;
        int support_confirm_ticks                = 100;
        int dash_min_ticks                       = 500;
        int dash_timeout_ticks                   = 3000;
        int support_retract_ticks                = 1500;
    };

    struct Input {
        double chassis_pitch_imu           = nan_;
        double climber_back_left_torque    = nan_;
        double climber_back_right_torque   = nan_;
        double climber_back_left_velocity  = nan_;
        double climber_back_right_velocity = nan_;
    };

    struct Output {
        double front_track_velocity  = nan_;
        double back_climber_velocity = nan_;
        double override_chassis_vx   = nan_;
    };

    explicit ClimberController(Config config)
        : config_(config) {}

    void reset() {
        state_               = State::IDLE;
        timer_               = 0;
        stair_index_         = 0;
        support_block_count_ = 0;
    }

    void abort() { reset(); }

    void start(Mode mode) {
        reset();
        stair_index_ = mode == Mode::TwoStairs ? 1 : 0;
        enter_state(State::APPROACH);
    }

    Output update(const Input& input) {
        if (state_ == State::IDLE)
            return {};

        ++timer_;

        switch (state_) {
        case State::IDLE: return {};
        case State::APPROACH: return update_approach(input);
        case State::SUPPORT_DEPLOY: return update_support_deploy(input);
        case State::DASH: return update_dash(input);
        case State::SUPPORT_RETRACT: return update_support_retract();
        }

        return {};
    }

    bool active() const { return state_ != State::IDLE; }

    State state() const { return state_; }

    int stair_index() const { return stair_index_; }

private:
    void enter_state(State state) {
        if (state == state_)
            return;

        state_               = state;
        timer_               = 0;
        support_block_count_ = 0;
    }

    bool is_back_climber_blocked(const Input& input) const {
        return (std::abs(input.climber_back_left_torque) > config_.back_blocked_torque_threshold
                && std::abs(input.climber_back_left_velocity)
                       < config_.back_blocked_velocity_threshold)
            || (std::abs(input.climber_back_right_torque) > config_.back_blocked_torque_threshold
                && std::abs(input.climber_back_right_velocity)
                       < config_.back_blocked_velocity_threshold);
    }

    Output update_approach(const Input& input) {
        Output output{
            .front_track_velocity  = config_.track_velocity_max,
            .back_climber_velocity = 0.0,
            .override_chassis_vx   = config_.approach_chassis_velocity,
        };

        const double target_pitch = std::abs(
            stair_index_ == 0 ? config_.first_stair_approach_pitch
                              : config_.second_stair_approach_pitch);

        if (std::abs(input.chassis_pitch_imu) > target_pitch)
            enter_state(State::SUPPORT_DEPLOY);

        return output;
    }

    Output update_support_deploy(const Input& input) {
        Output output{
            .front_track_velocity  = 0.0,
            .back_climber_velocity = config_.climber_back_control_velocity_abs,
            .override_chassis_vx   = config_.support_deploy_chassis_velocity,
        };

        // if (is_back_climber_blocked(input)) // TODO: judge by pitch_imu_angle_imu_
        //     ++support_block_count_;
        // else
        //     support_block_count_ = 0;

        if (std::abs(input.chassis_pitch_imu) < 0.015)
            enter_state(State::DASH);

        return output;
    }

    Output update_dash(const Input& input) {
        Output output{
            .front_track_velocity  = 0.0,
            .back_climber_velocity = config_.climber_back_control_velocity_abs,
            .override_chassis_vx   = config_.dash_chassis_velocity,
        };

        const bool is_leveled = std::abs(input.chassis_pitch_imu) < config_.leveled_pitch_threshold
                             && timer_ > config_.dash_min_ticks;
        const bool timeout = timer_ > config_.dash_timeout_ticks;

        if (is_leveled || timeout)
            enter_state(State::SUPPORT_RETRACT);

        return output;
    }

    Output update_support_retract() {
        Output output{
            .front_track_velocity  = 0.0,
            .back_climber_velocity = -config_.support_retract_velocity_abs,
            .override_chassis_vx   = 0.0,
        };

        if (timer_ > config_.support_retract_ticks)
            reset();

        return output;
    }

    Config config_;
    State state_             = State::IDLE;
    int timer_               = 0;
    int stair_index_         = 0;
    int support_block_count_ = 0;
};

} // namespace rmcs_core::controller::chassis::climber
