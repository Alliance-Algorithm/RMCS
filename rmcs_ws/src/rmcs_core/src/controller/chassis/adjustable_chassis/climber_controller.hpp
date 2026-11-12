//  仿照/workspaces/RMCS/rmcs_ws/src/rmcs_core/src/controller/climber/climber_controller.hpp，应当可以被其他组件持有调用
//  前面两个master_lift_motor为前抬升电机
//  后面两个masterlift为后抬升电机
//  中间两个slavelift为辅助抬升电机
//  默认阶段IDLE，清空输出
//  第一阶段LIFT当进入爬台阶模式，所有电机都伸出，让机体达到最高软件限位
//  第二阶段APPROACHING靠近，依靠某个传感器得知是否已靠近台阶，此处暂时留空
//  第三阶段PRE_CLIMBING，抬起前两轮
//  第四阶段SLAVE_COLLAPSE，收起中间两轮
//  第五阶段MASTER_COLLAPSE，收起后面两轮
//
//  Each pair retracts by step_height from the lifted height, so the body keeps its world
//  height while that wheel pair rises onto the step. All stage changes are currently driven
//  by the rising edge of Input::advance (manual). After MASTER_COLLAPSE the final angles are
//  held for hold_ticks, then the controller returns to IDLE (outputs cleared to NaN).
//
//  Targets jump between stages without ramping: the lift controller's position-loop output
//  limit already caps leg speed.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <rmcs_utility/tick_timer.hpp>

namespace rmcs_core::controller::chassis::adjustable {

class ClimberController {
public:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    enum class State {
        IDLE,
        LIFT,
        APPROACHING,
        PRE_CLIMBING,
        SLAVE_COLLAPSE,
        MASTER_COLLAPSE,
        HOLD,
    };

    /// Soft limits of one leg group, output-shaft angle [rad].
    struct Group {
        double retracted_angle = 0.0; // lower soft limit (leg fully retracted)
        double extended_angle = 0.0;  // upper soft limit (leg fully extended, body highest)
        double lead = 0.0;            // leg travel per output-shaft radian [m/rad], > 0
    };

    struct Config {
        Group front_master; // master [0] LF, [3] RF
        Group back_master;  // master [1] LB, [2] RB
        Group slave;        // slave [0] left, [1] right
        double step_height = 0.0;      // [m]
        unsigned int hold_ticks = 500; // HOLD duration before IDLE (update ticks)
    };

    struct Input {
        bool advance = false; // manual "next stage" flag, rising-edge triggered
    };

    /// NaN = no command. Order matches /chassis/lift/master_{lf,lb,rb,rf} and slave_{left,right}.
    struct Output {
        std::array<double, 4> master_target_angle{nan_, nan_, nan_, nan_};
        std::array<double, 2> slave_target_angle{nan_, nan_};
    };

    explicit ClimberController(const Config& config)
        : config_(config)
        , front_retracted_(retracted_target(config.front_master, config.step_height))
        , back_retracted_(retracted_target(config.back_master, config.step_height))
        , slave_retracted_(retracted_target(config.slave, config.step_height)) {}

    void reset() { state_ = State::IDLE; } // keeps last_advance_ so edges stay consistent

    void abort() { reset(); }

    void start() {
        reset();
        enter_state(State::LIFT);
    }

    Output update(const Input& input) {
        const bool advance = input.advance && !last_advance_;
        last_advance_ = input.advance;

        switch (state_) {
        case State::IDLE: return {};
        case State::LIFT: return update_lift(advance);
        case State::APPROACHING: return update_approaching(advance);
        case State::PRE_CLIMBING: return update_pre_climbing(advance);
        case State::SLAVE_COLLAPSE: return update_slave_collapse(advance);
        case State::MASTER_COLLAPSE: return update_master_collapse(advance);
        case State::HOLD: return update_hold();
        }
        return {};
    }

    bool active() const { return state_ != State::IDLE; }

    State state() const { return state_; }

    const char* state_str() const { return state_to_string(state_); }

private:
    static const char* state_to_string(State state) {
        switch (state) {
        case State::IDLE: return "IDLE";
        case State::LIFT: return "LIFT";
        case State::APPROACHING: return "APPROACHING";
        case State::PRE_CLIMBING: return "PRE_CLIMBING";
        case State::SLAVE_COLLAPSE: return "SLAVE_COLLAPSE";
        case State::MASTER_COLLAPSE: return "MASTER_COLLAPSE";
        case State::HOLD: return "HOLD";
        }
        return "UNKNOWN";
    }

    // Extended angle moved toward the retracted limit by step_height, clamped to the limits.
    // Works whichever way the angle increases. Invalid lead -> fully retracted.
    static double retracted_target(const Group& group, double step_height) {
        if (!(group.lead > 0.0) || !std::isfinite(step_height))
            return group.retracted_angle;

        const double dir = group.extended_angle >= group.retracted_angle ? 1.0 : -1.0;
        const double target = group.extended_angle - dir * step_height / group.lead;
        return std::clamp(
            target, std::min(group.retracted_angle, group.extended_angle),
            std::max(group.retracted_angle, group.extended_angle));
    }

    void enter_state(State state) {
        state_ = state;
        if (state == State::HOLD)
            hold_timer_.reset(config_.hold_ticks);
    }

    Output make_output(bool front_retracted, bool slave_retracted, bool back_retracted) const {
        const double front =
            front_retracted ? front_retracted_ : config_.front_master.extended_angle;
        const double back = back_retracted ? back_retracted_ : config_.back_master.extended_angle;
        const double slave = slave_retracted ? slave_retracted_ : config_.slave.extended_angle;

        Output output;
        output.master_target_angle = {front, back, back, front}; // LF, LB, RB, RF
        output.slave_target_angle = {slave, slave};
        return output;
    }

    // Stage 1: extend everything to the upper soft limit.
    Output update_lift(bool advance) {
        if (advance)
            enter_state(State::APPROACHING);
        return make_output(false, false, false);
    }

    // Stage 2: drive up to the step at full height.
    Output update_approaching(bool advance) {
        // TODO: replace with step-proximity sensor
        if (advance)
            enter_state(State::PRE_CLIMBING);
        return make_output(false, false, false);
    }

    // Stage 3: lift the front wheels onto the step.
    Output update_pre_climbing(bool advance) {
        if (advance)
            enter_state(State::SLAVE_COLLAPSE);
        return make_output(true, false, false);
    }

    // Stage 4: lift the middle (slave) wheels onto the step.
    Output update_slave_collapse(bool advance) {
        if (advance)
            enter_state(State::MASTER_COLLAPSE);
        return make_output(true, true, false);
    }

    // Stage 5: lift the rear wheels onto the step.
    Output update_master_collapse(bool advance) {
        if (advance)
            enter_state(State::HOLD);
        return make_output(true, true, true);
    }

    // Hold the final angles for hold_ticks updates, then clear outputs.
    Output update_hold() {
        if (hold_timer_.tick()) {
            reset();
            return {};
        }
        return make_output(true, true, true);
    }

    Config config_;
    double front_retracted_;
    double back_retracted_;
    double slave_retracted_;

    State state_ = State::IDLE;
    bool last_advance_ = false;
    rmcs_utility::TickTimer hold_timer_;
};

} // namespace rmcs_core::controller::chassis::adjustable
