#ifndef HSM_UP_STAIRS_HPP_H_
#define HSM_UP_STAIRS_HPP_H_

#include "hardware/device/trajectory.hpp"
#include "hardware/hsm/HSM.hpp"
#include <cassert>
#include <optional>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <variant>
#include <vector>

using InputD = rmcs_executor::Component::InputInterface<double>;
using InputVct =
    rmcs_executor::Component::InputInterface<rmcs_description::BaseLink::DirectionVector>;
enum class UpStairsState {
    Initial,
    StepByOne,
    StepByTwo,
};

enum class StepSubState {
    Wait,
    Press,
    Lift,
    PressAndLift,
    PressAgain,
    LiftAgain,
    InitialAgain,
    Delay
};
enum class UpStairsEventId {
    quit,
    stop,
    tick,
    go_to_OneProcess,
    go_to_TwoProcess,
    go_to_Press_And_Lift,
    go_to_Press,
    go_to_Lift,
    tof_already, // args:: double distance
    go_to_Initial_Again
};

namespace rmcs_core::hardware::hsm::up_stairs_hsm {

namespace events {
using UpStairsEvent = Event<UpStairsEventId>;
const UpStairsEvent tick{UpStairsEventId::tick, {}};

const UpStairsEvent quit{UpStairsEventId::quit, {}};

const UpStairsEvent stop{UpStairsEventId::stop, {}};

const UpStairsEvent go_to_OneProcess{UpStairsEventId::go_to_OneProcess, {}};

const UpStairsEvent go_to_TwoProcess{UpStairsEventId::go_to_TwoProcess, {}};

const UpStairsEvent go_to_Press_And_Lift{UpStairsEventId::go_to_Press_And_Lift, {}};

const UpStairsEvent go_to_Press{UpStairsEventId::go_to_Press, {}};

const UpStairsEvent go_to_Lift{UpStairsEventId::go_to_Lift, {}};

const UpStairsEvent go_to_Initial_Again{UpStairsEventId::go_to_Initial_Again, {}};

} // namespace events

struct UpStairsContext {

    InputD* theta_lf = nullptr;
    InputD* theta_lb = nullptr;
    InputD* theta_rb = nullptr;
    InputD* theta_rf = nullptr;
    InputVct* speed  = nullptr;

    static constexpr double v_reference = 1.5;
    std::vector<double> k;
    std::vector<double> b;
    long int count;

    hardware::device::Trajectory<device::TrajectoryType::JOINT>& initial;
    hardware::device::Trajectory<device::TrajectoryType::JOINT>& press;
    hardware::device::Trajectory<device::TrajectoryType::JOINT>& lift;
    hardware::device::Trajectory<device::TrajectoryType::JOINT>& press_and_lift;
    hardware::device::Trajectory<device::TrajectoryType::JOINT>& initial_again;

    std::array<double, 6>& result;

    UpStairsContext(
        hardware::device::Trajectory<device::TrajectoryType::JOINT>& initial_,
        hardware::device::Trajectory<device::TrajectoryType::JOINT>& press_,
        hardware::device::Trajectory<device::TrajectoryType::JOINT>& lift_,
        hardware::device::Trajectory<device::TrajectoryType::JOINT>& press_and_lift_,
        hardware::device::Trajectory<device::TrajectoryType::JOINT>& initial_again_,
        std::array<double, 6>& result_buf_)
        : initial(initial_)
        , press(press_)
        , lift(lift_)
        , press_and_lift(press_and_lift_)
        , initial_again(initial_again_)
        , result(result_buf_) {}

    int calculate_steps(double k, double b) const {
        double raw =
            static_cast<double>(k) * ((**speed)->x() - v_reference) + static_cast<double>(b);
        std::max(static_cast<int>(raw), 1);
    }
};

class WaitState final : public IState<StepSubState, UpStairsEventId, UpStairsContext> {
public:
    StepSubState getStateID() const override { return StepSubState::Wait; }

    std::optional<StepSubState>
        handleEvent(const Event<UpStairsEventId>& event, UpStairsContext& ctx) override {
        if (event.id == UpStairsEventId::go_to_Press_And_Lift) {
            return StepSubState::PressAndLift;
        }
        if (event.id == UpStairsEventId::go_to_Press) {
            return StepSubState::Press;
        }
        if (event.id == UpStairsEventId::go_to_Lift) {
            return StepSubState::Lift;
        }
        if (event.id == UpStairsEventId::tick) {
            // 保持最后姿态（什么都不改也不行，至少明确）
            // ctx.result[0] = **ctx.theta_lf;
            // ctx.result[1] = **ctx.theta_lb;
            // ctx.result[2] = **ctx.theta_rb;
            // ctx.result[3] = **ctx.theta_rf;
            return StepSubState::Wait;
        }

        if (event.id == UpStairsEventId::go_to_Initial_Again) {
            return StepSubState::InitialAgain;
        }
        return std::nullopt;
    }
};

class DelayState final : public IState<StepSubState, UpStairsEventId, UpStairsContext> {
public:
    StepSubState getStateID() const override { return StepSubState::Delay; }




    void enter(UpStairsContext& ctx, const EventArgs&) override {
        ctx.count=0;
    }












    std::optional<StepSubState>
        handleEvent(const Event<UpStairsEventId>& event, UpStairsContext& ctx) override {
        if (event.id == UpStairsEventId::go_to_Press_And_Lift) {
            return StepSubState::PressAndLift;
        }
        if (event.id == UpStairsEventId::go_to_Press) {
            return StepSubState::Press;
        }
        if (event.id == UpStairsEventId::go_to_Lift) {
            return StepSubState::Lift;
        }
        if (event.id == UpStairsEventId::tick) {
            ctx.count++;
            if (ctx.count <= 500) {
                return StepSubState::Delay;
            } else {
                return StepSubState::Lift;
            }
        }

        if (event.id == UpStairsEventId::go_to_Initial_Again) {
            return StepSubState::InitialAgain;
        }
        return std::nullopt;
    }
};

class PressAndLiftState final : public IState<StepSubState, UpStairsEventId, UpStairsContext> {
public:
    StepSubState getStateID() const override { return StepSubState::PressAndLift; }

    void enter(UpStairsContext& ctx, const EventArgs&) override {
        ctx.press_and_lift.reset();
        ctx.press_and_lift.set_start_point(
            std::array<double, 6>{
                **ctx.theta_lf, **ctx.theta_lb, **ctx.theta_rb, **ctx.theta_rf, 0.0, 0.0});
        // ctx.press_and_lift.set_total_step(static_cast<double>(ctx.calculate_steps(ctx.k[2],
        // ctx.b[2])));
        ctx.press_and_lift.set_total_step(2100);
    }

    std::optional<StepSubState>
        handleEvent(const Event<UpStairsEventId>& event, UpStairsContext& ctx) override {
        if (event.id == UpStairsEventId::tick) {
            if (!ctx.press_and_lift.get_complete()) {
                ctx.result = ctx.press_and_lift.trajectory();
            } else {
                ctx.result[0] = 0.8;
                ctx.result[3] = 0.8;
                return StepSubState::Wait;
            }
        }
        return std::nullopt;
    }
};

class PressState final : public IState<StepSubState, UpStairsEventId, UpStairsContext> {
public:
    StepSubState getStateID() const override { return StepSubState::Press; }

    void enter(UpStairsContext& ctx, const EventArgs&) override {
        ctx.press.reset();
        ctx.press.set_start_point(
            std::array<double, 6>{
                **ctx.theta_lf, **ctx.theta_lb, **ctx.theta_rb, **ctx.theta_rf, 0.0, 0.0});
        ctx.press.set_total_step(ctx.calculate_steps(ctx.k[0], ctx.b[0]));
        // ctx.press.set_total_step(1000);
    }

    std::optional<StepSubState>
        handleEvent(const Event<UpStairsEventId>& event, UpStairsContext& ctx) override {
        if (event.id == UpStairsEventId::tick) {
            if (!ctx.press.get_complete()) {
                ctx.result = ctx.press.trajectory();
                return StepSubState::Press;
            } else {
                return StepSubState::Delay;
                // return StepSubState::Wait;
            }
        }
        return std::nullopt;
    }
};

class PressAgainState final : public IState<StepSubState, UpStairsEventId, UpStairsContext> {
public:
    StepSubState getStateID() const override { return StepSubState::PressAgain; }

    void enter(UpStairsContext& ctx, const EventArgs&) override {
        ctx.press.reset();
        ctx.press.set_start_point(
            std::array<double, 6>{
                **ctx.theta_lf, **ctx.theta_lb, **ctx.theta_rb, **ctx.theta_rf, 0.0, 0.0});
        ctx.press.set_total_step(ctx.calculate_steps(ctx.k[0], ctx.b[0]));
        // ctx.press.set_total_step(1000);
    }

    std::optional<StepSubState>
        handleEvent(const Event<UpStairsEventId>& event, UpStairsContext& ctx) override {
        if (event.id == UpStairsEventId::tick) {
            if (!ctx.press.get_complete()) {
                ctx.result = ctx.press.trajectory();
                return StepSubState::PressAgain;
            } else {
                return StepSubState::LiftAgain;
                // return StepSubState::Wait;
            }
        }
        return std::nullopt;
    }
};

class LiftState final : public IState<StepSubState, UpStairsEventId, UpStairsContext> {
public:
    StepSubState getStateID() const override { return StepSubState::Lift; }

    void enter(UpStairsContext& ctx, const EventArgs&) override {
        ctx.lift.reset();
        ctx.lift.set_start_point(
            std::array<double, 6>{
                **ctx.theta_lf, **ctx.theta_lb, **ctx.theta_rb, **ctx.theta_rf, 0.0, 0.0});
        ctx.lift.set_total_step(ctx.calculate_steps(ctx.k[1], ctx.b[1]));
        // ctx.lift.set_total_step(2000);
    }

    std::optional<StepSubState>
        handleEvent(const Event<UpStairsEventId>& event, UpStairsContext& ctx) override {
        if (event.id == UpStairsEventId::tick) {
            if (!ctx.lift.get_complete()) {
                ctx.result = ctx.lift.trajectory();
                return StepSubState::Lift;
            } else {
                // return StepSubState::Press; // 回到 Press 循环
                // return StepSubState::Wait;
                return StepSubState::InitialAgain;
            }
        }
        return std::nullopt; // 没有处理到事件
    }
};

class LiftAgainState final : public IState<StepSubState, UpStairsEventId, UpStairsContext> {
public:
    StepSubState getStateID() const override { return StepSubState::LiftAgain; }

    void enter(UpStairsContext& ctx, const EventArgs&) override {
        ctx.lift.reset();
        ctx.lift.set_start_point(
            std::array<double, 6>{
                **ctx.theta_lf, **ctx.theta_lb, **ctx.theta_rb, **ctx.theta_rf, 0.0, 0.0});
        ctx.lift.set_total_step(ctx.calculate_steps(ctx.k[1], ctx.b[1]));
        // ctx.lift.set_total_step(2000);
    }

    std::optional<StepSubState>
        handleEvent(const Event<UpStairsEventId>& event, UpStairsContext& ctx) override {
        if (event.id == UpStairsEventId::tick) {
            if (!ctx.lift.get_complete()) {
                ctx.result = ctx.lift.trajectory();
                return StepSubState::LiftAgain;
            } else {
                //return StepSubState::PressAgain; // 回到 Press 循环
                 return StepSubState::Wait;
            }
        }
        return std::nullopt; // 没有处理到事件
    }
};

class InitialAgainState final : public IState<StepSubState, UpStairsEventId, UpStairsContext> {
public:
    StepSubState getStateID() const override { return StepSubState::InitialAgain; }

    void enter(UpStairsContext& ctx, const EventArgs&) override {
        ctx.initial_again.reset();
        ctx.initial_again.set_start_point(
            std::array<double, 6>{
                **ctx.theta_lf, **ctx.theta_lb, **ctx.theta_rb, **ctx.theta_rf, 0.0, 0.0});
        // ctx.lift.set_total_step(static_cast<double>(ctx.calculate_steps(ctx.k[1], ctx.b[1])));
        ctx.initial_again.set_total_step(ctx.calculate_steps(ctx.k[2], ctx.b[2]));
    }

    std::optional<StepSubState>
        handleEvent(const Event<UpStairsEventId>& event, UpStairsContext& ctx) override {
        if (event.id == UpStairsEventId::tick) {
            if (!ctx.initial_again.get_complete()) {
                ctx.result = ctx.initial_again.trajectory();
                return StepSubState::InitialAgain;
            } else {
                return StepSubState::PressAgain; // 回到 Press 循环
                // return StepSubState::Wait;
            }
        }
        return std::nullopt; // 没有处理到事件
    }
};

class InitialState final : public IState<UpStairsState, UpStairsEventId, UpStairsContext> {
public:
    UpStairsState getStateID() const override { return UpStairsState::Initial; }

    void enter(UpStairsContext& ctx, const EventArgs&) override {
        ctx.initial.reset();
        ctx.initial.set_start_point(
            std::array<double, 6>{
                **ctx.theta_lf, **ctx.theta_lb, **ctx.theta_rb, **ctx.theta_rf, 0.0, 0.0});
        ctx.initial.set_total_step(1000);

    }

    std::optional<UpStairsState>
        handleEvent(const Event<UpStairsEventId>& event, UpStairsContext& ctx) override {
        if (event.id == UpStairsEventId::tick) {
            if (!ctx.initial.get_complete()) {
                ctx.result = ctx.initial.trajectory();
                return UpStairsState::Initial;
            } else {
                // 由外层决定进入 StepByOne/StepByTwo
                return std::nullopt; // 轨迹处理完了之后没有处理其他事件
            }
        }
        if (event.id == UpStairsEventId::go_to_OneProcess) {
            return UpStairsState::StepByOne;
        }
        if (event.id == UpStairsEventId::go_to_TwoProcess) {
            return UpStairsState::StepByTwo;
        }

        return std::nullopt;         // 没有tick到
    }
};

class StepByOneState
    : public CompositeState<UpStairsState, StepSubState, UpStairsEventId, UpStairsContext> {
public:
    UpStairsState getStateID() const override { return UpStairsState::StepByOne; }

    void onRegister(UpStairsContext& ctx) override {
        createInnerHsm(ctx);
        registerInnerState<WaitState>();
        registerInnerState<PressAndLiftState>();
        setInitialSubState(StepSubState::Wait);
    }
};

class StepByTwoState
    : public CompositeState<UpStairsState, StepSubState, UpStairsEventId, UpStairsContext> {
public:
    UpStairsState getStateID() const override { return UpStairsState::StepByTwo; }

    void onRegister(UpStairsContext& ctx) override {
        createInnerHsm(ctx);
        registerInnerState<PressState>();
        registerInnerState<LiftState>();
        registerInnerState<WaitState>();
        registerInnerState<PressAgainState>();
        registerInnerState<LiftAgainState>();
        registerInnerState<InitialAgainState>();
        registerInnerState<DelayState>();
        setInitialSubState(StepSubState::Press);
    }
};

class Auto_Leg_Up_Stairs : public rclcpp::Node {
public:
    explicit Auto_Leg_Up_Stairs()
        : context_(
              up_stairs_initial, up_stairs_leg_press, up_stairs_leg_lift,
              up_stairs_leg_press_and_lift, up_stairs_leg_initial_again, result)
        , up_stairs_hsm(context_)
        , Node{"auto_leg_up_stairs"} {}

    void processEvent(const Event<UpStairsEventId>& event) {
        if (static_cast<int>(event.id) != 2) {
            RCLCPP_INFO(this->get_logger(), "Processing event: %d", static_cast<int>(event.id));
        }
        up_stairs_hsm.processEvent(event);
    }

    void start(UpStairsState initial, const EventArgs& args = {}) {

        check_context_ready();
        context_.count =0;
        up_stairs_hsm.start(initial, args);
    }

    void stop() { up_stairs_hsm.stop(); }

    void check_context_ready() const {
        assert(context_.theta_lf && "theta_lf not bound");
        assert(context_.theta_lb && "theta_lb not bound");
        assert(context_.theta_rb && "theta_rb not bound");
        assert(context_.theta_rf && "theta_rf not bound");
        assert(context_.speed && "speed not bound");
    }

    Auto_Leg_Up_Stairs&
        bind_real_theta_and_speed(InputD& lf, InputD& lb, InputD& rb, InputD& rf, InputVct& speed) {
        context_.theta_lf = &lf;
        context_.theta_lb = &lb;
        context_.theta_rb = &rb;
        context_.theta_rf = &rf;
        context_.speed    = &speed;
        return *this;
    }

    Auto_Leg_Up_Stairs&
        bind_k_and_b_parameter(const std::vector<double>& k_, const std::vector<double>& b_) {
        context_.k = k_;
        context_.b = b_;
        return *this;
    }

    // Auto_Leg_Up_Stairs& bind_results(std::array<double, 6>* result_) {
    //     result_ = &context_.result;
    //     return *this;
    // }

    std::array<double, 6>& get_result() { return context_.result; }

    Auto_Leg_Up_Stairs& init_and_trajectory_set(
        const std::vector<double>& initial_end_point_, const std::vector<double>& press_end_point_,
        const std::vector<double>& lift_end_point_,
        const std::vector<double>& press_and_lift_end_point_,
        const std::vector<double>& initial_again_end_point_) {

        up_stairs_hsm.registerState<InitialState>();
        up_stairs_hsm.registerState<StepByOneState>();
        up_stairs_hsm.registerState<StepByTwoState>();

        up_stairs_initial.set_end_point(
            {initial_end_point_[0], initial_end_point_[1], initial_end_point_[2],
             initial_end_point_[3], 0, 0});
        up_stairs_leg_press.set_end_point(
            {press_end_point_[0], press_end_point_[1], press_end_point_[2], press_end_point_[3], 0,
             0});
        //.set_total_step(1000);
        up_stairs_leg_lift.set_end_point(
            {lift_end_point_[0], lift_end_point_[1], lift_end_point_[2], lift_end_point_[3], 0, 0});
        //.set_total_step(3100);
        up_stairs_leg_press_and_lift.set_end_point(
            {press_and_lift_end_point_[0], press_and_lift_end_point_[1],
             press_and_lift_end_point_[2], press_and_lift_end_point_[3], 0, 0});

        up_stairs_leg_initial_again.set_end_point(
            {initial_again_end_point_[0], initial_again_end_point_[1], initial_again_end_point_[2],
             initial_again_end_point_[3], 0, 0});

        return *this;
    }

private:
    std::array<double, 6> result;
    hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT> up_stairs_initial;
    hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT> up_stairs_leg_press;
    hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT> up_stairs_leg_lift;
    hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT>
        up_stairs_leg_press_and_lift;
    hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT>
        up_stairs_leg_initial_again;
    UpStairsContext context_;
    HSM<UpStairsState, UpStairsEventId, UpStairsContext> up_stairs_hsm;
};

} // namespace rmcs_core::hardware::hsm::up_stairs_hsm

#endif // HSM_UP_STAIRS_HPP_H_