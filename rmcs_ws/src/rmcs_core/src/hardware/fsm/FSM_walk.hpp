#pragma once

#include "hardware/device/Kinematic.hpp"
#include "hardware/device/trajectory.hpp"
#include "hardware/fsm/FSM.hpp"
#include "rclcpp/node.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <memory>
#include <numbers>
#include <rclcpp/logging.hpp>
#include <string>

enum class Auto_Walk_State {
    Set_Walk_Arm,
};
enum class Auto_Walk_Event { Up, Down };
struct Auto_Walk_Context {};

class Set_Walk_Arm_State : public IState<Auto_Walk_State, Auto_Walk_Event, Auto_Walk_Context> {
public:
    void enter(
        FiniteStateMachine<Auto_Walk_State, Auto_Walk_Event, Auto_Walk_Context>& fsm,
        const Auto_Walk_Context& context) override {}
    void exit(
        FiniteStateMachine<Auto_Walk_State, Auto_Walk_Event, Auto_Walk_Context>& fsm,
        const Auto_Walk_Context& context) override {}
    std::shared_ptr<IState<Auto_Walk_State, Auto_Walk_Event, Auto_Walk_Context>> handleEvent(
        FiniteStateMachine<Auto_Walk_State, Auto_Walk_Event, Auto_Walk_Context>& fsm,
        const Auto_Walk_Event& event, Auto_Walk_Context& context) override {

        if (event == Auto_Walk_Event::Up) {
            return fsm.getState(Auto_Walk_State::Set_Walk_Arm);
        }
        return nullptr;
    }
    Auto_Walk_State getStateID() const override { return Auto_Walk_State::Set_Walk_Arm; }
};

class Auto_Set_Walk_Arm {
public:
    explicit Auto_Set_Walk_Arm() {
        fsm.registerState<Set_Walk_Arm_State>();
        fsm.addTransition<Auto_Walk_Event>(
            Auto_Walk_State::Set_Walk_Arm, Auto_Walk_Event::Up,
            [this](const Auto_Walk_Event& event, const Auto_Walk_Context& context) {
                if (event == Auto_Walk_Event::Up) {
                    set_walk_arm.set_start_point(enter_theta_);
                    result = set_walk_arm.trajectory();
                }
                return false;
            },
            [this](const Auto_Walk_Event& event, Auto_Walk_Context& context) {},
            Auto_Walk_State::Set_Walk_Arm);
        fsm.start(Auto_Walk_State::Set_Walk_Arm);

        set_walk_arm.set_total_step(2000).set_end_point({0, 1.04719, -1.0472, 0, -0.9, 0});
    }
    void get_current_theta(std::array<double, 6> theta) { enter_theta_ = theta; }
    std::array<double, 6> get_result() { return result; }
    auto getState() { return fsm.getCurrentState(); }
        void processEvent(const Auto_Walk_Event& event) { fsm.processEvent(event); }

    int fsm_direction = initial_enter;
    void reset() {
        fsm.start(Auto_Walk_State::Set_Walk_Arm);
        set_walk_arm.reset();
        fsm_direction = initial_enter;
    }

private:
    Auto_Walk_Context context_;
    FiniteStateMachine<Auto_Walk_State, Auto_Walk_Event, Auto_Walk_Context> fsm{context_};
    rmcs_core::hardware::device::Trajectory<rmcs_core::hardware::device::TrajectoryType::JOINT>
        set_walk_arm;
    std::array<double, 6> enter_theta_, result;
};