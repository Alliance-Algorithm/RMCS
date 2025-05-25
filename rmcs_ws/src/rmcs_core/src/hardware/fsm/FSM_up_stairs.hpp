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
enum class Auto_Up_Stairs_State { Leg_Initial, Leg_Lift };
enum class Auto_Up_Stairs_Event { Up, Down };
struct Auto_Up_Stairs_Context {};
class Leg_Initial_State
    : public IState<Auto_Up_Stairs_State, Auto_Up_Stairs_Event, Auto_Up_Stairs_Context> {
public:
    void enter(
        FiniteStateMachine<Auto_Up_Stairs_State, Auto_Up_Stairs_Event, Auto_Up_Stairs_Context>& fsm,
        const Auto_Up_Stairs_Context& context) override {
        // RCLCPP_INFO(logger, "bbbb");
    }
    void exit(
        FiniteStateMachine<Auto_Up_Stairs_State, Auto_Up_Stairs_Event, Auto_Up_Stairs_Context>& fsm,
        const Auto_Up_Stairs_Context& context) override {
        // RCLCPP_INFO(logger, "cccc");
    }
    std::shared_ptr<IState<Auto_Up_Stairs_State, Auto_Up_Stairs_Event, Auto_Up_Stairs_Context>>
        handleEvent(
            FiniteStateMachine<Auto_Up_Stairs_State, Auto_Up_Stairs_Event, Auto_Up_Stairs_Context>&
                fsm,
            const Auto_Up_Stairs_Event& event, Auto_Up_Stairs_Context& context) override {
        if (event == Auto_Up_Stairs_Event::Up) {
            return fsm.getState(Auto_Up_Stairs_State::Leg_Lift);
        }
        return nullptr;
    }
    Auto_Up_Stairs_State getStateID() const override { return Auto_Up_Stairs_State::Leg_Initial; }

private:
    rclcpp::Logger logger = rclcpp::get_logger("Set_initial_State");
};
class Leg_Press_State
    : public IState<Auto_Up_Stairs_State, Auto_Up_Stairs_Event, Auto_Up_Stairs_Context> {
public:
    void enter(
        FiniteStateMachine<Auto_Up_Stairs_State, Auto_Up_Stairs_Event, Auto_Up_Stairs_Context>& fsm,
        const Auto_Up_Stairs_Context& context) override {}
    void exit(
        FiniteStateMachine<Auto_Up_Stairs_State, Auto_Up_Stairs_Event, Auto_Up_Stairs_Context>& fsm,
        const Auto_Up_Stairs_Context& context) override {}
    std::shared_ptr<IState<Auto_Up_Stairs_State, Auto_Up_Stairs_Event, Auto_Up_Stairs_Context>>
        handleEvent(
            FiniteStateMachine<Auto_Up_Stairs_State, Auto_Up_Stairs_Event, Auto_Up_Stairs_Context>&
                fsm,
            const Auto_Up_Stairs_Event& event, Auto_Up_Stairs_Context& context) override {

        if (event == Auto_Up_Stairs_Event::Down) {
            return fsm.getState(Auto_Up_Stairs_State::Leg_Initial);
        }
        return nullptr;
    }
    Auto_Up_Stairs_State getStateID() const override { return Auto_Up_Stairs_State::Leg_Lift; }

private:
    rclcpp::Logger logger = rclcpp::get_logger("Lift_State");
};
class Auto_Up_Stairs {
public:
    explicit Auto_Up_Stairs() {
        lift.set_total_step(1600).set_end_point({0.0, -0.4, 0.2, 0.0, 0.0, 0.0});
        initial.set_total_step(2000).set_end_point({0.0, -0.9, 0.5, 0.0, 0.0, 0.0});
        fsm.registerState<Leg_Initial_State>();
        fsm.registerState<Leg_Press_State>();
        fsm.start(Auto_Up_Stairs_State::Leg_Initial);

        fsm.addTransition<Auto_Up_Stairs_Event>(
            Auto_Up_Stairs_State::Leg_Initial, Auto_Up_Stairs_Event::Up,
            [this](const Auto_Up_Stairs_Event& event, const Auto_Up_Stairs_Context& context) {
                if (event == Auto_Up_Stairs_Event::Up) {
                    if (initial.get_complete()) {
                        return true;
                    }
                    initial.set_start_point(last_theta_);
                    result = initial.trajectory();
                }
                return false;
            },
            [this](const Auto_Up_Stairs_Event& event, Auto_Up_Stairs_Context& context) {},
            Auto_Up_Stairs_State::Leg_Lift);

        fsm.addTransition<Auto_Up_Stairs_Event>(
            Auto_Up_Stairs_State::Leg_Lift, Auto_Up_Stairs_Event::Up,
            [this](const Auto_Up_Stairs_Event& event, const Auto_Up_Stairs_Context& context) {
                if (event == Auto_Up_Stairs_Event::Up) {
                    lift.set_start_point(last_theta_);
                    result = lift.trajectory();
                }

                return false;
            },
            [this](const Auto_Up_Stairs_Event& event, Auto_Up_Stairs_Context& context) {},
            Auto_Up_Stairs_State::Leg_Initial);
    }

    void reset() {
        fsm.start(Auto_Up_Stairs_State::Leg_Initial);
        fsm_direction = initial_enter;
        lift.reset();
        initial.reset();
    }
    void get_current_theta(std::array<double, 6> theta) { last_theta_ = theta; }
    Auto_Up_Stairs_State getState() { return fsm.getCurrentState(); }
    std::array<double, 6> get_result() { return result; }
    bool get_first_trajectory_result() { return initial.get_complete(); }
    bool get_second_trajectory_result() { return lift.get_complete(); }
    int fsm_direction = initial_enter;
    void processEvent(const Auto_Up_Stairs_Event& event) { fsm.processEvent(event); }

private:
    std::array<double, 6> last_theta_, result;
    Auto_Up_Stairs_Context context_;

    FiniteStateMachine<Auto_Up_Stairs_State, Auto_Up_Stairs_Event, Auto_Up_Stairs_Context> fsm{
        context_};

    rmcs_core::hardware::device::Trajectory<rmcs_core::hardware::device::TrajectoryType::JOINT>
        initial;
    rmcs_core::hardware::device::Trajectory<rmcs_core::hardware::device::TrajectoryType::JOINT>
        lift;
};