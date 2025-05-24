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

enum class Auto_Storage_State {
    Set_Storage_Lb,
};
enum class Auto_Storage_Event { Up, Down };
struct Auto_Storage_Context {};

class Set_Storage_Lb_State
    : public IState<Auto_Storage_State, Auto_Storage_Event, Auto_Storage_Context> {
public:
    void enter(
        FiniteStateMachine<Auto_Storage_State, Auto_Storage_Event, Auto_Storage_Context>&
            fsm,
        const Auto_Storage_Context& context) override {}
    void exit(
        FiniteStateMachine<Auto_Storage_State, Auto_Storage_Event, Auto_Storage_Context>&
            fsm,
        const Auto_Storage_Context& context) override {}
    std::shared_ptr<IState<Auto_Storage_State, Auto_Storage_Event, Auto_Storage_Context>>
        handleEvent(
            FiniteStateMachine<
                Auto_Storage_State, Auto_Storage_Event, Auto_Storage_Context>& fsm,
            const Auto_Storage_Event& event, Auto_Storage_Context& context) override {

        if (event == Auto_Storage_Event::Up) {
            return fsm.getState(Auto_Storage_State::Set_Storage_Lb);
        }
        return nullptr;
    }
    Auto_Storage_State getStateID() const override {
        return Auto_Storage_State::Set_Storage_Lb;
    }
};

class Auto_Storage_Lb {
public:
    explicit Auto_Storage_Lb() {
        std::array<double, 3> press_start_point_position = {-0.23, 0.26, 0.33};
        std::array<double, 3> press_end_point_position   = {-0.23, 0.26, 0.3};
        std::array<double, 3> press_point_orientation    = {
            -41 * std::numbers::pi / 180.0, 0.0, 0.0};

        std::array<double, 6> move_to_target_ =
            rmcs_core::hardware::device::Kinematic::arm_inverse_kinematic(
                {press_start_point_position[0], press_start_point_position[1],
                 press_start_point_position[2], press_point_orientation[0],
                 press_point_orientation[1], press_point_orientation[2]});
        move_to_target.set_total_step(1500).set_end_point(move_to_target_);
        press.set_total_step(300)
            .set_start_point(press_start_point_position, press_point_orientation)
            .set_end_point(press_end_point_position, press_point_orientation);

        fsm.registerState<Set_Storage_Lb_State>();
        fsm.addTransition<Auto_Storage_Event>(
            Auto_Storage_State::Set_Storage_Lb, Auto_Storage_Event::Up,
            [this](const Auto_Storage_Event& event, const Auto_Storage_Context& context) {
                if (event == Auto_Storage_Event::Up) {
                    move_to_target.set_start_point(last_theta_);
                    if (!move_to_target.get_complete()) {
                        result = move_to_target.trajectory();
                    } else {

                        result = rmcs_core::hardware::device::Kinematic::arm_inverse_kinematic(
                            {press.trajectory()});
                    }
                }
                return false;
            },
            [this](const Auto_Storage_Event& event, Auto_Storage_Context& context) {},
            Auto_Storage_State::Set_Storage_Lb);
        fsm.start(Auto_Storage_State::Set_Storage_Lb);
    }
    void get_current_theta(std::array<double, 6> theta) { last_theta_ = theta; }
    std::array<double, 6> get_result() { return result; }
    auto getState() { return fsm.getCurrentState(); }
    void processEvent(const Auto_Storage_Event& event) { fsm.processEvent(event); }
    bool get_first_trajectory_result(){return move_to_target.get_complete();}

    int fsm_direction = initial_enter;
    void reset() {
        fsm.start(Auto_Storage_State::Set_Storage_Lb);
        move_to_target.reset();
        press.reset();
        fsm_direction = initial_enter;
    }

private:
    Auto_Storage_Context context_;
    FiniteStateMachine<Auto_Storage_State, Auto_Storage_Event, Auto_Storage_Context> fsm{
        context_};
    rmcs_core::hardware::device::Trajectory<rmcs_core::hardware::device::TrajectoryType::JOINT>
        move_to_target;
    rmcs_core::hardware::device::Trajectory<rmcs_core::hardware::device::TrajectoryType::LINE>
        press;
    std::array<double, 6> last_theta_, result;
};