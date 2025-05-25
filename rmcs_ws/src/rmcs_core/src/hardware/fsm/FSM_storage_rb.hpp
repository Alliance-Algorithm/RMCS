#pragma once
#include "hardware/fsm/FSM_storage_lb.hpp"
class Auto_Storage_RB {
public:
    explicit Auto_Storage_RB() {
        move_to_former_target.set_total_step(600);
        move_to_target.set_total_step(900);
        press.set_total_step(300)
            .set_start_point(press_start_point_position, press_point_orientation)
            .set_end_point(press_end_point_position, press_point_orientation);

        fsm.registerState<Set_Storage_State>();
        fsm.addTransition<Auto_Storage_Event>(
            Auto_Storage_State::Set_Storage, Auto_Storage_Event::Up,
            [this](const Auto_Storage_Event& event, const Auto_Storage_Context& context) {
                if (event == Auto_Storage_Event::Up) {
                    move_to_former_target.set_start_point(last_theta_);
                    move_to_former_target.set_end_point(
                        {last_theta_[0], move_to_target_[1], move_to_target_[2], move_to_target_[3],
                         move_to_target_[4], move_to_target_[5]});
                    move_to_target
                        .set_start_point(
                            {last_theta_[0], move_to_target_[1], move_to_target_[2],
                             move_to_target_[3], move_to_target_[4], move_to_target_[5]})
                        .set_end_point(move_to_target_);
                    if (!move_to_former_target.get_complete()) {
                        result = move_to_former_target.trajectory();
                    } else {
                        if (!move_to_target.get_complete()) {
                            result = move_to_target.trajectory();
                        } else {
                            if (!press.get_complete()) {
                                result =
                                    rmcs_core::hardware::device::Kinematic::arm_inverse_kinematic(
                                        {press.trajectory()});
                            } else {
                                if (!lift.get_complete()) {
                                    result = rmcs_core::hardware::device::Kinematic::
                                        arm_inverse_kinematic(lift.trajectory());
                                } else {
                                    result = back_to_safety_.trajectory();
                                }
                            }
                        }
                    }
                }
                return false;
            },
            [this](const Auto_Storage_Event& event, Auto_Storage_Context& context) {},
            Auto_Storage_State::Set_Storage);
        fsm.start(Auto_Storage_State::Set_Storage);
    }
    void get_current_theta(std::array<double, 6> theta) { last_theta_ = theta; }
    std::array<double, 6> get_result() { return result; }
    auto getState() { return fsm.getCurrentState(); }
    void processEvent(const Auto_Storage_Event& event) { fsm.processEvent(event); }
    bool get_first_trajectory_result() { return move_to_target.get_complete(); }

    int fsm_direction = initial_enter;
    void reset() {
        fsm.start(Auto_Storage_State::Set_Storage);
        move_to_target.reset();
        press.reset();
        move_to_former_target.reset();
        lift.reset();
        back_to_safety_.reset();
        fsm_direction = initial_enter;
    }

private:
    Auto_Storage_Context context_;
    FiniteStateMachine<Auto_Storage_State, Auto_Storage_Event, Auto_Storage_Context> fsm{context_};
    rmcs_core::hardware::device::Trajectory<rmcs_core::hardware::device::TrajectoryType::JOINT>
        move_to_target;
    rmcs_core::hardware::device::Trajectory<rmcs_core::hardware::device::TrajectoryType::JOINT>
        move_to_former_target;
    rmcs_core::hardware::device::Trajectory<rmcs_core::hardware::device::TrajectoryType::LINE>
        press;
        rmcs_core::hardware::device::Trajectory<rmcs_core::hardware::device::TrajectoryType::LINE> lift;
    rmcs_core::hardware::device::Trajectory<rmcs_core::hardware::device::TrajectoryType::JOINT>
        back_to_safety_;
    std::array<double, 6> last_theta_, result;
    std::array<double, 3> press_start_point_position = {-0.27, -0.22, 0.35};
    std::array<double, 3> press_end_point_position   = {-0.27, -0.22, 0.3};
    std::array<double, 3> lift_start_point_position  = {-0.27, -0.22, 0.3};
    std::array<double, 3> lift_end_point_position    = {-0.27, -0.22, 0.35};
    std::array<double, 3> press_point_orientation    = {-41 * std::numbers::pi / 180.0, 0.0, 0.0};

    std::array<double, 6> move_to_target_ =
        rmcs_core::hardware::device::Kinematic::arm_inverse_kinematic(
            {press_start_point_position[0], press_start_point_position[1],
             press_start_point_position[2], press_point_orientation[0], press_point_orientation[1],
             press_point_orientation[2]});
    std::array<double, 6> back_to_safety =
        rmcs_core::hardware::device::Kinematic::arm_inverse_kinematic(
            {lift_end_point_position[0], lift_end_point_position[1], lift_end_point_position[2],
             press_point_orientation[0], press_point_orientation[1], press_point_orientation[2]});
};