#pragma once

#include "controller/arm/arm_MTC/MTC_dictionary.hpp"
#include "controller/arm/arm_MTC/MTC_step.hpp"

#include <string>
#include <unordered_map>

namespace rmcs_core::controller::arm::MTC {

inline auto make_mtc_parameter_map() {
    using ParameterMap = std::unordered_map<std::string, TaskConfig>;
    return ParameterMap{
        {"pick_place_example",
         TaskConfig{
             .name = "pick_place_example",
             .children =
                 {Step::makeCurrentState("current_start", CurrentStateConfig{.timeout = 3.0}),
                  ContainerConfig{
                      .name = "pick",
                      .type = ContainerType::Serial,
                      .children =
                          {Step::makeOpenGripper("open_before_pick"),
                           Step::makeMoveTo(
                               "move_to_pregrasp",
                               MoveToConfig{
                                   .goal = makePoseGoal(
                                       PoseTarget{
                                           .frame_id = "base_link",
                                           .x = 0.495,
                                           .y = -0.045,
                                           .z = 0.587,
                                           .roll = -0.843,
                                           .pitch = -1.571,
                                           .yaw = 0.843}),
                                   .group = "alliance_arm",
                                   .ik_frame = "link_6"}),
                           Step::makeAllowCollisions("energy_unit_left_front"),
                           Step::makeMoveRelative(
                               "approach",
                               MoveRelativeConfig{
                                   .direction = makeDirection(
                                       DirectionTarget{
                                           .frame_id = "base_link",
                                           .dir_x = 0.0,
                                           .dir_y = 0.0,
                                           .dir_z = -1.0}),
                                   .min_distance = 0.1,
                                   .max_distance = 0.1,
                                   .motion =
                                       {.solver = CartesianPathConfig{.step_size = 0.005},
                                        .velocity_scaling = 0.1,
                                        .acceleration_scaling = 0.1,
                                        .timeout = 3.0},
                                   .group = "alliance_arm",
                                   .ik_frame = "link_6"}),
                           Step::makeCloseGripper("close_gripper"),
                           Step::makeAttachObject("energy_unit_left_front"),
                           Step::makeMoveRelative(
                               "lift",
                               MoveRelativeConfig{
                                   .direction = makeDirection(
                                       DirectionTarget{
                                           .frame_id = "base_link",
                                           .dir_x = 0.0,
                                           .dir_y = 0.0,
                                           .dir_z = 1.0}),
                                   .min_distance = 0.1,
                                   .max_distance = 0.1,
                                   .motion =
                                       {.solver = CartesianPathConfig{.step_size = 0.005},
                                        .velocity_scaling = 0.1,
                                        .acceleration_scaling = 0.1,
                                        .timeout =
                                            3.0},
                                   .group = "alliance_arm",
                                   .ik_frame =
                                       "link_6"})}},
                  ContainerConfig{
                      .name = "place",
                      .type = ContainerType::Serial,
                      .children =
                          {Step::makeMoveTo(
                               "move_to_preplace",
                               MoveToConfig{
                                   .goal = makePoseGoal(
                                       PoseTarget{
                                           .frame_id = "base_link",
                                           .x = 0.473,
                                           .y = 0.375,
                                           .z = 0.271,
                                           .roll = 0.736,
                                           .pitch = -1.571,
                                           .yaw = 0.128}),
                                   .motion =
                                       {.solver = PipelineConfig{"ompl", "BiTRRTkConfigDefault"},
                                        .velocity_scaling = 0.1,
                                        .acceleration_scaling = 0.1},
                                   .group = "alliance_arm",
                                   .ik_frame = "link_6"}),
                           Step::makeMoveRelative(
                               "lower",
                               MoveRelativeConfig{
                                   .direction = makeDirection(
                                       DirectionTarget{
                                           .frame_id = "base_link",
                                           .dir_x = 0.0,
                                           .dir_y = 0.0,
                                           .dir_z = -1.0}),
                                   .min_distance = 0.1,
                                   .max_distance = 0.1,
                                   .motion =
                                       {.solver = CartesianPathConfig{.step_size = 0.005},
                                        .velocity_scaling = 0.1,
                                        .acceleration_scaling = 0.1,
                                        .timeout = 3.0},
                                   .group = "alliance_arm",
                                   .ik_frame = "link_6"}),
                           Step::makeOpenGripper("release_gripper"),
                           Step::makeDetachObject("energy_unit_left_front"),
                           Step::makeMoveRelative(
                               "retreat",
                               MoveRelativeConfig{
                                   .direction = makeDirection(
                                       DirectionTarget{
                                           .frame_id = "base_link",
                                           .dir_x = 0.0,
                                           .dir_y = 0.0,
                                           .dir_z = 1.0}),
                                   .min_distance = 0.1,
                                   .max_distance = 0.1,
                                   .motion =
                                       {.solver = CartesianPathConfig{.step_size = 0.005},
                                        .velocity_scaling = 0.1,
                                        .acceleration_scaling = 0.1,
                                        .timeout = 3.0},
                                   .group = "alliance_arm",
                                   .ik_frame = "link_6"}),
                           Step::makeForbidCollisions("energy_unit_left_front")}}}}}};
}

inline auto make_default_dictionary() { return Dictionary(make_mtc_parameter_map()); }

} // namespace rmcs_core::controller::arm::MTC
