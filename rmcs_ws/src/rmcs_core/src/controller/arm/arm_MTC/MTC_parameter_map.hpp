#pragma once

#include "controller/arm/arm_MTC/MTC_dictionary.hpp"
#include "controller/arm/arm_MTC/MTC_step.hpp"

#include <string>
#include <unordered_map>

namespace rmcs_core::controller::arm::MTC {

inline auto make_mtc_parameter_map() {
    return Dictionary::Entries{
        {"current_start",
         {Step::makeCurrentState("current_start", CurrentStateConfig{.timeout = 3.0})}},
        {"extra_left_front",
         {ContainerConfig{
             .name = "extra_left_front",
             .type = ContainerType::Serial,
             .children = {Step::makeMoveTo(
                 "extra_left_front_pre_grasp",
                 MoveToConfig{
                     .goal = makePoseGoal(
                         PoseTarget{
                             .frame_id = "base_link",
                             .x = -0.142,
                             .y = 0.128,
                             .z = 0.165,
                             .roll = 0.001,
                             .pitch = -1.550,
                             .yaw = 3.116}),
                     .group = "alliance_arm",
                     .ik_frame = "link_6"})}}}},
        {"store_left_front",
         {ContainerConfig{
             .name = "store_left_front",
             .type = ContainerType::Serial,
             .children = {Step::makeMoveTo(
                 "store_left_front_pre_place",
                 MoveToConfig{
                     .goal = makePoseGoal(
                         PoseTarget{
                             .frame_id = "base_link",
                             .x = 0.520,
                             .y = 0.0,
                             .z = 0.471,
                             .roll = -0.785,
                             .pitch = -1.571,
                             .yaw = 0.785}),
                     .group = "alliance_arm",
                     .ik_frame = "link_6"})}}}},
        {"pick",
         {ContainerConfig{
             .name = "pick",
             .type = ContainerType::Serial,
             .children =
                 {Step::makeOpenGripper("open_before_pick"),
                  Step::makeAllowCollisions("energy_unit_left_front"),
                  Step::makeMoveRelative(
                      "approach",
                      MoveRelativeConfig{
                          .direction = makeDirection(
                              DirectionTarget{
                                  .frame_id = "base_link",
                                  .dir_x = -1.0,
                                  .dir_y = 0.0,
                                  .dir_z = 0.0}),
                          .min_distance = 0.09,
                          .max_distance = 0.09,
                          .motion =
                              {.solver = CartesianPathConfig{.step_size = 0.005},
                               .velocity_scaling = 0.3,
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
                               .velocity_scaling = 0.3,
                               .acceleration_scaling = 0.1,
                               .timeout = 3.0},
                          .group = "alliance_arm",
                          .ik_frame = "link_6"})}}}},
        {"place",
         {ContainerConfig{
             .name = "place",
             .type = ContainerType::Serial,
             .children = {
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
                              .velocity_scaling = 0.3,
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
                                 .dir_x = -1.0,
                                 .dir_y = 0.0,
                                 .dir_z = 0.0}),
                         .min_distance = 0.1,
                         .max_distance = 0.1,
                         .motion =
                             {.solver = CartesianPathConfig{.step_size = 0.005},
                              .velocity_scaling = 0.3,
                              .acceleration_scaling = 0.1,
                              .timeout = 3.0},
                         .group = "alliance_arm",
                         .ik_frame = "link_6"}),
                 Step::makeForbidCollisions("energy_unit_left_front"),
             }}}}};
}

inline auto make_default_dictionary() { return Dictionary(make_mtc_parameter_map()); }

} // namespace rmcs_core::controller::arm::MTC
