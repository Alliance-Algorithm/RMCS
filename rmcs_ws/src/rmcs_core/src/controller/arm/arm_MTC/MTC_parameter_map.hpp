#pragma once

#include "controller/arm/arm_MTC/MTC_dictionary.hpp"
#include "controller/arm/arm_MTC/MTC_step.hpp"

#include <numbers>
#include <string>
#include <unordered_map>

namespace rmcs_core::controller::arm::MTC {

inline auto make_mtc_parameter_map() {
    return Dictionary::
        Entries{
            {"current_start",
             {Step::makeCurrentState("current_start", CurrentStateConfig{.timeout = 3.0})}},
            {"tech_core_test",
             {ContainerConfig{
                 .name = "tech_core_test",
                 .type = ContainerType::Serial,
                 .children = {Step::makeMoveTo(
                     "tech_core_test_pre_place",
                     MoveToConfig{
                         .goal = makePoseGoal(
                             PoseTarget{
                                 .frame_id = "base_link",
                                 .x = 0.246,
                                 .y = -0.316,
                                 .z = 0.353,
                                 .roll = -2.233,
                                 .pitch = -0.313,
                                 .yaw = 1.635}),
                         .group = "alliance_arm",
                         .ik_frame = "link_6"})}}}},
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
            {"extra",
             {ContainerConfig{
                 .name = "extra",
                 .type = ContainerType::Serial,
                 .children =
                     {
                         Step::makeOpenGripper("open_before_extra"),
                         Step::makeAllowCollisions("unit"),
                         Step::makeAllowCollisions("gimbal"),
                         Step::makeAllowObjectCollisions("unit", "gimbal"),
                         Step::makeMoveRelative(
                             "approach",
                             MoveRelativeConfig{
                                 .direction = makeDirection(
                                     DirectionTarget{
                                         .frame_id = "link_6",
                                         .dir_x = 0.0,
                                         .dir_y = 0.0,
                                         .dir_z = -1.0}),
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
                         Step::makeAttachObject("unit"),
                         Step::makeMoveRelative(
                             "lift",
                             MoveRelativeConfig{
                                 .direction = makeDirection(
                                     DirectionTarget{
                                         .frame_id = "link_6",
                                         .dir_x = 1.0,
                                         .dir_y = 0.0,
                                         .dir_z = 0.0}),
                                 .min_distance = 0.15,
                                 .max_distance = 0.15,
                                 .motion =
                                     {.solver = CartesianPathConfig{.step_size = 0.005},
                                      .velocity_scaling = 0.3,
                                      .acceleration_scaling = 0.1,
                                      .timeout = 3.0},
                                 .group = "alliance_arm",
                                 .ik_frame = "link_6"}),
                         Step::makeForbidObjectCollisions("unit", "gimbal"),
                         Step::makeForbidCollisions("gimbal"),
                     }}}},
            {"store",
             {ContainerConfig{
                 .name = "store",
                 .type = ContainerType::Serial,
                 .children =
                     {
                         Step::makeMoveRelative(
                             "lower",
                             MoveRelativeConfig{
                                 .direction = makeDirection(
                                     DirectionTarget{
                                         .frame_id = "link_6",
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
                         Step::makeOpenGripper("release_gripper"),
                         Step::makeDetachObject("unit"),
                         Step::makeMoveRelative(
                             "retreat",
                             MoveRelativeConfig{
                                 .direction = makeDirection(
                                     DirectionTarget{
                                         .frame_id = "link_6",
                                         .dir_x = 0.0,
                                         .dir_y = 0.0,
                                         .dir_z = 1.0}),
                                 .min_distance = 0.1,
                                 .max_distance = 0.1,
                                 .motion =
                                     {.solver = CartesianPathConfig{.step_size = 0.005},
                                      .velocity_scaling = 0.3,
                                      .acceleration_scaling = 0.1,
                                      .timeout =
                                          3.0},
                                 .group = "alliance_arm",
                                 .ik_frame = "link_6"}),
                         Step::makeForbidCollisions("unit"),
                     }}}},
            {"tech_core_place",
             {ContainerConfig{
                 .name = "tech_core_place",
                 .type = ContainerType::Serial,
                 .children =
                     {
                         Step::makeAllowObjectCollisions("unit", "pillar"),
                         Step::makeAllowCollisions("pillar"),
                         Step::makeMoveRelative(
                             "tech_core_place_lower",
                             MoveRelativeConfig{
                                 .direction =
                                     makeDirection(
                                         DirectionTarget{
                                             .frame_id = "link_6",
                                             .dir_x = -1.0,
                                             .dir_y = 0.0,
                                             .dir_z = 0.0}),
                                 .min_distance = 0.11,
                                 .max_distance = 0.11,
                                 .motion =
                                     {.solver = CartesianPathConfig{.step_size = 0.001},
                                      .velocity_scaling = 0.3,
                                      .acceleration_scaling =
                                          0.1,
                                      .timeout = 3.0},
                                 .group = "alliance_arm",
                                 .ik_frame = "link_6"}),
                         Step::makeDetachObject("unit"),
                     }}}},
            {"push",
             {ContainerConfig{
                 .name = "push",
                 .type = ContainerType::Serial,
                 .children =
                     {
                         Step::makeAllowCollisions("unit"),
                         Step::makeAllowCollisions("pillar"),
                         Step::makeAllowObjectCollisions("unit", "pillar"),
                         Step::makeAllowObjectCollisions("exchange", "pillar"),
                         Step::makeAttachObject("unit"),
                         Step::makeAttachObject("pillar"),
                         Step::makeMoveRelative(
                             "tech_core_push",
                             MoveRelativeConfig{
                                 .direction =
                                     makeDirection(
                                         DirectionTarget{
                                             .frame_id = "link_6",
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
                         Step::makeDetachObject("pillar"),
                         Step::makeDetachObject("unit"),
                     }}}},

            {"rotate_P",
             {ContainerConfig{
                 .name = "rotate_P",
                 .type = ContainerType::Serial,
                 .children =
                     {
                         Step::makeAllowCollisions("unit"),
                         Step::makeAllowCollisions("pillar"),
                         Step::makeAllowCollisions("exchange"),
                         Step::makeAllowObjectCollisions("unit", "pillar"),
                         Step::makeAllowObjectCollisions("exchange", "pillar"),
                         Step::makeAllowObjectCollisions("exchange", "unit"),
                         Step::makeAttachObject("unit"),
                         Step::makeAttachObject("pillar"),
                         Step::makeMoveRelative(
                             "tech_core_rotate_P",
                             MoveRelativeConfig{
                                 .direction =
                                     makeRotationDirection(
                                         {.frame_id = "base_link",
                                          .axis_point = {0.4254, -0.5498, 0.4083},
                                          .axis_direction =
                                              {-0.5980, -0.2818, 0.7503}}),
                                 .min_distance = -std::numbers::pi / 4 * 0,
                                 .max_distance = -std::numbers::pi / 4 * 2,
                                 .motion =
                                     {.solver = CartesianPathConfig{.step_size = 0.001},
                                      .velocity_scaling = 0.3,
                                      .acceleration_scaling = 0.1,
                                      .timeout = 100.0},
                                 .group = "alliance_arm",
                                 .ik_frame = "link_6"}),
                         Step::makeDetachObject("pillar"),
                         Step::makeDetachObject("unit"),
                     }}}},
            {"rotate_Q",
             {ContainerConfig{
                 .name = "rotate_Q",
                 .type = ContainerType::Serial,
                 .children =
                     {
                         Step::makeAllowCollisions("unit"),
                         Step::makeAllowCollisions("pillar"),
                         Step::makeAllowCollisions("exchange"),
                         Step::makeAllowObjectCollisions("unit", "pillar"),
                         Step::makeAllowObjectCollisions("exchange", "pillar"),
                         Step::makeAllowObjectCollisions("exchange", "unit"),
                         Step::makeAttachObject("unit"),
                         Step::makeAttachObject("pillar"),
                         Step::makeMoveRelative(
                             "tech_core_rotate_Q",
                             MoveRelativeConfig{
                                 .direction =
                                     makeRotationDirection(
                                         {.frame_id = "base_link",
                                          .axis_point = {0.4374, -0.5519, 0.4171},
                                          .axis_direction =
                                              {-0.0610, 0.9495, 0.3079}}),
                                 .min_distance = 0,
                                 .max_distance = std::numbers::pi / 4,
                                 .motion =
                                     {.solver = CartesianPathConfig{.step_size = 0.001},
                                      .velocity_scaling = 0.3,
                                      .acceleration_scaling = 0.1,
                                      .timeout = 100.0},
                                 .group = "alliance_arm",
                                 .ik_frame = "link_6"}),
                         Step::makeDetachObject("pillar"),
                         Step::makeDetachObject("unit"),
                     }}}},

        };
}

inline auto make_default_dictionary() { return Dictionary(make_mtc_parameter_map()); }

} // namespace rmcs_core::controller::arm::MTC
