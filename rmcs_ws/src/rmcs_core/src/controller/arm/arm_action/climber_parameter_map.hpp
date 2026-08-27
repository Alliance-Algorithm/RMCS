#pragma once
#include "controller/arm/arm_action/action_step.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace rmcs_core::controller::arm {

inline auto make_climber_action_parameter_map() {
    using ParameterMap = std::unordered_map<std::string, std::vector<Action::Step>>;
    return ParameterMap{
        // ---------- 简单动作 ----------
        {                  "gripper_open",{Action::Step::makeOpenGripper()}                                          },

        {                 "gripper_close",                   {Action::Step::makeCloseGripper()}},
        {                         "delay",                          {Action::Step::makeDelay()}},
        {                          "test",                    {Action::Step::makeOpenGripper()}},
        {                     "auto_walk",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001})}           },
        {        "crash_wall_calibration",
         {Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.015,
         .y     = 0.001,
         .z     = 0.449,
         .roll  = 2.524,
         .pitch = -1.335,
         .yaw   = -2.521},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.02})}            },

        { "roll_out_in_three_mines_first",
         {Action::Step::makeOpenGripper(),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 0.465947,
         .joint_3 = -0.094436,
         .joint_4 = 0.0,
         .joint_5 = -0.293182,
         .joint_6 = -0.000000},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.0007, .tolerance_ori = 0.0007})}         },
        {"roll_out_in_three_mines_second",
         {Action::Step::makeOpenGripper(),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.0,
         .joint_2 = 0.532291,
         .joint_3 = -0.798916,
         .joint_4 = 0.0,
         .joint_5 = 0.356459,
         .joint_6 = 3.099504},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001})}           },
        {         "up_one_stairs_initial",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001})}           },

        {         "up_two_stairs_initial",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001})}           },

        {   "up_two_stairs_initial_again",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001})}           },

        {      "up_two_stairs_lift_again",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001})}           },

        // ---------- 组合动作 ----------
        {                    "extract_lf",
         {
         Action::Step::makeOpenGripper(),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.000096,
         .joint_2 = 0.219647,
         .joint_3 = -1.368023,
         .joint_4 = -0.006136,
         .joint_5 = 1.347602,
         .joint_6 = -0.044773},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 1.957935,
         .joint_2 = 0.218592,
         .joint_3 = -1.365339,
         .joint_4 = -0.001917,
         .joint_5 = 1.350478,
         .joint_6 = -0.044965},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 2.651773,
         .joint_2 = 0.443033,
         .joint_3 = -1.378857,
         .joint_4 = 0.001150,
         .joint_5 = 0.965545,
         .joint_6 = -0.066824},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.005}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.105},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.005}),
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.15},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.172,
         .y     = 0.084,
         .z     = 0.293,
         .roll  = 3.059,
         .pitch = -0.697,
         .yaw   = -0.352},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.005, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.000096,
         .joint_2 = 0.219647,
         .joint_3 = -1.368023,
         .joint_4 = -0.006136,
         .joint_5 = 1.347602,
         .joint_6 = -0.044773},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),

         }                                                                                     },
        {                    "extract_lb",
         {
         Action::Step::makeOpenGripper(),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.000096,
         .joint_2 = 0.219647,
         .joint_3 = -1.368023,
         .joint_4 = -0.006136,
         .joint_5 = 1.347602,
         .joint_6 = -0.044773},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 1.957935,
         .joint_2 = 0.218592,
         .joint_3 = -1.365339,
         .joint_4 = -0.001917,
         .joint_5 = 1.350478,
         .joint_6 = -0.044965},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 2.651773,
         .joint_2 = 0.443033,
         .joint_3 = -1.378857,
         .joint_4 = 0.001150,
         .joint_5 = 0.965545,
         .joint_6 = -0.066824},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.005}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.105},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.005}),
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.15},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.172,
         .y     = 0.084,
         .z     = 0.293,
         .roll  = 3.059,
         .pitch = -0.697,
         .yaw   = -0.352},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.005, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.000096,
         .joint_2 = 0.219647,
         .joint_3 = -1.368023,
         .joint_4 = -0.006136,
         .joint_5 = 1.347602,
         .joint_6 = -0.044773},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         }                                                                                     }, // as same as auto_walk
        {                    "extract_rb",
         {
         Action::Step::makeOpenGripper(),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.000096,
         .joint_2 = 0.219647,
         .joint_3 = -1.368023,
         .joint_4 = -0.006136,
         .joint_5 = 1.347602,
         .joint_6 = -0.044773},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -2.027156,
         .joint_2 = 0.525101,
         .joint_3 = -1.383842,
         .joint_4 = 0.008437,
         .joint_5 = 0.916170,
         .joint_6 = 0.010163},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -2.590510,
         .joint_2 = 0.370552,
         .joint_3 = -1.369365,
         .joint_4 = 0.006519,
         .joint_5 = 0.990376,
         .joint_6 = -0.054840},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.005}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.10},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.15},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.162,
         .y     = -0.100,
         .z     = 0.345,
         .roll  = 0.814,
         .pitch = -1.377,
         .yaw   = 2.871},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.1}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.180,
         .y     = -0.240,
         .z     = 0.423,
         .roll  = 1.089,
         .pitch = -1.350,
         .yaw   = -1.966},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.1}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.000096,
         .joint_2 = 0.219647,
         .joint_3 = -1.368023,
         .joint_4 = -0.006136,
         .joint_5 = 1.347602,
         .joint_6 = -0.044773},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         }                                                                                     }, // as same as auto_walk
        {                    "extract_rf",
         {
         Action::Step::makeOpenGripper(),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.000096,
         .joint_2 = 0.219647,
         .joint_3 = -1.368023,
         .joint_4 = -0.006136,
         .joint_5 = 1.347602,
         .joint_6 = -0.044773},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -2.027156,
         .joint_2 = 0.525101,
         .joint_3 = -1.383842,
         .joint_4 = 0.008437,
         .joint_5 = 0.916170,
         .joint_6 = 0.010163},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -2.590510,
         .joint_2 = 0.370552,
         .joint_3 = -1.369365,
         .joint_4 = 0.006519,
         .joint_5 = 0.990376,
         .joint_6 = -0.054840},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.005}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.10},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.15},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.162,
         .y     = -0.100,
         .z     = 0.345,
         .roll  = 0.814,
         .pitch = -1.377,
         .yaw   = 2.871},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.1}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.180,
         .y     = -0.240,
         .z     = 0.423,
         .roll  = 1.089,
         .pitch = -1.350,
         .yaw   = -1.966},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.1}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.000096,
         .joint_2 = 0.219647,
         .joint_3 = -1.368023,
         .joint_4 = -0.006136,
         .joint_5 = 1.347602,
         .joint_6 = -0.044773},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         }                                                                                     },
        {                    "storage_lf",
         {
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.000096,
         .joint_2 = 0.219647,
         .joint_3 = -1.368023,
         .joint_4 = -0.006136,
         .joint_5 = 1.347602,
         .joint_6 = -0.044773},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 1.648742,
         .joint_2 = 0.284266,
         .joint_3 = -1.308006,
         .joint_4 = 0.007095,
         .joint_5 = 1.466294,
         .joint_6 = -0.000096},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 2.903250,
         .joint_2 = 0.382249,
         .joint_3 = -1.289982,
         .joint_4 = -0.080055,
         .joint_5 = 1.534077,
         .joint_6 = 0.045348},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 2.883021,
         .joint_2 = 0.376976,
         .joint_3 = -1.028246,
         .joint_4 = -0.074973,
         .joint_5 = 1.512026,
         .joint_6 = 0.045444},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 2.594633,
         .joint_2 = 0.490394,
         .joint_3 = -0.988938,
         .joint_4 = -0.220222,
         .joint_5 = 0.472945,
         .joint_6 = -0.271419},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = -1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.16},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.2, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makeOpenGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = 1.0, .distance = 0.085},
         Action::MotionParams{
         .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.200,
         .y     = 0.107,
         .z     = 0.124,
         .roll  = 2.526,
         .pitch = -1.326,
         .yaw   = 0.103},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.000096,
         .joint_2 = 0.219647,
         .joint_3 = -1.368023,
         .joint_4 = -0.006136,
         .joint_5 = 1.347602,
         .joint_6 = -0.044773},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         }                                                                                     },
        {                    "storage_lb",
         {
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.000096,
         .joint_2 = 0.219647,
         .joint_3 = -1.368023,
         .joint_4 = -0.006136,
         .joint_5 = 1.347602,
         .joint_6 = -0.044773},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 1.648742,
         .joint_2 = 0.284266,
         .joint_3 = -1.308006,
         .joint_4 = 0.007095,
         .joint_5 = 1.466294,
         .joint_6 = -0.000096},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 2.903250,
         .joint_2 = 0.382249,
         .joint_3 = -1.289982,
         .joint_4 = -0.080055,
         .joint_5 = 1.534077,
         .joint_6 = 0.045348},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 2.883021,
         .joint_2 = 0.376976,
         .joint_3 = -1.028246,
         .joint_4 = -0.074973,
         .joint_5 = 1.512026,
         .joint_6 = 0.045444},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 2.594633,
         .joint_2 = 0.490394,
         .joint_3 = -0.988938,
         .joint_4 = -0.220222,
         .joint_5 = 0.472945,
         .joint_6 = -0.271419},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = -1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.16},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.2, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makeOpenGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = 1.0, .distance = 0.085},
         Action::MotionParams{
         .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.200,
         .y     = 0.107,
         .z     = 0.124,
         .roll  = 2.526,
         .pitch = -1.326,
         .yaw   = 0.103},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.000096,
         .joint_2 = 0.219647,
         .joint_3 = -1.368023,
         .joint_4 = -0.006136,
         .joint_5 = 1.347602,
         .joint_6 = -0.044773},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         }                                                                                     },
        {                    "storage_rb",
         {
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.000096,
         .joint_2 = 0.219647,
         .joint_3 = -1.368023,
         .joint_4 = -0.006136,
         .joint_5 = 1.347602,
         .joint_6 = -0.044773},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -1.627362,
         .joint_2 = 0.283499,
         .joint_3 = -1.138310,
         .joint_4 = -0.006999,
         .joint_5 = 1.368694,
         .joint_6 = -0.000000},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -2.603549,
         .joint_2 = 0.471028,
         .joint_3 = -0.938892,
         .joint_4 = 0.006711,
         .joint_5 = 0.489723,
         .joint_6 = -0.108625},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = -1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.13},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
         Action::Step::makeOpenGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = 1.0, .distance = 0.10},
         Action::MotionParams{
         .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.127,
         .y     = -0.083,
         .z     = 0.234,
         .roll  = 3.139,
         .pitch = -1.462,
         .yaw   = 0.582},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.068,
         .y     = -0.302,
         .z     = 0.433,
         .roll  = 1.171,
         .pitch = 1.278,
         .yaw   = -2.559},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.000096,
         .joint_2 = 0.219647,
         .joint_3 = -1.368023,
         .joint_4 = -0.006136,
         .joint_5 = 1.347602,
         .joint_6 = -0.044773},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         }                                                                                     },
        {                    "storage_rf",
         {
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.000096,
         .joint_2 = 0.219647,
         .joint_3 = -1.368023,
         .joint_4 = -0.006136,
         .joint_5 = 1.347602,
         .joint_6 = -0.044773},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -1.627362,
         .joint_2 = 0.283499,
         .joint_3 = -1.138310,
         .joint_4 = -0.006999,
         .joint_5 = 1.368694,
         .joint_6 = -0.000000},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -2.603549,
         .joint_2 = 0.471028,
         .joint_3 = -0.938892,
         .joint_4 = 0.006711,
         .joint_5 = 0.489723,
         .joint_6 = -0.108625},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = -1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.13},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
         Action::Step::makeOpenGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = 1.0, .distance = 0.10},
         Action::MotionParams{
         .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.127,
         .y     = -0.083,
         .z     = 0.234,
         .roll  = 3.139,
         .pitch = -1.462,
         .yaw   = 0.582},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.068,
         .y     = -0.302,
         .z     = 0.433,
         .roll  = 1.171,
         .pitch = 1.278,
         .yaw   = -2.559},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.000096,
         .joint_2 = 0.219647,
         .joint_3 = -1.368023,
         .joint_4 = -0.006136,
         .joint_5 = 1.347602,
         .joint_6 = -0.044773},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         }                                                                                     },
        {  "transition_to_storage_mine_1",
         {
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.15},
         Action::MotionParams{
         .vel = 0.06, .acc = 0.05, .tolerance_pos = 0.008, .tolerance_ori = 0.005}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.023777,
         .joint_2 = 1.006291,
         .joint_3 = -0.310823,
         .joint_4 = -0.016394,
         .joint_5 = -0.639574,
         .joint_6 = 0.055127},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.022914,
         .joint_2 = 1.290078,
         .joint_3 = -0.832280,
         .joint_4 = 0.022626,
         .joint_5 = -0.394329,
         .joint_6 = 0.032022},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         }                                                                                     },
        {  "transition_to_extract_mine_2",
         {
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.022914,
         .joint_2 = 1.290078,
         .joint_3 = -0.832280,
         .joint_4 = 0.022626,
         .joint_5 = -0.394329,
         .joint_6 = 0.032022},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.054,
         .y     = 0.133,
         .z     = 0.606,
         .roll  = -2.072,
         .pitch = -0.192,
         .yaw   = 2.842},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.199208,
         .y     = 0.117225,
         .z     = 0.535502,
         .roll  = -1.570796,
         .pitch = -0.523599,
         .yaw   = 1.570796},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.123},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.18},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.054,
         .y     = 0.133,
         .z     = 0.606,
         .roll  = -2.072,
         .pitch = -0.192,
         .yaw   = 2.842},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.022914,
         .joint_2 = 1.290078,
         .joint_3 = -0.832280,
         .joint_4 = 0.022626,
         .joint_5 = -0.394329,
         .joint_6 = 0.032022},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         }                                                                                     },
        {  "transition_to_extract_mine_3",
         {
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.023777,
         .joint_2 = 1.006291,
         .joint_3 = -0.310823,
         .joint_4 = -0.016394,
         .joint_5 = -0.639574,
         .joint_6 = 0.055127},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -1.155471,
         .joint_2 = 0.658557,
         .joint_3 = -0.187050,
         .joint_4 = 0.298839,
         .joint_5 = -0.053785,
         .joint_6 = -0.799396},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.199208,
         .y     = -0.117227,
         .z     = 0.535502,
         .roll  = 1.570796,
         .pitch = -0.523599,
         .yaw   = -1.570796},

         Action::MotionParams{
         .vel = 0.01, .acc = 0.02, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.125},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.19},
         Action::MotionParams{
         .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -1.155471,
         .joint_2 = 0.658557,
         .joint_3 = -0.187050,
         .joint_4 = 0.298839,
         .joint_5 = -0.053785,
         .joint_6 = -0.799396},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         }                                                                                     },
        {  "transition_to_storage_mine_4",
         {
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.080},
         Action::MotionParams{
         .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.000288,
         .joint_2 = 0.308426,
         .joint_3 = -1.221624,
         .joint_4 = -0.006328,
         .joint_5 = 0.942535,
         .joint_6 = 1.514806},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         }                                                                                     },
        {  "transition_to_extract_mine_5",
         {
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.871397,
         .joint_2 = 0.799971,
         .joint_3 = -0.693072,
         .joint_4 = -1.534364,
         .joint_5 = -0.881847,
         .joint_6 = 0.564409},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.14},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.20},
         Action::MotionParams{
         .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.172,
         .y     = 0.161,
         .z     = 0.245,
         .roll  = -1.478,
         .pitch = 0.256,
         .yaw   = 2.348},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.01, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         }                                                                                     },
        {  "transition_to_extract_mine_6",
         {
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.964778,
         .joint_2 = 0.604676,
         .joint_3 = -1.376173,
         .joint_4 = 0.006136,
         .joint_5 = 0.946658,
         .joint_6 = -0.044773},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -1.058351,
         .joint_2 = 0.604676,
         .joint_3 = -0.830746,
         .joint_4 = -0.101722,
         .joint_5 = 0.971873,
         .joint_6 = -1.437723},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.199208,
         .y     = -0.117227,
         .z     = 0.400141,
         .roll  = 1.616,
         .pitch = 0.639,
         .yaw   = -1.676},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.01, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.15},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.20},
         Action::MotionParams{
         .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.964778,
         .joint_2 = 0.604676,
         .joint_3 = -1.376173,
         .joint_4 = 0.006136,
         .joint_5 = 0.946658,
         .joint_6 = -0.044773},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.000096,
         .joint_2 = 0.219647,
         .joint_3 = -1.368023,
         .joint_4 = -0.006136,
         .joint_5 = 1.347602,
         .joint_6 = -0.044773},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001}),
         }                                                                                     },
    };
}

} // namespace rmcs_core::controller::arm
