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
         Action::MotionParams{.vel = 0.03, .acc = 0.03})}                                      },
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
         .joint_2 = 0.466426,
         .joint_3 = -0.018600,
         .joint_4 = 0.0,
         .joint_5 = -0.270172,
         .joint_6 = 0.0},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.0007, .tolerance_ori = 0.0007})}         },
        {"roll_out_in_three_mines_second",
         {Action::Step::makeOpenGripper(), Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 0.506118,
         .joint_3 = -0.781371,
         .joint_4 = -0.0,
         .joint_5 = 0.406505,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.03, .acc = 0.03})}                                      },
        {         "up_one_stairs_initial",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.03, .acc = 0.03})}                                      },

        {         "up_two_stairs_initial",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.03, .acc = 0.03})}                                      },

        {   "up_two_stairs_initial_again",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.03, .acc = 0.03})}                                      },

        {      "up_two_stairs_lift_again",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.03, .acc = 0.03})}                                      },

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
         Action::MotionParams{.vel = 0.03, .acc = 0.03}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.000096,
         .joint_2 = 0.219647,
         .joint_3 = -1.368023,
         .joint_4 = -0.006136,
         .joint_5 = 1.347602,
         .joint_6 = -0.044773},
         Action::MotionParams{.vel = 0.03, .acc = 0.03}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.186,
         .y     = 0.103,
         .z     = 0.100,
         .roll  = 2.772,
         .pitch = -1.501,
         .yaw   = -0.133},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.005}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.095},
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
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.000096,
         .joint_2 = 0.219647,
         .joint_3 = -1.368023,
         .joint_4 = -0.006136,
         .joint_5 = 1.347602,
         .joint_6 = -0.044773},
         Action::MotionParams{.vel = 0.03, .acc = 0.03}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.03, .acc = 0.03}),

         }                                                                                     },
        {                    "extract_lb",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.23,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.03, .acc = 0.03})}                                      }, // as same as auto_walk
        {                    "extract_rb",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.23,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.03, .acc = 0.03})}                                      }, // as same as auto_walk
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
         Action::MotionParams{.vel = 0.03, .acc = 0.03}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.000096,
         .joint_2 = 0.219647,
         .joint_3 = -1.368023,
         .joint_4 = -0.006136,
         .joint_5 = 1.347602,
         .joint_6 = -0.044773},
         Action::MotionParams{.vel = 0.03, .acc = 0.03}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.228,
         .y     = 0.001,
         .z     = 0.106,
         .roll  = -3.131,
         .pitch = -1.325,
         .yaw   = 3.138},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.188,
         .y     = -0.113,
         .z     = 0.112,
         .roll  = -3.079,
         .pitch = -1.520,
         .yaw   = 0.480},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.005}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.201,
         .y     = -0.125,
         .z     = 0.092,
         .roll  = 2.421,
         .pitch = -1.534,
         .yaw   = 1.281},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.08},
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
         Action::MotionParams{.vel = 0.03, .acc = 0.03}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.03, .acc = 0.03}),
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
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.228,
         .y     = 0.035,
         .z     = 0.188,
         .roll  = -3.001,
         .pitch = -0.921,
         .yaw   = -0.280},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.005}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.353,
         .y     = 0.119,
         .z     = 0.295,
         .roll  = -3.131,
         .pitch = -1.063,
         .yaw   = -0.331},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.253,
         .y     = 0.137,
         .z     = 0.264,
         .roll  = -3.141,
         .pitch = -1.440,
         .yaw   = -0.496},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.005}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = -1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.14},
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
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.23,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.03, .acc = 0.03})}                                      }, //  as same as storage_lf
        {                    "storage_rb",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.23,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.03, .acc = 0.03})}                                      }, //  as same as storage_rf
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
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.01, .tolerance_ori = 0.01}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.000096,
         .joint_2 = 0.219647,
         .joint_3 = -1.368023,
         .joint_4 = -0.006136,
         .joint_5 = 1.347602,
         .joint_6 = -0.044773},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.01, .tolerance_ori = 0.01}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -1.627362,
         .joint_2 = 0.283499,
         .joint_3 = -1.138310,
         .joint_4 = -0.006999,
         .joint_5 = 1.368694,
         .joint_6 = -0.000000},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.01, .tolerance_ori = 0.01}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.247,
         .y     = -0.166,
         .z     = 0.250,
         .roll  = -3.140,
         .pitch = -1.418,
         .yaw   = 0.591},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.005}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = -1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.13},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
         Action::Step::makeOpenGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = 1.0, .distance = 0.11},
         Action::MotionParams{
         .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.02, .tolerance_ori = 0.1}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.127,
         .y     = -0.083,
         .z     = 0.234,
         .roll  = 3.139,
         .pitch = -1.462,
         .yaw   = 0.582},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.068,
         .y     = -0.302,
         .z     = 0.433,
         .roll  = 1.171,
         .pitch = 1.278,
         .yaw   = -2.559},
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
         Action::MotionParams{.vel = 0.03, .acc = 0.03}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.03, .acc = 0.03}),
         }                                                                                     },
        {  "transition_to_storage_mine_1",
         {
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.13},
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
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.022914,
         .joint_2 = 1.290078,
         .joint_3 = -0.832280,
         .joint_4 = 0.022626,
         .joint_5 = -0.394329,
         .joint_6 = 0.032022},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
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
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.054,
         .y     = 0.133,
         .z     = 0.606,
         .roll  = -2.072,
         .pitch = -0.192,
         .yaw   = 2.842},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.199208,
         .y     = 0.117225,
         .z     = 0.555502,
         .roll  = -1.570796,
         .pitch = -0.523599,
         .yaw   = 1.570796},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.123},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.18},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.054,
         .y     = 0.133,
         .z     = 0.606,
         .roll  = -2.072,
         .pitch = -0.192,
         .yaw   = 2.842},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.022914,
         .joint_2 = 1.290078,
         .joint_3 = -0.832280,
         .joint_4 = 0.022626,
         .joint_5 = -0.394329,
         .joint_6 = 0.032022},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
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
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -1.155471,
         .joint_2 = 0.658557,
         .joint_3 = -0.187050,
         .joint_4 = 0.298839,
         .joint_5 = -0.053785,
         .joint_6 = -0.799396},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.199208,
         .y     = -0.117227,
         .z     = 0.555502,
         .roll  = 1.570796,
         .pitch = -0.523599,
         .yaw   = -1.570796},

         Action::MotionParams{
         .vel = 0.01, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.105},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.19},
         Action::MotionParams{
         .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -1.155471,
         .joint_2 = 0.658557,
         .joint_3 = -0.187050,
         .joint_4 = 0.298839,
         .joint_5 = -0.053785,
         .joint_6 = -0.799396},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.107534,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.03, .acc = 0.03}),
         }                                                                                     },
        {  "transition_to_extract_mine_4",
         {Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.080},
         Action::MotionParams{
         .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.008, .tolerance_ori = 0.005})}           },
        {  "transition_to_extract_mine_5",
         {Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.299208,
         .y     = 0.117225,
         .z     = 0.420141,
         .roll  = -1.469123,
         .pitch = 0.514541,
         .yaw   = 1.775219},

         Action::MotionParams{
         .vel = 0.02, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.271,
         .y     = 0.117224,
         .z     = 0.358077,
         .roll  = -1.570796,
         .pitch = 0.523599,
         .yaw   = 1.570796},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.001, .tolerance_ori = 0.02}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.110},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.20},
         Action::MotionParams{
         .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.159,
         .y     = 0.249,
         .z     = 0.461,
         .roll  = -1.530,
         .pitch = -0.503,
         .yaw   = 2.349},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.046,
         .y     = -0.036,
         .z     = 0.582,
         .roll  = 0.517,
         .pitch = -0.946,
         .yaw   = -1.023},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.02})}            },
        {  "transition_to_extract_mine_6",
         {Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.299208,
         .y     = -0.117227,
         .z     = 0.420141,
         .roll  = 1.469118,
         .pitch = 0.514531,
         .yaw   = -1.775218},

         Action::MotionParams{
         .vel = 0.02, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.271,
         .y     = 0.117224,
         .z     = 0.358077,
         .roll  = -1.570796,
         .pitch = 0.523599,
         .yaw   = 1.570796},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.001, .tolerance_ori = 0.02}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.110},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.20},
         Action::MotionParams{
         .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.159,
         .y     = 0.249,
         .z     = 0.461,
         .roll  = -1.530,
         .pitch = -0.503,
         .yaw   = 2.349},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.046,
         .y     = -0.036,
         .z     = 0.582,
         .roll  = 0.517,
         .pitch = -0.946,
         .yaw   = -1.023},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.02})}            },
    };
}

} // namespace rmcs_core::controller::arm
