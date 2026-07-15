#pragma once
#include "controller/arm/arm_action/action_step.hpp"
#include <string>
#include <unordered_map>
#include <vector>

namespace rmcs_core::controller::arm {

inline const std::unordered_map<std::string, std::vector<Action::Step>> kActionParameterMap = {
    // ---------- 简单动作 ----------
    { "gripper_open",{Action::Step::makeOpenGripper()}                     },

    {"gripper_close",  {Action::Step::makeCloseGripper()}},
    {"delay",  {Action::Step::makeDelay()}},
    {         "test",
     {
     Action::Step::makeOpenGripper(),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = 0.281561,
     .y     = -0.033675,
     .z     = 0.388642,
     .roll  = 0.725640,
     .pitch = -1.475059,
     .yaw   = -0.839970},
     Action::MotionParams{
     .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = -0.062,
     .y     = -0.239,
     .z     = 0.225,
     .roll  = 2.981,
     .pitch = -1.519,
     .yaw   = 1.483},
     Action::MotionParams{
     .vel = 0.04, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x = -0.240, .y = -0.079, .z = 0.187, .roll = 2.355, .pitch = -1.57, .yaw = 1.23},
     Action::MotionParams{
     .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
     }                                                   },
    {    "auto_walk",
     {Action::Step::makeJoint(
     Action::JointTarget{
     .joint_1 = 0.0,
     .joint_2 = 1.23,
     .joint_3 = -1.36,
     .joint_4 = 0.0,
     .joint_5 = 0.63,
     .joint_6 = 0.0},
     Action::MotionParams{.vel = 0.03, .acc = 0.03})}    },

    {"up_one_stairs",
     {Action::Step::makeJoint(
     Action::JointTarget{
     .joint_1 = 0.0,
     .joint_2 = 0.25,
     .joint_3 = -0.55,
     .joint_4 = 0.0,
     .joint_5 = 0.27,
     .joint_6 = 0.0},
     Action::MotionParams{.vel = 0.05, .acc = 0.03})}    },

    {      "initial",
     {Action::Step::makeJoint(
     Action::JointTarget{
     .joint_1 = 0.0,
     .joint_2 = 0.25,
     .joint_3 = -0.55,
     .joint_4 = 0.0,
     .joint_5 = 0.27,
     .joint_6 = 0.0},
     Action::MotionParams{.vel = 0.05, .acc = 0.03})}    },

    {"initial_again",
     {Action::Step::makeJoint(
     Action::JointTarget{
     .joint_1 = 0.0,
     .joint_2 = 1.23,
     .joint_3 = -1.36,
     .joint_4 = 0.0,
     .joint_5 = 0.63,
     .joint_6 = 0.0},
     Action::MotionParams{.vel = 0.06, .acc = 0.04})}    },

    {   "lift_again",
     {Action::Step::makeJoint(
     Action::JointTarget{
     .joint_1 = 0.0,
     .joint_2 = 1.0,
     .joint_3 = -1.0,
     .joint_4 = 0.0,
     .joint_5 = 0.27,
     .joint_6 = 0.0},
     Action::MotionParams{.vel = 0.03, .acc = 0.03})}    },

    // ---------- 组合动作 ----------
    {   "extract_lf",
     {
     Action::Step::makeOpenGripper(),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = 0.281561,
     .y     = -0.033675,
     .z     = 0.388642,
     .roll  = 0.725640,
     .pitch = -1.475059,
     .yaw   = -0.839970},
     Action::MotionParams{
     .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x = -0.100, .y = 0.209, .z = 0.181, .roll = 0.307, .pitch = -1.305, .yaw = 1.770},
     Action::MotionParams{
     .vel = 0.04, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x = -0.201, .y = 0.125, .z = 0.168, .roll = 1.296, .pitch = -1.444, .yaw = 1.245},
     Action::MotionParams{
     .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.01}),
     Action::Step::makeLinear(
     Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.143},
     Action::MotionParams{
     .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
     Action::Step::makeCloseGripper(),
     Action::Step::makeLinear(
     Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.15},
     Action::MotionParams{
     .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = 0.221731,
     .y     = 0.002842,
     .z     = 0.308955,
     .roll  = -3.084697,
     .pitch = -1.097439,
     .yaw   = 3.096949},
     Action::MotionParams{
     .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
     }                                                   },
    {   "extract_lb",
     {
     Action::Step::makeOpenGripper(),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = 0.281561,
     .y     = -0.033675,
     .z     = 0.388642,
     .roll  = 0.725640,
     .pitch = -1.475059,
     .yaw   = -0.839970},
     Action::MotionParams{
     .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x = -0.100, .y = 0.209, .z = 0.181, .roll = 0.307, .pitch = -1.305, .yaw = 1.770},
     Action::MotionParams{
     .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = -0.258,
     .y     = 0.070,
     .z     = 0.185,
     .roll  = -0.291,
     .pitch = -1.446,
     .yaw   = -3.084},
     Action::MotionParams{
     .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
     Action::Step::makeLinear(
     Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.190},
     Action::MotionParams{
     .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
     Action::Step::makeCloseGripper(),
     Action::Step::makeLinear(
     Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.17},
     Action::MotionParams{
     .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = 0.221731,
     .y     = 0.002842,
     .z     = 0.308955,
     .roll  = -3.084697,
     .pitch = -1.097439,
     .yaw   = 3.096949},
     Action::MotionParams{
     .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
     }                                                   },
    {   "extract_rf",
     {
     Action::Step::makeOpenGripper(),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = 0.281561,
     .y     = -0.033675,
     .z     = 0.388642,
     .roll  = 0.725640,
     .pitch = -1.475059,
     .yaw   = -0.839970},
     Action::MotionParams{
     .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = -0.062,
     .y     = -0.239,
     .z     = 0.225,
     .roll  = 2.981,
     .pitch = -1.519,
     .yaw   = 1.483},
     Action::MotionParams{
     .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = -0.199,
     .y     = -0.152,
     .z     = 0.168,
     .roll  = -0.138,
     .pitch = -1.568567,
     .yaw   = -2.666},
     Action::MotionParams{
     .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
     Action::Step::makeLinear(
     Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.125},
     Action::MotionParams{
     .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
     Action::Step::makeCloseGripper(),
     Action::Step::makeLinear(
     Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.15},
     Action::MotionParams{
     .vel = 0.06, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = 0.221731,
     .y     = 0.002842,
     .z     = 0.308955,
     .roll  = -3.084697,
     .pitch = -1.097439,
     .yaw   = 3.096949},
     Action::MotionParams{
     .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
     }                                                   },
    {   "extract_rb",
     {
     Action::Step::makeOpenGripper(),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = 0.281561,
     .y     = -0.033675,
     .z     = 0.388642,
     .roll  = 0.725640,
     .pitch = -1.475059,
     .yaw   = -0.839970},
     Action::MotionParams{
     .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = -0.062,
     .y     = -0.239,
     .z     = 0.225,
     .roll  = 2.981,
     .pitch = -1.519,
     .yaw   = 1.483},
     Action::MotionParams{
     .vel = 0.04, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x = -0.240, .y = -0.079, .z = 0.187, .roll = 2.355, .pitch = -1.57, .yaw = 1.23},
     Action::MotionParams{
     .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
     Action::Step::makeLinear(
     Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.20},
     Action::MotionParams{
     .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
     Action::Step::makeCloseGripper(),
     Action::Step::makeLinear(
     Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.17},
     Action::MotionParams{
     .vel = 0.06, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = 0.221731,
     .y     = 0.002842,
     .z     = 0.308955,
     .roll  = -3.084697,
     .pitch = -1.097439,
     .yaw   = 3.096949},
     Action::MotionParams{
     .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
     }                                                   },
    {   "storage_lf",
     {
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = 0.281561,
     .y     = -0.033675,
     .z     = 0.388642,
     .roll  = 0.725640,
     .pitch = -1.475059,
     .yaw   = -0.839970},
     Action::MotionParams{
     .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = -0.138798,
     .y     = 0.338888,
     .z     = 0.355177,
     .roll  = -0.714410,
     .pitch = -1.365410,
     .yaw   = 2.684959},
     Action::MotionParams{
     .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x = -0.306, .y = 0.205, .z = 0.311, .roll = 1.296, .pitch = -1.444, .yaw = 1.245},
     Action::MotionParams{
     .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
     Action::Step::makeLinear(
     Action::LinearTarget{.dir_x = -1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.16},
     Action::MotionParams{
     .vel = 0.03, .acc = 0.2, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
     Action::Step::makeOpenGripper(),
     Action::Step::makeLinear(
     Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = 1.0, .distance = 0.11},
     Action::MotionParams{
     .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = 0.221731,
     .y     = 0.002842,
     .z     = 0.308955,
     .roll  = -3.084697,
     .pitch = -1.097439,
     .yaw   = 3.096949},
     Action::MotionParams{
     .vel = 0.06, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
     }                                                   },
    {   "storage_lb",
     {
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = 0.281561,
     .y     = -0.033675,
     .z     = 0.388642,
     .roll  = 0.725640,
     .pitch = -1.475059,
     .yaw   = -0.839970},
     Action::MotionParams{
     .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = -0.138156,
     .y     = 0.340795,
     .z     = 0.430307,
     .roll  = -0.714410,
     .pitch = -1.365410,
     .yaw   = 2.684959},
     Action::MotionParams{
     .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x = -0.441, .y = 0.122, .z = 0.31, .roll = 1.57, .pitch = -1.570, .yaw = 1.319},
     Action::MotionParams{
     .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
     Action::Step::makeLinear(
     Action::LinearTarget{.dir_x = -1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.16},
     Action::MotionParams{
     .vel = 0.03, .acc = 0.2, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
     Action::Step::makeOpenGripper(),
     Action::Step::makeLinear(
     Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = 1.0, .distance = 0.160},
     Action::MotionParams{
     .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = 0.221731,
     .y     = 0.002842,
     .z     = 0.308955,
     .roll  = -3.084697,
     .pitch = -1.097439,
     .yaw   = 3.096949},
     Action::MotionParams{
     .vel = 0.06, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
     }                                                   },
    {   "storage_rf",
     {
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = 0.281561,
     .y     = -0.033675,
     .z     = 0.388642,
     .roll  = 0.725640,
     .pitch = -1.475059,
     .yaw   = -0.839970},
     Action::MotionParams{
     .vel = 0.02, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = -0.138156,
     .y     = -0.360795,
     .z     = 0.390307,
     .roll  = 0.593263,
     .pitch = -1.490951,
     .yaw   = -2.549442},
     Action::MotionParams{
     .vel = 0.02, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = -0.300,
     .y     = -0.192,
     .z     = 0.309,
     .roll  = -0.138,
     .pitch = -1.568567,
     .yaw   = -2.666},
     Action::MotionParams{
     .vel = 0.02, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.01}),
     Action::Step::makeLinear(
     Action::LinearTarget{.dir_x = -1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.15},
     Action::MotionParams{
     .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
     Action::Step::makeOpenGripper(),
     Action::Step::makeLinear(
     Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = 1.0, .distance = 0.11},
     Action::MotionParams{
     .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = 0.221731,
     .y     = 0.002842,
     .z     = 0.308955,
     .roll  = -3.084697,
     .pitch = -1.097439,
     .yaw   = 3.096949},
     Action::MotionParams{
     .vel = 0.06, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
     }                                                   },
    {   "storage_rb",
     {
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = 0.281561,
     .y     = -0.033675,
     .z     = 0.388642,
     .roll  = 0.725640,
     .pitch = -1.475059,
     .yaw   = -0.839970},
     Action::MotionParams{
     .vel = 0.02, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = -0.138156,
     .y     = -0.340795,
     .z     = 0.420307,
     .roll  = 0.593263,
     .pitch = -1.490951,
     .yaw   = -2.549442},
     Action::MotionParams{
     .vel = 0.02, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = -0.442,
     .y     = -0.144,
     .z     = 0.311,
     .roll  = -0.525,
     .pitch = -1.425,
     .yaw   = -2.291},
     Action::MotionParams{
     .vel = 0.02, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
     Action::Step::makeLinear(
     Action::LinearTarget{.dir_x = -1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.17},
     Action::MotionParams{
     .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
     Action::Step::makeOpenGripper(),
     Action::Step::makeLinear(
     Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = 1.0, .distance = 0.137},
     Action::MotionParams{
     .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.02, .tolerance_ori = 0.2}),
     Action::Step::makePose(
     Action::PoseTarget{
     .x     = 0.221731,
     .y     = 0.002842,
     .z     = 0.308955,
     .roll  = -3.084697,
     .pitch = -1.097439,
     .yaw   = 3.096949},
     Action::MotionParams{
     .vel = 0.06, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
     }                                                   },
};

} // namespace rmcs_core::controller::arm
