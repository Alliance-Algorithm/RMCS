#pragma once
#include "controller/arm/arm_action/action_step.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace rmcs_core::controller::arm {

inline auto make_lunar_rover_action_parameter_map() {
    using ParameterMap = std::unordered_map<std::string, std::vector<Action::Step>>;
    return ParameterMap{
        // ---------- 简单动作 ----------
        {                "gripper_open",{Action::Step::makeOpenGripper()}                                        },

        {               "gripper_close",                     {Action::Step::makeCloseGripper()}},
        {                       "delay",                            {Action::Step::makeDelay()}},
        {                        "test",                      {Action::Step::makeOpenGripper()}},
        {                   "auto_walk",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.23,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.03, .acc = 0.03})}                                      },
        {      "crash_wall_calibration",
         {Action::Step::makePose(
         Action::PoseTarget{
         .x = 0.008, .y = 0.004, .z = 0.621, .roll = 3.015, .pitch = -1.274, .yaw = -2.927},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.02})}            },
        {      "roll_out_in_five_mines",
         {Action::Step::makeOpenGripper(), Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 0.348405,
         .joint_3 = -0.068262,
         .joint_4 = 0.0,
         .joint_5 = -0.257901,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.03, .acc = 0.03})}                                      },
        {"transition_to_storage_mine_1",
         {Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.145},
         Action::MotionParams{
         .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.008, .tolerance_ori = 0.05})}            },
        {"transition_to_extract_mine_2",
         {
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.150,
         .y     = 0.148,
         .z     = 0.499,
         .roll  = -0.655,
         .pitch = -1.041,
         .yaw   = 1.483},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.176,
         .y     = 0.257,
         .z     = 0.486,
         .roll  = -1.408,
         .pitch = -0.351,
         .yaw   = 1.729},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.271,
         .y     = 0.117224,
         .z     = 0.49343819,
         .roll  = -1.570796,
         .pitch = -0.523599,
         .yaw   = 1.570796},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.001, .tolerance_ori = 0.02}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.115},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.18},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.191,
         .y     = 0.257,
         .z     = 0.606,
         .roll  = -1.221,
         .pitch = -0.614,
         .yaw   = 2.203},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         }                                                                                     },
        {"transition_to_extract_mine_3",
         {Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.159,
         .y     = -0.005,
         .z     = 0.548,
         .roll  = 1.362,
         .pitch = -0.730,
         .yaw   = -1.427},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.271,
         .y     = -0.117227,
         .z     = 0.4934382,
         .roll  = 1.570796,
         .pitch = -0.523599,
         .yaw   = -1.570796},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.02, .tolerance_pos = 0.001, .tolerance_ori = 0.02}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.115},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.18},
         Action::MotionParams{
         .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05})}            },
        {"transition_to_extract_mine_4",
         {
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.215,
         .y     = -0.220,
         .z     = 0.305,
         .roll  = 1.359,
         .pitch = 0.460,
         .yaw   = -2.437},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.271,
         .y     = -0.117227,
         .z     = 0.3580773,
         .roll  = 1.570796,
         .pitch = 0.523599,
         .yaw   = -1.570796},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.0008, .tolerance_ori = 0.02}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.110},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.19},
         Action::MotionParams{
         .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.211,
         .y     = -0.231,
         .z     = 0.592,
         .roll  = -1.860,
         .pitch = 0.212,
         .yaw   = 0.826},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         }                                                                                     },
        {"transition_to_extract_mine_5",
         {Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.150,
         .y     = 0.148,
         .z     = 0.499,
         .roll  = -0.655,
         .pitch = -1.041,
         .yaw   = 1.483},
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

        {       "up_one_stairs_initial",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 0.25,
         .joint_3 = -0.55,
         .joint_4 = 0.0,
         .joint_5 = 0.27,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.05, .acc = 0.03})}                                      },

        {       "up_two_stairs_initial",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 0.25,
         .joint_3 = -0.55,
         .joint_4 = 0.0,
         .joint_5 = 0.27,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.05, .acc = 0.03})}                                      },

        { "up_two_stairs_initial_again",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.321045,
         .joint_3 = -0.555301,
         .joint_4 = 0.0,
         .joint_5 = 1.228239,
         .joint_6 = 0.042280},
         Action::MotionParams{.vel = 0.06, .acc = 0.04})
        }                                      },

        {    "up_two_stairs_lift_again",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.0,
         .joint_3 = -1.0,
         .joint_4 = 0.0,
         .joint_5 = 0.27,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.03, .acc = 0.03})}                                      },

        // ---------- 组合动作 ----------
        {                  "extract_lf",
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
         .x     = -0.100,
         .y     = 0.209,
         .z     = 0.181,
         .roll  = 0.307,
         .pitch = -1.305,
         .yaw   = 1.770},
         Action::MotionParams{
         .vel = 0.04, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.201,
         .y     = 0.125,
         .z     = 0.168,
         .roll  = 1.296,
         .pitch = -1.444,
         .yaw   = 1.245},
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
         }                                                                                     },
        {                  "extract_lb",
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
         .x     = -0.100,
         .y     = 0.209,
         .z     = 0.181,
         .roll  = 0.307,
         .pitch = -1.305,
         .yaw   = 1.770},
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
         }                                                                                     },
        {                  "extract_rf",
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
         }                                                                                     },
        {                  "extract_rb",
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
         .x     = -0.240,
         .y     = -0.079,
         .z     = 0.187,
         .roll  = 2.355,
         .pitch = -1.57,
         .yaw   = 1.23},
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
         }                                                                                     },
        {                  "storage_lf",
         {Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.046,
         .y     = -0.036,
         .z     = 0.582,
         .roll  = 0.517,
         .pitch = -0.946,
         .yaw   = -1.023},
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
         .x     = -0.306,
         .y     = 0.205,
         .z     = 0.311,
         .roll  = 1.296,
         .pitch = -1.444,
         .yaw   = 1.245},
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
         .x     = 0.046,
         .y     = -0.036,
         .z     = 0.582,
         .roll  = 0.517,
         .pitch = -0.946,
         .yaw   = -1.023},
         Action::MotionParams{
         .vel = 0.06, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2})}             },
        {                  "storage_lb",
         {
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.046,
         .y     = -0.036,
         .z     = 0.582,
         .roll  = 0.517,
         .pitch = -0.946,
         .yaw   = -1.023},
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
         .x     = -0.441,
         .y     = 0.122,
         .z     = 0.31,
         .roll  = 1.57,
         .pitch = -1.570,
         .yaw   = 1.319},
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
         .x     = 0.046,
         .y     = -0.036,
         .z     = 0.582,
         .roll  = 0.517,
         .pitch = -0.946,
         .yaw   = -1.023},
         Action::MotionParams{
         .vel = 0.06, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
         }                                                                                     },
        {                  "storage_rf",
         {
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.046,
         .y     = -0.036,
         .z     = 0.582,
         .roll  = 0.517,
         .pitch = -0.946,
         .yaw   = -1.023},
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
         .x     = -0.31,
         .y     = -0.20,
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
         .x     = 0.046,
         .y     = -0.036,
         .z     = 0.582,
         .roll  = 0.517,
         .pitch = -0.946,
         .yaw   = -1.023},
         Action::MotionParams{
         .vel = 0.06, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
         }                                                                                     },
        {                  "storage_rb",
         {
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.046,
         .y     = -0.036,
         .z     = 0.582,
         .roll  = 0.517,
         .pitch = -0.946,
         .yaw   = -1.023},
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
         .x     = 0.046,
         .y     = -0.036,
         .z     = 0.582,
         .roll  = 0.517,
         .pitch = -0.946,
         .yaw   = -1.023},
         Action::MotionParams{
         .vel = 0.06, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
         }                                                                                     },
    };
}

} // namespace rmcs_core::controller::arm
