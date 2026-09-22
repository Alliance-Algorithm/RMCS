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
        {                  "gripper_open",{Action::Step::makeOpenGripper()}                                          },

        {                 "gripper_close",                   {Action::Step::makeCloseGripper()}},
        {                         "delay",                          {Action::Step::makeDelay()}},
        {                          "test",
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
         .x     = -0.302,
         .y     = -0.187,
         .z     = 0.282,
         .roll  = 0.441,
         .pitch = -1.397,
         .yaw   = 3.108},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.01}),
         }                                                                                     },
        {                     "auto_walk",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.23,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.03, .acc = 0.03})}                                      },
        {        "crash_wall_calibration",
         {Action::Step::makePose(
         Action::PoseTarget{
         .x = 0.008, .y = 0.004, .z = 0.621, .roll = 3.015, .pitch = -1.274, .yaw = -2.927},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.02})}            },
        {        "roll_out_in_five_mines",
         {Action::Step::makeOpenGripper(), Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.00,
         .joint_2 = 0.327888,
         .joint_3 = -0.191460,
         .joint_4 = -0.000,
         .joint_5 = -0.104694,
         .joint_6 = -0.00},
         Action::MotionParams{.vel = 0.03, .acc = 0.03})}                                      },
        { "roll_out_in_three_mines_first",
         {Action::Step::makeOpenGripper(), Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.00,
         .joint_2 = 0.327888,
         .joint_3 = -0.191460,
         .joint_4 = -0.000,
         .joint_5 = -0.104694,
         .joint_6 = -0.00},
         Action::MotionParams{.vel = 0.03, .acc = 0.03})}                                      },
        {"roll_out_in_three_mines_second",
         {Action::Step::makeOpenGripper(), Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.0,
         .joint_2 = 0.234411,
         .joint_3 = -0.729600,
         .joint_4 = 3.14,
         .joint_5 = -0.547727,
         .joint_6 = -0.0},
         Action::MotionParams{.vel = 0.03, .acc = 0.03})}                                      },
        {  "transition_to_storage_mine_1",
         {Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.192},
         Action::MotionParams{
         .vel = 0.04, .acc = 0.03, .tolerance_pos = 0.008, .tolerance_ori = 0.05})}            },
        {  "transition_to_storage_mine_6",
         {Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.192},
         Action::MotionParams{
         .vel = 0.04, .acc = 0.03, .tolerance_pos = 0.008, .tolerance_ori = 0.05}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.047362,
         .joint_2 = 0.930551,
         .joint_3 = -1.382980,
         .joint_4 = 3.141497,
         .joint_5 = -0.548111,
         .joint_6 = -0.013710},
         Action::MotionParams{.vel = 0.03, .acc = 0.03})}                                      },
        {  "transition_to_extract_mine_2",
         {
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 1.304267,
         .joint_2 = 0.422228,
         .joint_3 = 0.059346,
         .joint_4 = -1.287489,
         .joint_5 = -1.129393,
         .joint_6 = -0.069029},
         Action::MotionParams{.vel = 0.03, .acc = 0.03}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.292978,
         .y     = 0.129224,
         .z     = 0.441641,
         .roll  = -1.570796,
         .pitch = -0.523599,
         .yaw   = 1.570796},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.0008, .tolerance_ori = 0.001}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.12},
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
        {  "transition_to_extract_mine_3",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -1.092674,
         .joint_2 = 0.800642,
         .joint_3 = -0.250806,
         .joint_4 = 0.886066,
         .joint_5 = -0.419927,
         .joint_6 = -0.666419},
         Action::MotionParams{.vel = 0.03, .acc = 0.03}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.292978,
         .y     = -0.130228,
         .z     = 0.456641,
         .roll  = 1.570796,
         .pitch = -0.523599,
         .yaw   = -1.570796},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.02, .tolerance_pos = 0.0005, .tolerance_ori = 0.001}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.125},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.22},
         Action::MotionParams{
         .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05})}            },
        {  "transition_to_extract_mine_4",
         {
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.949246,
         .joint_2 = 0.602375,
         .joint_3 = 0.007095,
         .joint_4 = 0.925662,
         .joint_5 = -0.031638,
         .joint_6 = -1.106480},
         Action::MotionParams{.vel = 0.03, .acc = 0.03}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.949246,
         .joint_2 = 0.602183,
         .joint_3 = -0.798054,
         .joint_4 = 0.925662,
         .joint_5 = -0.031638,
         .joint_6 = -1.104274},
         Action::MotionParams{.vel = 0.03, .acc = 0.03}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.292978,
         .y     = -0.109228,
         .z     = 0.325280,
         .roll  = 1.570796,
         .pitch = 0.523599,
         .yaw   = -1.570796},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.0001, .tolerance_ori = 0.0005}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.121},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.22},
         Action::MotionParams{
         .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.991335,
         .joint_2 = 0.468727,
         .joint_3 = -0.446005,
         .joint_4 = 1.374063,
         .joint_5 = 0.055223,
         .joint_6 = -0.049950},
         Action::MotionParams{.vel = 0.03, .acc = 0.03}),
         }                                                                                     },
        {  "transition_to_extract_mine_5",
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
         .x     = 0.292978,
         .y     = 0.108224,
         .z     = 0.317280,
         .roll  = -1.570796,
         .pitch = 0.523599,
         .yaw   = 1.570796},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.0008, .tolerance_ori = 0.001}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.13},
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

        {         "up_one_stairs_initial",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 0.25,
         .joint_3 = -0.55,
         .joint_4 = 0.0,
         .joint_5 = 0.27,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.05, .acc = 0.03})}                                      },

        {         "up_two_stairs_initial",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 0.25,
         .joint_3 = -0.55,
         .joint_4 = 0.0,
         .joint_5 = 0.27,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.05, .acc = 0.03})}                                      },

        {   "up_two_stairs_initial_again",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.23,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.06, .acc = 0.04})}                                      },

        {      "up_two_stairs_lift_again",
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
        {                    "extract_lf",
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
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 1.997914,
         .joint_2 = 0.792876,
         .joint_3 = -1.273204,
         .joint_4 = -0.051676,
         .joint_5 = 0.438623,
         .joint_6 = -0.069029},
         Action::MotionParams{.vel = 0.03, .acc = 0.03}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.209,
         .y     = 0.114,
         .z     = 0.155,
         .roll  = -0.370,
         .pitch = -1.547,
         .yaw   = 3.004},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.003}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.120},
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
        {                    "extract_lb",
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
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.357034,
         .joint_2 = 0.712055,
         .joint_3 = -1.336097,
         .joint_4 = 0.014573,
         .joint_5 = 0.613688,
         .joint_6 = 0.079288},
         Action::MotionParams{.vel = 0.02, .acc = 0.03}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 1.812302,
         .joint_2 = 0.815790,
         .joint_3 = -1.43976,
         .joint_4 = 0.014381,
         .joint_5 = 0.613592,
         .joint_6 = 0.079479},
         Action::MotionParams{.vel = 0.03, .acc = 0.03}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 2.991934,
         .joint_2 = 0.815215,
         .joint_3 = -1.410879,
         .joint_4 = -0.111597,
         .joint_5 = 0.615222,
         .joint_6 = 0.064331},
         Action::MotionParams{.vel = 0.02, .acc = 0.03}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 2.894601,
         .joint_2 = 0.447539,
         .joint_3 = -1.196217,
         .joint_4 = -0.034131,
         .joint_5 = 0.702276,
         .joint_6 = 0.064235},
         Action::MotionParams{.vel = 0.02, .acc = 0.02}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.170},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.18},
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
        {                    "extract_rf",
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
         .vel = 0.04, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.02}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -1.848734,
         .joint_2 = 0.884244,
         .joint_3 = -1.311074,
         .joint_4 = -0.075932,
         .joint_5 = 0.453100,
         .joint_6 = -0.058867},
         Action::MotionParams{.vel = 0.02, .acc = 0.03}),
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
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.120},
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
        {                    "extract_rb",
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
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.285608,
         .joint_2 = 0.653668,
         .joint_3 = -1.308006,
         .joint_4 = 0.086862,
         .joint_5 = 0.771784,
         .joint_6 = -0.048033},
         Action::MotionParams{.vel = 0.03, .acc = 0.03}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -1.732248,
         .joint_2 = 0.741776,
         .joint_3 = -1.354122,
         .joint_4 = 0.086958,
         .joint_5 = 0.658557,
         .joint_6 = -0.00968},
         Action::MotionParams{.vel = 0.02, .acc = 0.03}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -2.967477,
         .joint_2 = 0.530374,
         .joint_3 = -1.221816,
         .joint_4 = -0.340064,
         .joint_5 = 0.694510,
         .joint_6 = -0.231535},
         Action::MotionParams{.vel = 0.03, .acc = 0.02}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.19},
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
        {                    "storage_lf",
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
         .x = -0.296, .y = 0.19, .z = 0.311, .roll = 1.296, .pitch = -1.444, .yaw = 1.245},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = -1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.15},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.2, .tolerance_pos = 0.003, .tolerance_ori = 0.01}),
         Action::Step::makeOpenGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = 1.0, .distance = 0.11},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.046,
         .y     = -0.036,
         .z     = 0.582,
         .roll  = 0.517,
         .pitch = -0.946,
         .yaw   = -1.023},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2})}             },
        {                    "storage_lb",
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
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 1.824478,
         .joint_2 = 0.411586,
         .joint_3 = -0.387905,
         .joint_4 = 0.065386,
         .joint_5 = -0.033460,
         .joint_6 = 0.080246},
         Action::MotionParams{.vel = 0.02, .acc = 0.03}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 2.540560,
         .joint_2 = 0.199609,
         .joint_3 = -0.297209,
         .joint_4 = -0.108913,
         .joint_5 = 0.194624,
         .joint_6 = 0.045252},
         Action::MotionParams{.vel = 0.02, .acc = 0.03}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.402,
         .y     = 0.112,
         .z     = 0.308,
         .roll  = 2.900,
         .pitch = -1.452,
         .yaw   = 0.072},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = -1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.17},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.2, .tolerance_pos = 0.003, .tolerance_ori = 0.01}),
         Action::Step::makeOpenGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = 1.0, .distance = 0.195},
         Action::MotionParams{
         .vel = 0.06, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.046,
         .y     = -0.036,
         .z     = 0.582,
         .roll  = 0.517,
         .pitch = -0.946,
         .yaw   = -1.023},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
         }                                                                                     },
        {                    "storage_rf",
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
         .x     = -0.305,
         .y     = -0.180,
         .z     = 0.311,
         .roll  = -1.136,
         .pitch = -1.511,
         .yaw   = -1.675},
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
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
         }                                                                                     },
        {                    "storage_rb",
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
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -2.176431,
         .joint_2 = 0.264803,
         .joint_3 = -0.264036,
         .joint_4 = 0.062126,
         .joint_5 = -0.027899,
         .joint_6 = 0.034515},
         Action::MotionParams{.vel = 0.03, .acc = 0.03}),
         Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -2.695012,
         .joint_2 = 0.267200,
         .joint_3 = -0.270268,
         .joint_4 = 0.059250,
         .joint_5 = -0.029625,
         .joint_6 = 0.034035},
         Action::MotionParams{.vel = 0.03, .acc = 0.03}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.410,
         .y     = -0.109,
         .z     = 0.332,
         .roll  = 2.950,
         .pitch = -1.500,
         .yaw   = 0.448},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = -1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.18},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
         Action::Step::makeOpenGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = 1.0, .distance = 0.20},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.03, .tolerance_pos = 0.02, .tolerance_ori = 0.2}),
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
         }                                                                                     },
    };
}

} // namespace rmcs_core::controller::arm
