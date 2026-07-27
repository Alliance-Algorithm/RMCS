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
        {               "gripper_open",{Action::Step::makeOpenGripper()}                                       },

        {              "gripper_close", {Action::Step::makeCloseGripper()}},
        {                      "delay",        {Action::Step::makeDelay()}},
        {                       "test",
         {
         Action::Step::makeOpenGripper(),
         }                                                                },
        {                  "auto_walk",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = 0.0,
         .joint_2 = 1.23,
         .joint_3 = -1.36,
         .joint_4 = 0.0,
         .joint_5 = 0.63,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.03, .acc = 0.03})}                 },

        {      "up_one_stairs_initial",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.028187,
         .joint_2 = 1.452584,
         .joint_3 = -1.346260,
         .joint_4 = 0.0,
         .joint_5 = 0.559999,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.03, .acc = 0.03})}                 },

        {      "up_two_stairs_initial",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.028187,
         .joint_2 = 1.452584,
         .joint_3 = -1.346260,
         .joint_4 = 0.0,
         .joint_5 = 0.559999,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.03, .acc = 0.03})}                 },

        {"up_two_stairs_initial_again",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.028187,
         .joint_2 = 1.452584,
         .joint_3 = -1.346260,
         .joint_4 = 0.0,
         .joint_5 = 0.559999,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.03, .acc = 0.03})}                 },

        {   "up_two_stairs_lift_again",
         {Action::Step::makeJoint(
         Action::JointTarget{
         .joint_1 = -0.028187,
         .joint_2 = 1.452584,
         .joint_3 = -1.346260,
         .joint_4 = 0.0,
         .joint_5 = 0.559999,
         .joint_6 = 0.0},
         Action::MotionParams{.vel = 0.03, .acc = 0.03})}                 },

        // ---------- 组合动作 ----------
        {                 "extract_lf",
         {
         Action::Step::makeOpenGripper(),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.271,
         .y     = -0.012,
         .z     = 0.441,
         .roll  = 3.127,
         .pitch = -1.272,
         .yaw   = 3.114},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.1}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.151,
         .y     = 0.098,
         .z     = 0.234,
         .roll  = 2.165,
         .pitch = -1.299,
         .yaw   = 0.415},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.1}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.200,
         .y     = 0.107,
         .z     = 0.124,
         .roll  = 2.526,
         .pitch = -1.326,
         .yaw   = 0.103},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.005}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.085},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.20},
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
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.271,
         .y     = -0.012,
         .z     = 0.441,
         .roll  = 3.127,
         .pitch = -1.272,
         .yaw   = 3.114},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.1}),
         }                                                                },
        {                 "extract_lb",
         {
         Action::Step::makeOpenGripper(),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.271,
         .y     = -0.012,
         .z     = 0.441,
         .roll  = 3.127,
         .pitch = -1.272,
         .yaw   = 3.114},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.191,
         .y     = 0.046,
         .z     = 0.314,
         .roll  = -3.047,
         .pitch = -1.145,
         .yaw   = -0.176},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.285,
         .y     = -0.035,
         .z     = 0.083,
         .roll  = 1.478,
         .pitch = -1.354,
         .yaw   = 2.205},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.07},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.20},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.05}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.174,
         .y     = 0.110,
         .z     = 0.336,
         .roll  = 1.063,
         .pitch = -1.340,
         .yaw   = 1.502},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.255,
         .y     = 0.015,
         .z     = 0.413,
         .roll  = 3.072,
         .pitch = -0.551,
         .yaw   = -2.910},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
         }                                                                },
        {                 "extract_rf",
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
         }                                                                },
        {                 "extract_rb",
         {
         Action::Step::makeOpenGripper(),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.304,
         .y     = -0.014,
         .z     = 0.441,
         .roll  = 2.529,
         .pitch = -1.434,
         .yaw   = -2.572},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.1}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.068,
         .y     = -0.302,
         .z     = 0.433,
         .roll  = 1.171,
         .pitch = 1.278,
         .yaw   = -2.559},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.127,
         .y     = -0.083,
         .z     = 0.234,
         .roll  = 3.139,
         .pitch = -1.462,
         .yaw   = 0.582},
         Action::MotionParams{
         .vel = 0.01, .acc = 0.01, .tolerance_pos = 0.003, .tolerance_ori = 0.005}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.211,
         .y     = -0.129,
         .z     = 0.136,
         .roll  = 2.445,
         .pitch = -1.387,
         .yaw   = 1.243},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.08},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
         Action::Step::makeCloseGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.20},
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
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.335,
         .y     = -0.008,
         .z     = 0.390,
         .roll  = 1.383,
         .pitch = -1.469,
         .yaw   = -1.408},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.1}),
         }                                                                },
        {                 "storage_lf",
         {
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.271,
         .y     = -0.012,
         .z     = 0.441,
         .roll  = 3.127,
         .pitch = -1.272,
         .yaw   = 3.114},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.1}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.146,
         .y     = 0.083,
         .z     = 0.313,
         .roll  = 2.960,
         .pitch = -0.958,
         .yaw   = -0.324},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.267,
         .y     = 0.134,
         .z     = 0.293,
         .roll  = 2.793,
         .pitch = -1.268,
         .yaw   = -0.134},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.005}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = -1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.10},
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
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.151,
         .y     = 0.098,
         .z     = 0.234,
         .roll  = 2.165,
         .pitch = -1.299,
         .yaw   = 0.415},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.1}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.271,
         .y     = -0.012,
         .z     = 0.441,
         .roll  = 3.127,
         .pitch = -1.272,
         .yaw   = 3.114},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.02, .tolerance_pos = 0.003, .tolerance_ori = 0.1}),
         }                                                                },
        {                 "storage_lb",
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
         .x     = 0.221731,
         .y     = 0.002842,
         .z     = 0.308955,
         .roll  = -3.084697,
         .pitch = -1.097439,
         .yaw   = 3.096949},
         Action::MotionParams{
         .vel = 0.06, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
         }                                                                },
        {                 "storage_rf",
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
         }                                                                },
        {                 "storage_rb",
         {
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.304,
         .y     = -0.014,
         .z     = 0.441,
         .roll  = 2.529,
         .pitch = -1.434,
         .yaw   = -2.572},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.1}),
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = -0.266,
         .y     = -0.157,
         .z     = 0.268,
         .roll  = 2.823,
         .pitch = -1.290,
         .yaw   = 0.822},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.005}),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = -1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.10},
         Action::MotionParams{
         .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.008}),
         Action::Step::makeOpenGripper(),
         Action::Step::makeLinear(
         Action::LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = 1.0, .distance = 0.08},
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
         Action::Step::makePose(
         Action::PoseTarget{
         .x     = 0.304,
         .y     = -0.014,
         .z     = 0.441,
         .roll  = 2.529,
         .pitch = -1.434,
         .yaw   = -2.572},
         Action::MotionParams{
         .vel = 0.02, .acc = 0.03, .tolerance_pos = 0.003, .tolerance_ori = 0.2}),
         }                                                                },
    };
}

} // namespace rmcs_core::controller::arm
