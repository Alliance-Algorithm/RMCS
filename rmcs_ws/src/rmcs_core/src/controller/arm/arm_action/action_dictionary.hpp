#pragma once
#include "controller/arm/arm_action/action_step.hpp"
#include <array>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace rmcs_core::controller::arm {

class ActionDictionary {
public:
    ActionDictionary()
        : parameter_dict_(make_parameter_dict()) {
        validate_check();
    }

    std::vector<Action::Step> helper_find_chunk(const std::string& name) const {
        if (auto it = parameter_dict_.find(name); it != parameter_dict_.end())
            return it->second;
        else {throw std::runtime_error("Invalid'" + name + "'in dictionary");}
    }

    std::vector<Action::Step> helper_build_chunk(const std::vector<std::string>& composed) {
        std::string cache_key;
        for (const auto& name : composed) {
            if (!cache_key.empty())
                cache_key += '|';
            cache_key += name;
        }
        if (auto it = helper_composed_cache_.find(cache_key); it != helper_composed_cache_.end()) {
            return it->second;
        }
      std::vector<Action::Step> result;
        for (const auto& name : composed) {
             auto chunk = helper_find_chunk(name);   // 只调用一次
            result.insert(result.end(),
                          std::make_move_iterator(chunk.begin()),
                          std::make_move_iterator(chunk.end()));
        }
        helper_composed_cache_.emplace(cache_key, std::move(result));
        return helper_composed_cache_[cache_key];
    }

private:
    void validate_check() const {
        for (const auto& [name, steps] : parameter_dict_) {
            if (steps.empty()) {
                throw std::runtime_error("Action '" + name + "' has no steps");
            }
            for (const auto& step : steps) {
                // 检查速度、加速度 > 0 等
                if (step.params().vel <= 0.0 || step.params().acc <= 0.0
                    || step.params().tolerance_ori <= 0.0 || step.params().tolerance_pos <= 0.0) {
                    throw std::runtime_error("Invalid motion params in action '" + name + "'");
                }
            }
        }
    }
    static std::unordered_map<std::string, std::vector<Action::Step>> make_parameter_dict() {
        using namespace rmcs_core::controller::arm;

        return {
            // ---------- 简单动作 ----------
            { "gripper_open",     {Action::Step::makeOpenGripper()}                             },

            {"gripper_close",                                 {Action::Step::makeCloseGripper()}},

            {    "auto_walk",
             {Action::Step::makeJoint(
             Action::JointTarget{
             .joint_1 = 0.0,
             .joint_2 = 1.23,
             .joint_3 = -1.36,
             .joint_4 = 0.0,
             .joint_5 = 0.63,
             .joint_6 = 0.0},
             Action::MotionParams{
             .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001})}        },

            {"up_one_stairs",
             {Action::Step::makeJoint(
             Action::JointTarget{
             .joint_1 = 0.0,
             .joint_2 = 0.25,
             .joint_3 = -0.55,
             .joint_4 = 0.0,
             .joint_5 = 0.27,
             .joint_6 = 0.0},
             Action::MotionParams{.vel = 0.03, .acc = 0.03})}                                   },

            {      "initial",
             {Action::Step::makeJoint(
             Action::JointTarget{
             .joint_1 = 0.0,
             .joint_2 = 0.25,
             .joint_3 = -0.55,
             .joint_4 = 0.0,
             .joint_5 = 0.27,
             .joint_6 = 0.0},
             Action::MotionParams{.vel = 0.03, .acc = 0.03})}                                   },

            {"initial_again",
             {Action::Step::makeJoint(
             Action::JointTarget{
             .joint_1 = 0.0,
             .joint_2 = 1.23,
             .joint_3 = -1.36,
             .joint_4 = 0.0,
             .joint_5 = 0.63,
             .joint_6 = 0.0},
             Action::MotionParams{.vel = 0.06, .acc = 0.04})}                                   },

            {   "lift_again",
             {Action::Step::makeJoint(
             Action::JointTarget{
             .joint_1 = 0.0,
             .joint_2 = 1.0,
             .joint_3 = -1.0,
             .joint_4 = 0.0,
             .joint_5 = 0.27,
             .joint_6 = 0.0},
             Action::MotionParams{.vel = 0.03, .acc = 0.03})}                                   },

            // ---------- 组合动作 ----------
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
             .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003}),
             Action::Step::makePose(
             Action::PoseTarget{
             .x     = -0.138798,
             .y     = 0.338888,
             .z     = 0.355177,
             .roll  = -0.714410,
             .pitch = -1.365410,
             .yaw   = 2.684959},
             Action::MotionParams{
             .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.2, .tolerance_ori = 0.003}),
             Action::Step::makePose(
             Action::PoseTarget{
             .x     = -0.326852,
             .y     = 0.177367,
             .z     = 0.345939,
             .roll  = -0.00001,
             .pitch = -1.5708,
             .yaw   = 3.049787},
             Action::MotionParams{
             .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.2, .tolerance_ori = 0.003}),
             Action::Step::makeLinear(
             Action::LinearTarget{
             .dir_x = -1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.16},
             Action::MotionParams{
             .vel = 0.03, .acc = 0.2, .tolerance_pos = 0.2, .tolerance_ori = 0.0}),
             Action::Step::makeOpenGripper(),
             Action::Step::makeLinear(
             Action::LinearTarget{
             .dir_x = 0.0, .dir_y = 0.0, .dir_z = 1.0, .distance = 0.09},
             Action::MotionParams{
             .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.0}),
             Action::Step::makePose(
             Action::PoseTarget{
             .x     = 0.221731,
             .y     = 0.002842,
             .z     = 0.308955,
             .roll  = -3.084697,
             .pitch = -1.097439,
             .yaw   = 3.096949},
             Action::MotionParams{
             .vel = 0.06, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003}),
             }                                                                                  },

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
             .vel = 0.02, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003}),
             Action::Step::makePose(
             Action::PoseTarget{
             .x     = -0.138156,
             .y     = -0.340795,
             .z     = 0.360307,
             .roll  = 0.593263,
             .pitch = -1.490951,
             .yaw   = -2.549442},
             Action::MotionParams{
             .vel = 0.02, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003}),
             Action::Step::makePose(
             Action::PoseTarget{
             .x     = -0.325210,
             .y     = -0.160399,
             .z     = 0.321385,
             .roll  = 1.573944,
             .pitch = -1.570285,
             .yaw   = 1.673210},
             Action::MotionParams{
             .vel = 0.02, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003}),
             Action::Step::makeLinear(
             Action::LinearTarget{
             .dir_x = -1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.15},
             Action::MotionParams{
             .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.0}),
             Action::Step::makeOpenGripper(),
             Action::Step::makeLinear(
             Action::LinearTarget{
             .dir_x = 0.0, .dir_y = 0.0, .dir_z = 1.0, .distance = 0.11},
             Action::MotionParams{
             .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.0}),
             Action::Step::makePose(
             Action::PoseTarget{
             .x     = 0.221731,
             .y     = 0.002842,
             .z     = 0.308955,
             .roll  = -3.084697,
             .pitch = -1.097439,
             .yaw   = 3.096949},
             Action::MotionParams{
             .vel = 0.06, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003}),
             }                                                                                  },

            {   "extract_lb",
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
             .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003}),
             Action::Step::makePose(
             Action::PoseTarget{
             .x     = 0.002843,
             .y     = 0.285341,
             .z     = 0.269419,
             .roll  = -1.974588,
             .pitch = -1.453901,
             .yaw   = -2.682938},
             Action::MotionParams{
             .vel = 0.04, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003}),
             Action::Step::makePose(
             Action::PoseTarget{
             .x     = -0.252779,
             .y     = 0.127352,
             .z     = 0.180452,
             .roll  = 0.0,
             .pitch = -1.571699,
             .yaw   = 2.627386},
             Action::MotionParams{
             .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.2, .tolerance_ori = 0.003}),
             Action::Step::makeLinear(
             Action::LinearTarget{
             .dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.09},
             Action::MotionParams{
             .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.0}),
             Action::Step::makeCloseGripper(),
             Action::Step::makeLinear(
             Action::LinearTarget{
             .dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.15},
             Action::MotionParams{
             .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.0}),
             Action::Step::makePose(
             Action::PoseTarget{
             .x     = 0.221731,
             .y     = 0.002842,
             .z     = 0.308955,
             .roll  = -3.084697,
             .pitch = -1.097439,
             .yaw   = 3.096949},
             Action::MotionParams{
             .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003}),
             }                                                                                  },

            {   "extract_rb",
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
             .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003}),
             Action::Step::makePose(
             Action::PoseTarget{
             .x     = 0.004106,
             .y     = -0.284904,
             .z     = 0.266808,
             .roll  = 1.225331,
             .pitch = -1.497006,
             .yaw   = -2.802148},
             Action::MotionParams{
             .vel = 0.02, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003}),
             Action::Step::makePose(
             Action::PoseTarget{
             .x     = -0.255603,
             .y     = -0.131261,
             .z     = 0.183260,
             .roll  = -2.70866,
             .pitch = -1.568567,
             .yaw   = 0.028266},
             Action::MotionParams{
             .vel = 0.02, .acc = 0.02, .tolerance_pos = 0.2, .tolerance_ori = 0.003}),
             Action::Step::makeLinear(
             Action::LinearTarget{
             .dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.08},
             Action::MotionParams{
             .vel = 0.05, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.0}),
             Action::Step::makeCloseGripper(),
             Action::Step::makeLinear(
             Action::LinearTarget{
             .dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.15},
             Action::MotionParams{
             .vel = 0.06, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.0}),
             Action::Step::makePose(
             Action::PoseTarget{
             .x     = 0.221731,
             .y     = 0.002842,
             .z     = 0.308955,
             .roll  = -3.084697,
             .pitch = -1.097439,
             .yaw   = 3.096949},
             Action::MotionParams{
             .vel = 0.03, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003}),
             }                                                                                  },
        };
    }

    std::unordered_map<std::string, std::vector<Action::Step>> parameter_dict_;
    std::unordered_map<std::string, std::vector<Action::Step>> helper_composed_cache_;
};

} // namespace rmcs_core::controller::arm
