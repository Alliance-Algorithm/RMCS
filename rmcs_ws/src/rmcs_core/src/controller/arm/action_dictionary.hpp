#pragma once
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <variant>

#include <rmcs_msgs/gripper_mode.hpp>
namespace rmcs_core::controller::arm::dictionary{
enum class MotionType { Pose, Linear, Joint, OpenGripper, CloseGripper };
enum class GripperState { None, Open, Close };
struct PoseTarget {
    double x, y, z, roll, pitch, yaw;
};
struct JointTarget {
    double joint_1, joint_2, joint_3, joint_4, joint_5, joint_6;
};
struct LinearTarget {
    double dir_x, dir_y, dir_z, distance;
};
// 需要时还可以扩展，如 GripperTarget 为空
struct NoTarget {};
using Target = std::variant<NoTarget,PoseTarget, JointTarget, LinearTarget>;

// ---------- 参数 ----------
struct MotionParams {
    double vel, acc, tolerance_pos, tolerance_ori;
};
struct Step {
    // 规划器配置
    MotionType motion_type;
    std::string pipeline_id = "ompl";
    std::string planner_id= " ";
    // 目标数据
    Target target;

    // 动态参数
    MotionParams params;
};
using action_chunk  = std::vector<Step>;
using ParameterDict = std::unordered_map<std::string, action_chunk>;

inline ParameterDict parameter_dict{

    // ========================================================================
    // Single-step actions / primitives
    // ========================================================================

    {"gripper_open",
     {{Step{
         .motion_type = MotionType::OpenGripper,
         .pipeline_id = "",
         .planner_id  = "",
         .target      = NoTarget{},
         .params      = {},
     }}}},
    {"gripper_close",
     {{Step{
         .motion_type = MotionType::CloseGripper,
         .pipeline_id = "",
         .planner_id  = "",
         .target      = NoTarget{},
         .params      = {},
     }}}},

    {"auto_walk",
     {{Step{
         .motion_type = MotionType::Joint,
         .pipeline_id = "ompl",
         .planner_id  = "",
         .target      = JointTarget{0.0, 1.23, -1.36, 0.0, 0.63, 0.0},
         .params      = MotionParams{.vel = 0.03, .acc = 0.03, .tolerance_pos = 0.001, .tolerance_ori = 0.001},
     }}}},
    {"up_one_stairs",
     {{Step{
         .motion_type = MotionType::Joint,
         .target      = JointTarget{0.0, 0.25, -0.55, 0.0, 0.27, 0.0},
         .params      = MotionParams{.vel = 0.03, .acc = 0.03, .tolerance_pos = 0.0, .tolerance_ori = 0.0},
     }}}},

    // ========================================================================
    // Multi-step tasks
    // ========================================================================

    // ---- Up_Two_Stairs (3 steps) ----
    {"initial",
     {{Step{
         .motion_type = MotionType::Joint,
         .target      = JointTarget{0.0, 0.25, -0.55, 0.0, 0.27, 0.0},
         .params      = MotionParams{.vel = 0.03, .acc = 0.03, .tolerance_pos = 0.0, .tolerance_ori = 0.0},
     }}}},
    {"initial_again",
     {{Step{
         .motion_type = MotionType::Joint,
         .target      = JointTarget{0.0, 1.23, -1.36, 0.0, 0.63, 0.0},
         .params      = MotionParams{.vel = 0.06, .acc = 0.04, .tolerance_pos = 0.0, .tolerance_ori = 0.0},
     }}}},
    {"lift_again",
     {{Step{
         .motion_type = MotionType::Joint,
         .target      = JointTarget{0.0, 1.0, -1.0, 0.0, 0.27, 0.0},
         .params      = MotionParams{.vel = 0.03, .acc = 0.03, .tolerance_pos = 0.0, .tolerance_ori = 0.0},
     }}}},

    // ---- Storage_LB (7 steps: 3 pose + lin_forward + gripper_open + lin_up + pose) ----
    {"storage_lb",
     {{
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{.x = 0.281561, .y = -0.033675, .z = 0.388642, .roll = 0.725640, .pitch = -1.475059, .yaw = -0.839970},
             .params      = MotionParams{.vel = 0.03, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{.x = -0.138798, .y = 0.338888, .z = 0.355177, .roll = -0.714410, .pitch = -1.365410, .yaw = 2.684959},
             .params      = MotionParams{.vel = 0.02, .acc = 0.02, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{.x = -0.326852, .y = 0.177367, .z = 0.345939, .roll = -0.00001, .pitch = -1.5708, .yaw = 3.049787},
             .params      = MotionParams{.vel = 0.02, .acc = 0.02, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
         Step{
             .motion_type = MotionType::Linear,
             .pipeline_id = "pilz_industrial_motion_planner",
             .planner_id  = "LIN",
             .target      = LinearTarget{.dir_x = -1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.16},
             .params      = MotionParams{.vel = 0.03, .acc = 0.2, .tolerance_pos = 0.2, .tolerance_ori = 0.0},
         },
         Step{
             .motion_type = MotionType::OpenGripper,
             .target      = NoTarget{},
             .params      = {},
         },
         Step{
             .motion_type = MotionType::Linear,
             .pipeline_id = "pilz_industrial_motion_planner",
             .planner_id  = "LIN",
             .target      = LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = 1.0, .distance = 0.09},
             .params      = MotionParams{.vel = 0.05, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.0},
         },
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{.x = 0.221731, .y = 0.002842, .z = 0.308955, .roll = -3.084697, .pitch = -1.097439, .yaw = 3.096949},
             .params      = MotionParams{.vel = 0.06, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
     }}},

    // ---- Storage_RB (7 steps: 3 pose + lin_forward + gripper_open + lin_up + pose) ----
    {"storage_rb",
     {{
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{.x = 0.281561, .y = -0.033675, .z = 0.388642, .roll = 0.725640, .pitch = -1.475059, .yaw = -0.839970},
             .params      = MotionParams{.vel = 0.02, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{.x = -0.138156, .y = -0.340795, .z = 0.360307, .roll = 0.593263, .pitch = -1.490951, .yaw = -2.549442},
             .params      = MotionParams{.vel = 0.02, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{.x = -0.325210, .y = -0.160399, .z = 0.321385, .roll = 1.573944, .pitch = -1.570285, .yaw = 1.673210},
             .params      = MotionParams{.vel = 0.02, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
         Step{
             .motion_type = MotionType::Linear,
             .pipeline_id = "pilz_industrial_motion_planner",
             .planner_id  = "LIN",
             .target      = LinearTarget{.dir_x = -1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.15},
             .params      = MotionParams{.vel = 0.03, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.0},
         },
         Step{
             .motion_type = MotionType::OpenGripper,
             .target      = NoTarget{},
             .params      = {},
         },
         Step{
             .motion_type = MotionType::Linear,
             .pipeline_id = "pilz_industrial_motion_planner",
             .planner_id  = "LIN",
             .target      = LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = 1.0, .distance = 0.11},
             .params      = MotionParams{.vel = 0.05, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.0},
         },
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{.x = 0.221731, .y = 0.002842, .z = 0.308955, .roll = -3.084697, .pitch = -1.097439, .yaw = 3.096949},
             .params      = MotionParams{.vel = 0.06, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
     }}},

    // ---- Extract_LB (7 steps: 3 pose + lin_down + gripper_close + lin_out + pose) ----
    {"extract_lb",
     {{
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{.x = 0.281561, .y = -0.033675, .z = 0.388642, .roll = 0.725640, .pitch = -1.475059, .yaw = -0.839970},
             .params      = MotionParams{.vel = 0.05, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{.x = 0.002843, .y = 0.285341, .z = 0.269419, .roll = -1.974588, .pitch = -1.453901, .yaw = -2.682938},
             .params      = MotionParams{.vel = 0.04, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{.x = -0.252779, .y = 0.127352, .z = 0.180452, .roll = 0.0, .pitch = -1.571699, .yaw = 2.627386},
             .params      = MotionParams{.vel = 0.02, .acc = 0.02, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
         Step{
             .motion_type = MotionType::Linear,
             .pipeline_id = "pilz_industrial_motion_planner",
             .planner_id  = "LIN",
             .target      = LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.09},
             .params      = MotionParams{.vel = 0.03, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.0},
         },
         Step{
             .motion_type = MotionType::CloseGripper,
             .target      = NoTarget{},
             .params      = {},
         },
         Step{
             .motion_type = MotionType::Linear,
             .pipeline_id = "pilz_industrial_motion_planner",
             .planner_id  = "LIN",
             .target      = LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.15},
             .params      = MotionParams{.vel = 0.05, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.0},
         },
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{.x = 0.221731, .y = 0.002842, .z = 0.308955, .roll = -3.084697, .pitch = -1.097439, .yaw = 3.096949},
             .params      = MotionParams{.vel = 0.03, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
     }}},

    // ---- Extract_RB (7 steps: 3 pose + lin_down + gripper_close + lin_out + pose) ----
    {"extract_rb",
     {{
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{.x = 0.281561, .y = -0.033675, .z = 0.388642, .roll = 0.725640, .pitch = -1.475059, .yaw = -0.839970},
             .params      = MotionParams{.vel = 0.05, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{.x = 0.004106, .y = -0.284904, .z = 0.266808, .roll = 1.225331, .pitch = -1.497006, .yaw = -2.802148},
             .params      = MotionParams{.vel = 0.02, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{.x = -0.255603, .y = -0.131261, .z = 0.183260, .roll = -2.70866, .pitch = -1.568567, .yaw = 0.028266},
             .params      = MotionParams{.vel = 0.02, .acc = 0.02, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
         Step{
             .motion_type = MotionType::Linear,
             .pipeline_id = "pilz_industrial_motion_planner",
             .planner_id  = "LIN",
             .target      = LinearTarget{.dir_x = 0.0, .dir_y = 0.0, .dir_z = -1.0, .distance = 0.08},
             .params      = MotionParams{.vel = 0.05, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.0},
         },
         Step{
             .motion_type = MotionType::CloseGripper,
             .target      = NoTarget{},
             .params      = {},
         },
         Step{
             .motion_type = MotionType::Linear,
             .pipeline_id = "pilz_industrial_motion_planner",
             .planner_id  = "LIN",
             .target      = LinearTarget{.dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0, .distance = 0.15},
             .params      = MotionParams{.vel = 0.06, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.0},
         },
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{.x = 0.221731, .y = 0.002842, .z = 0.308955, .roll = -3.084697, .pitch = -1.097439, .yaw = 3.096949},
             .params      = MotionParams{.vel = 0.03, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
     }}},
};

[[nodiscard]] inline const action_chunk* find_chunk(const std::string& name) noexcept {
    const auto it = parameter_dict.find(name);
    return it == parameter_dict.end() ? nullptr : &it->second;
}
} // namespace rmcs_core::controller::arm::dictionary
