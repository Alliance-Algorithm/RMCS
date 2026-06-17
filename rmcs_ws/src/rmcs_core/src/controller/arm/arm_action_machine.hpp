#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <thread>
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

namespace rmcs_core::controller::arm {

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
    std::string planner_id;
    // 目标数据
    Target target;

    // 动态参数
    MotionParams params;
};
using action_chunk  = std::vector<Step>;
using ParameterDict = std::unordered_map<std::string, action_chunk>;

struct PlanRequest {
    uint64_t request_id{0};
    std::vector<Step> steps;
};

struct PlannedTrajectory {
    uint64_t request_id{0};
    bool plan_success{false};
    // step_idx → all positions for that step (segments stored independently)
    std::map<int, std::vector<std::vector<double>>> step_position_map;
    // step_idx → Step definition (for gripper/condition checks at boundaries)
    std::map<int, Step> step_map;
};


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
             .target      = PoseTarget{0.281561, -0.033675, 0.388642, 0.725640, -1.475059, -0.839970},
             .params      = MotionParams{.vel = 0.03, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{-0.138798, 0.338888, 0.355177, -0.714410, -1.365410, 2.684959},
             .params      = MotionParams{.vel = 0.02, .acc = 0.02, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{-0.326852, 0.177367, 0.345939, -0.00001, -1.5708, 3.049787},
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
             .target      = PoseTarget{0.221731, 0.002842, 0.308955, -3.084697, -1.097439, 3.096949},
             .params      = MotionParams{.vel = 0.06, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
     }}},

    // ---- Storage_RB (7 steps: 3 pose + lin_forward + gripper_open + lin_up + pose) ----
    {"storage_rb",
     {{
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{0.281561, -0.033675, 0.388642, 0.725640, -1.475059, -0.839970},
             .params      = MotionParams{.vel = 0.02, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{-0.138156, -0.340795, 0.360307, 0.593263, -1.490951, -2.549442},
             .params      = MotionParams{.vel = 0.02, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{-0.325210, -0.160399, 0.321385, 1.573944, -1.570285, 1.673210},
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
             .target      = PoseTarget{0.221731, 0.002842, 0.308955, -3.084697, -1.097439, 3.096949},
             .params      = MotionParams{.vel = 0.06, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
     }}},

    // ---- Extract_LB (7 steps: 3 pose + lin_down + gripper_close + lin_out + pose) ----
    {"extract_lb",
     {{
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{0.281561, -0.033675, 0.388642, 0.725640, -1.475059, -0.839970},
             .params      = MotionParams{.vel = 0.05, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{0.002843, 0.285341, 0.269419, -1.974588, -1.453901, -2.682938},
             .params      = MotionParams{.vel = 0.04, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{-0.252779, 0.127352, 0.180452, 0.0, -1.571699, 2.627386},
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
             .target      = PoseTarget{0.221731, 0.002842, 0.308955, -3.084697, -1.097439, 3.096949},
             .params      = MotionParams{.vel = 0.03, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
     }}},

    // ---- Extract_RB (7 steps: 3 pose + lin_down + gripper_close + lin_out + pose) ----
    {"extract_rb",
     {{
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{0.281561, -0.033675, 0.388642, 0.725640, -1.475059, -0.839970},
             .params      = MotionParams{.vel = 0.05, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{0.004106, -0.284904, 0.266808, 1.225331, -1.497006, -2.802148},
             .params      = MotionParams{.vel = 0.02, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
         Step{
             .motion_type = MotionType::Pose,
             .target      = PoseTarget{-0.255603, -0.131261, 0.183260, -2.70866, -1.568567, 0.028266},
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
             .target      = PoseTarget{0.221731, 0.002842, 0.308955, -3.084697, -1.097439, 3.096949},
             .params      = MotionParams{.vel = 0.03, .acc = 0.03, .tolerance_pos = 0.2, .tolerance_ori = 0.003},
         },
     }}},
};

[[nodiscard]] inline const action_chunk* find_chunk(const std::string& name) noexcept {
    const auto it = parameter_dict.find(name);
    return it == parameter_dict.end() ? nullptr : &it->second;
}

class ActionMachine {
public:
    ActionMachine()
        : node_(
              std::make_shared<rclcpp::Node>(
                  "arm_moveit_planner",
                  rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)))
        , move_group_(
              std::make_unique<moveit::planning_interface::MoveGroupInterface>(
                  node_, "alliance_arm")) {
        move_group_->startStateMonitor();

        exec_.add_node(node_);
        spin_thread_ = std::thread([this] { exec_.spin(); });

        running_.store(true, std::memory_order_release);
        moveit_thread_ = std::thread([this] {
            while (running_.load(std::memory_order_acquire)) {
                moveit_loop();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
    }

    ~ActionMachine() {
        running_.store(false, std::memory_order_release);
        exec_.cancel();
        if (moveit_thread_.joinable())
            moveit_thread_.join();
        if (spin_thread_.joinable())
            spin_thread_.join();
    }

    ActionMachine(const ActionMachine&)            = delete;
    ActionMachine& operator=(const ActionMachine&) = delete;
    ActionMachine(ActionMachine&&)                 = delete;
    ActionMachine& operator=(ActionMachine&&)      = delete;

    ActionMachine& load(const std::string& task_name, const std::vector<std::string>& chunk_names) {
        action_chunk composed;
        for (auto& name : chunk_names) {
            if (auto* c = find_chunk(name))
                composed.insert(composed.end(), c->begin(), c->end());
        }
        if (!composed.empty())
            task_cache_.emplace(task_name, std::move(composed));
        return *this;
    }

    void process(const std::string& name) {
        const action_chunk* chunk = find_chunk(name); // 1) built-in?
        if (!chunk) {
            auto it = task_cache_.find(name);         // 2) composed?
            if (it != task_cache_.end())
                chunk = &it->second;
        }
        if (!chunk || chunk->empty())
            return;

        auto next          = std::make_shared<PlanRequest>();
        const auto current = plan_request_.load(std::memory_order_acquire);
        next->request_id   = current ? current->request_id + 1 : 1;
        next->steps        = *chunk;
        plan_request_.store(next, std::memory_order_release);
    }

    std::shared_ptr<const PlannedTrajectory> get_trajectory() const {
        return planned_trajectory_.load(std::memory_order_acquire);
    }

private:

    static bool planSingleStep(const Step& step,
                        moveit::planning_interface::MoveGroupInterface* mg,
                        const moveit::core::RobotStatePtr& current_state,
                        std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& out_points,
                        std::vector<std::string>& out_joint_names) {
        // 通用设置
        mg->clearPoseTargets();
        mg->clearPathConstraints();
        mg->setStartState(*current_state);
        mg->setPlanningTime(5.0);
        mg->setMaxVelocityScalingFactor(step.params.vel);
        mg->setMaxAccelerationScalingFactor(step.params.acc);
        mg->setGoalOrientationTolerance(step.params.tolerance_ori);
        mg->setGoalPositionTolerance(step.params.tolerance_pos);

        mg->setPlanningPipelineId(step.pipeline_id);
        mg->setPlannerId(step.planner_id);

        bool needs_plan = true;

        std::visit(
            [&](const auto& target) {
                using T = std::decay_t<decltype(target)>;
                if constexpr (std::is_same_v<T, JointTarget>) {
                    mg->setJointValueTarget(
                        std::map<std::string, double>{
                            {"joint_1", target.joint_1},
                            {"joint_2", target.joint_2},
                            {"joint_3", target.joint_3},
                            {"joint_4", target.joint_4},
                            {"joint_5", target.joint_5},
                            {"joint_6", target.joint_6},
                        });
                } else if constexpr (std::is_same_v<T, PoseTarget>) {
                    geometry_msgs::msg::Pose pose;
                    pose.position.x = target.x;
                    pose.position.y = target.y;
                    pose.position.z = target.z;
                    tf2::Quaternion q;
                    q.setRPY(target.roll, target.pitch, target.yaw);
                    pose.orientation = tf2::toMsg(q);
                    mg->setPoseTarget(pose, "link_6");
                } else if constexpr (std::is_same_v<T, LinearTarget>) {
                    const Eigen::Isometry3d& start =
                        current_state->getGlobalLinkTransform("link_6");
                    Eigen::Vector3d dir(target.dir_x, target.dir_y, target.dir_z);
                    dir.normalize();
                    Eigen::Isometry3d T = start;
                    T.translation() += T.linear() * (dir * target.distance);
                    mg->setPoseTarget(tf2::toMsg(T), "link_6");
                } else if constexpr (std::is_same_v<T, NoTarget>) {
                    // 抓取器：生成 500 个保持位置的点，不需要调用 plan()
                    out_joint_names = mg->getJointNames();
                    std::vector<double> q(out_joint_names.size());
                    for (size_t k = 0; k < out_joint_names.size(); ++k)
                        q[k] = current_state->getVariablePosition(out_joint_names[k]);
                    out_points.resize(500);
                    for (auto& pt : out_points)
                        pt.positions = q;
                    needs_plan = false;
                }
            },
            step.target);

        if (!needs_plan)
            return true;

        // 规划
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (mg->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
            return false;

        out_points      = plan.trajectory.joint_trajectory.points;
        out_joint_names = plan.trajectory.joint_trajectory.joint_names;
        return true;
    }

    void moveit_loop() {
        const auto request = plan_request_.load(std::memory_order_acquire);
        if (!request || request->request_id == last_planned_id_)
            return;

        auto result          = std::make_shared<PlannedTrajectory>();
        result->request_id   = request->request_id;
        result->plan_success = true;

        auto current_state = move_group_->getCurrentState();

        for (size_t i = 0; i < request->steps.size(); ++i) {
            const auto& step = request->steps[i];

            std::vector<trajectory_msgs::msg::JointTrajectoryPoint> segment_pts;
            std::vector<std::string> segment_joint_names;

            if (!planSingleStep(step, move_group_.get(), current_state,
                                segment_pts, segment_joint_names)) {
                RCLCPP_WARN(node_->get_logger(), "segment %zu plan failed", i);
                result->plan_success = false;
                break;
            }

            if (segment_pts.empty()) {
                RCLCPP_WARN(node_->get_logger(), "segment %zu empty", i);
                result->plan_success = false;
                break;
            }

            const size_t start_idx = (i == 0) ? 0 : 1;
            auto& step_pos         = result->step_position_map[static_cast<int>(i)];
            for (size_t j = start_idx; j < segment_pts.size(); ++j)
                step_pos.push_back(segment_pts[j].positions);

            result->step_map[static_cast<int>(i)] = step;

            auto next_state = std::make_shared<moveit::core::RobotState>(*current_state);
            next_state->setVariablePositions(segment_joint_names, segment_pts.back().positions);
            next_state->update();
            current_state = next_state;
        }

        last_planned_id_ = request->request_id;
        planned_trajectory_.store(result, std::memory_order::release);
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::MultiThreadedExecutor exec_;
    std::thread spin_thread_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    std::atomic_bool running_{false};
    std::thread moveit_thread_;

    std::atomic<std::shared_ptr<const PlanRequest>> plan_request_{nullptr};
    std::atomic<std::shared_ptr<const PlannedTrajectory>> planned_trajectory_{nullptr};
    uint64_t last_planned_id_{0};
    std::unordered_map<std::string, action_chunk> task_cache_;
};

} // namespace rmcs_core::controller::arm
