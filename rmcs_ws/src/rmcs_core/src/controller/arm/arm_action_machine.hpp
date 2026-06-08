#pragma once

#include <array>
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
#include <tf2_eigen/tf2_eigen.hpp>

#include <rmcs_msgs/gripper_mode.hpp>

namespace rmcs_core::controller::arm {

enum class MotionType { Pose, Linear, Joint, OpenGripper, CloseGripper };
enum class GripperState { None, Open, Close };

struct Step {
    MotionType motion_type;
    std::array<double, 6> target{}; // Pose/Joint: absolute xyzrpy or joint angles
                                    // Linear: target[0..2]=local dir, target[3]=distance
    std::array<double, 4> params{}; // vel, acc, tole_p, tole_rpy
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

    { "gripper_open",
     {Step{
     MotionType::OpenGripper,
     {},
     {},
     }}},
    {"gripper_close",
     {Step{
     MotionType::CloseGripper,
     {},
     {},
     }}},

    {    "auto_walk",
     {Step{
     MotionType::Joint,
     {0.0, 1.23, -1.36, 0.0, 0.63, 0.0},
     {0.03, 0.03, 0.0, 0.0},
     }}},
    {"up_one_stairs",
     {Step{
     MotionType::Joint,
     {0.0, 0.25, -0.55, 0.0, 0.27, 0.0},
     {0.03, 0.03, 0.0, 0.0},
     }}},

    // ========================================================================
    // Multi-step tasks
    // ========================================================================

    // ---- Up_Two_Stairs (3 steps) ----
    {"initial",
     {Step{
     MotionType::Joint,
     {0.0, 0.25, -0.55, 0.0, 0.27, 0.0},
     {0.03, 0.03, 0.0, 0.0},
     }}},
    {"initial_again",
     {Step{
     MotionType::Joint,
     {0.0, 1.23, -1.36, 0.0, 0.63, 0.0},
     {0.06, 0.04, 0.0, 0.0},
     }}},
    {   "lift_again",
     {Step{
     MotionType::Joint,
     {0.0, 1.0, -1.0, 0.0, 0.27, 0.0},
     {0.03, 0.03, 0.0, 0.0},
     }}},

    // ---- Storage_LB (7 steps: 3 pose + lin_forward + gripper_open + lin_up + pose) ----
    {   "storage_lb",
     {
     Step{
     MotionType::Pose,
     {0.281561, -0.033675, 0.388642, 0.725640, -1.475059, -0.839970},
     {0.03, 0.03, 0.2, 0.003},
     },
     Step{
     MotionType::Pose,
     {-0.138798, 0.338888, 0.355177, -0.714410, -1.365410, 2.684959},
     {0.02, 0.02, 0.2, 0.003},
     },
     Step{
     MotionType::Pose,
     {-0.326852, 0.177367, 0.345939, -0.00001, -1.5708, 3.049787},
     {0.02, 0.02, 0.2, 0.003},
     },
     Step{
     MotionType::Linear,
     {-1.0, 0.0, 0.0, 0.16, 0.0, 0.0}, // dir(-x) + distance
     {0.03, 0.2, 0.2, 0.0},
     },
     Step{
     MotionType::OpenGripper,
     {},
     {},
     },
     Step{
     MotionType::Linear,
     {0.0, 0.0, 1.0, 0.09, 0.0, 0.0},  // dir(+z) + distance
     {0.05, 0.03, 0.2, 0.0},
     },
     Step{
     MotionType::Pose,
     {0.221731, 0.002842, 0.308955, -3.084697, -1.097439, 3.096949},
     {0.06, 0.03, 0.2, 0.003},
     },
     } },

    // ---- Storage_RB (7 steps: 3 pose + lin_forward + gripper_open + lin_up + pose) ----
    {   "storage_rb",
     {
     Step{
     MotionType::Pose,
     {0.281561, -0.033675, 0.388642, 0.725640, -1.475059, -0.839970},
     {0.02, 0.03, 0.2, 0.003},
     },
     Step{
     MotionType::Pose,
     {-0.138156, -0.340795, 0.360307, 0.593263, -1.490951, -2.549442},
     {0.02, 0.03, 0.2, 0.003},
     },
     Step{
     MotionType::Pose,
     {-0.325210, -0.160399, 0.321385, 1.573944, -1.570285, 1.673210},
     {0.02, 0.03, 0.2, 0.003},
     },
     Step{
     MotionType::Linear,
     {-1.0, 0.0, 0.0, 0.15, 0.0, 0.0}, // dir(-x) + distance
     {0.03, 0.03, 0.2, 0.0},
     },
     Step{
     MotionType::OpenGripper,
     {},
     {},
     },
     Step{
     MotionType::Linear,
     {0.0, 0.0, 1.0, 0.11, 0.0, 0.0},  // dir(+z) + distance
     {0.05, 0.03, 0.2, 0.0},
     },
     Step{
     MotionType::Pose,
     {0.221731, 0.002842, 0.308955, -3.084697, -1.097439, 3.096949},
     {0.06, 0.03, 0.2, 0.003},
     },
     } },

    // ---- Extract_LB (7 steps: 3 pose + lin_down + gripper_close + lin_out + pose) ----
    {   "extract_lb",
     {
     Step{
     MotionType::Pose,
     {0.281561, -0.033675, 0.388642, 0.725640, -1.475059, -0.839970},
     {0.05, 0.03, 0.2, 0.003},
     },
     Step{
     MotionType::Pose,
     {0.002843, 0.285341, 0.269419, -1.974588, -1.453901, -2.682938},
     {0.04, 0.03, 0.2, 0.003},
     },
     Step{
     MotionType::Pose,
     {-0.252779, 0.127352, 0.180452, 0.0, -1.571699, 2.627386},
     {0.02, 0.02, 0.2, 0.003},
     },
     Step{
     MotionType::Linear,
     {0.0, 0.0, -1.0, 0.09, 0.0, 0.0}, // dir(-z) + distance
     {0.03, 0.03, 0.2, 0.0},
     },
     Step{
     MotionType::CloseGripper,
     {},
     {},
     },
     Step{
     MotionType::Linear,
     {1.0, 0.0, 0.0, 0.15, 0.0, 0.0},  // dir(+x) + distance
     {0.05, 0.03, 0.2, 0.0},
     },
     Step{
     MotionType::Pose,
     {0.221731, 0.002842, 0.308955, -3.084697, -1.097439, 3.096949},
     {0.03, 0.03, 0.2, 0.003},
     },
     } },

    // ---- Extract_RB (7 steps: 3 pose + lin_down + gripper_close + lin_out + pose) ----
    {   "extract_rb",
     {
     Step{
     MotionType::Pose,
     {0.281561, -0.033675, 0.388642, 0.725640, -1.475059, -0.839970},
     {0.05, 0.03, 0.2, 0.003},
     },
     Step{
     MotionType::Pose,
     {0.004106, -0.284904, 0.266808, 1.225331, -1.497006, -2.802148},
     {0.02, 0.03, 0.2, 0.003},
     },
     Step{
     MotionType::Pose,
     {-0.255603, -0.131261, 0.183260, -2.70866, -1.568567, 0.028266},
     {0.02, 0.02, 0.2, 0.003},
     },
     Step{
     MotionType::Linear,
     {0.0, 0.0, -1.0, 0.08, 0.0, 0.0}, // dir(-z) + distance
     {0.05, 0.03, 0.2, 0.0},
     },
     Step{
     MotionType::CloseGripper,
     {},
     {},
     },
     Step{
     MotionType::Linear,
     {1.0, 0.0, 0.0, 0.15, 0.0, 0.0},  // dir(+x) + distance
     {0.06, 0.03, 0.2, 0.0},
     },
     Step{
     MotionType::Pose,
     {0.221731, 0.002842, 0.308955, -3.084697, -1.097439, 3.096949},
     {0.03, 0.03, 0.2, 0.003},
     },
     } },
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

    void moveit_loop() {
        const auto linear_point_transformer = [](const geometry_msgs::msg::Pose& start_pose,
                                                 const Eigen::Vector3d& local_dir,
                                                 double distance) {
            Eigen::Isometry3d T;
            tf2::fromMsg(start_pose, T);
            Eigen::Vector3d p_local = local_dir.normalized() * distance;
            T.translation() += T.linear() * p_local;
            return tf2::toMsg(T);
        };

        const auto xyzrpy_to_pose = [](const std::array<double, 6>& xyzrpy) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = xyzrpy[0];
            pose.position.y = xyzrpy[1];
            pose.position.z = xyzrpy[2];

            tf2::Quaternion q;
            q.setRPY(xyzrpy[3], xyzrpy[4], xyzrpy[5]);
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();
            return pose;
        };
        const auto request = plan_request_.load(std::memory_order_acquire);
        if (!request || request->request_id == last_planned_id_)
            return;
        auto result          = std::make_shared<PlannedTrajectory>();
        result->request_id   = request->request_id;
        result->plan_success = true;

        auto current_state = move_group_->getCurrentState();

        for (size_t i = 0; i < request->steps.size(); ++i) {
            const auto& step = request->steps[i];

            move_group_->clearPoseTargets();
            move_group_->clearPathConstraints();
            move_group_->setStartState(*current_state);
            move_group_->setPlanningTime(5.0);
            move_group_->setMaxVelocityScalingFactor(step.params[0]);
            move_group_->setMaxAccelerationScalingFactor(step.params[1]);

            std::vector<trajectory_msgs::msg::JointTrajectoryPoint> segment_pts;
            std::vector<std::string> segment_joint_names;
            bool segment_failed = false;

            switch (step.motion_type) {

            case MotionType::Joint: {
                move_group_->setPlanningPipelineId("ompl");
                move_group_->setPlannerId("");
                move_group_->setJointValueTarget(
                    std::map<std::string, double>{
                        {"joint_1", step.target[0]},
                        {"joint_2", step.target[1]},
                        {"joint_3", step.target[2]},
                        {"joint_4", step.target[3]},
                        {"joint_5", step.target[4]},
                        {"joint_6", step.target[5]},
                });
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_WARN(node_->get_logger(), "segment %zu plan failed", i);
                    segment_failed = true;
                    break;
                }
                segment_pts         = plan.trajectory.joint_trajectory.points;
                segment_joint_names = plan.trajectory.joint_trajectory.joint_names;
                break;
            }

            case MotionType::Pose: {
                move_group_->setPlanningPipelineId("ompl");
                move_group_->setPlannerId("");
                move_group_->setGoalOrientationTolerance(step.params[2]);
                move_group_->setGoalPositionTolerance(step.params[3]);
                move_group_->setPoseTarget(xyzrpy_to_pose(step.target), "link_6");
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_WARN(node_->get_logger(), "segment %zu plan failed", i);
                    segment_failed = true;
                    break;
                }
                segment_pts         = plan.trajectory.joint_trajectory.points;
                segment_joint_names = plan.trajectory.joint_trajectory.joint_names;
                break;
            }

            case MotionType::Linear: {
                // target[0..2] = local direction, target[3] = distance
                const Eigen::Isometry3d& start_state =
                    current_state->getGlobalLinkTransform("link_6");
                auto target_pose = linear_point_transformer(
                    tf2::toMsg(start_state), {step.target[0], step.target[1], step.target[2]},
                    step.target[3]);
                move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
                move_group_->setPlannerId("LIN");
                move_group_->setGoalOrientationTolerance(step.params[2]);
                move_group_->setGoalPositionTolerance(step.params[3]);
                move_group_->setPoseTarget(target_pose, "link_6");

                moveit::planning_interface::MoveGroupInterface::Plan plan;
                if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_WARN(node_->get_logger(), "segment %zu plan failed", i);
                    segment_failed = true;
                    break;
                }
                segment_pts         = plan.trajectory.joint_trajectory.points;
                segment_joint_names = plan.trajectory.joint_trajectory.joint_names;
                break;
            }

            default: {
                segment_joint_names = move_group_->getJointNames();
                std::vector<double> q(segment_joint_names.size());
                for (size_t k = 0; k < segment_joint_names.size(); ++k)
                    q[k] = current_state->getVariablePosition(segment_joint_names[k]);

                segment_pts.resize(500);
                for (auto& pt : segment_pts)
                    pt.positions = q;
                break;
            }
            }
            if (segment_failed) {
                result->plan_success = false;
                break; // exit for loop
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
