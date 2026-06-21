#pragma once

#include "controller/arm/action_dictionary.hpp"
#include <atomic>
#include <chrono>
#include <cstdint>
#include <geometry_msgs/msg/pose.hpp>
#include <map>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <unordered_map>
#include <variant>
#include <vector>

#include <rmcs_msgs/gripper_mode.hpp>

namespace rmcs_core::controller::arm {

class ActionMachine {
public:
    struct PlannedTrajectory {
        uint64_t request_id{0};
        bool plan_success{false};
        std::map<int, std::vector<std::vector<double>>> step_position_map;
        std::map<int, dictionary::Step> step_map;
    };
    struct PlanRequest {
        uint64_t request_id{0};
        std::vector<dictionary::Step> steps;
    };

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

    ActionMachine& register_task(const std::string& task_name, const std::vector<std::string>& chunk_names) {
        dictionary::action_chunk composed;
        for (auto& name : chunk_names) {
            if (auto* c = dictionary::find_chunk(name))
                composed.insert(composed.end(), c->begin(), c->end());
        }
        if (!composed.empty())
            task_cache_.emplace(task_name, std::move(composed));
        return *this;
    }

    void process(const std::string& name) {
        const dictionary::action_chunk* chunk = dictionary::find_chunk(name); // 1) built-in?
        if (!chunk) {
            auto it = task_cache_.find(name);                                 // 2) composed?
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
    static bool planSingleStep(
        const dictionary::Step& step, moveit::planning_interface::MoveGroupInterface* mg,
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
                if constexpr (std::is_same_v<T, dictionary::JointTarget>) {
                    mg->setJointValueTarget(
                        std::map<std::string, double>{
                            {"joint_1", target.joint_1},
                            {"joint_2", target.joint_2},
                            {"joint_3", target.joint_3},
                            {"joint_4", target.joint_4},
                            {"joint_5", target.joint_5},
                            {"joint_6", target.joint_6},
                    });
                } else if constexpr (std::is_same_v<T, dictionary::PoseTarget>) {
                    geometry_msgs::msg::Pose pose;
                    pose.position.x = target.x;
                    pose.position.y = target.y;
                    pose.position.z = target.z;
                    tf2::Quaternion q;
                    q.setRPY(target.roll, target.pitch, target.yaw);
                    pose.orientation = tf2::toMsg(q);
                    mg->setPoseTarget(pose, "link_6");
                } else if constexpr (std::is_same_v<T, dictionary::LinearTarget>) {
                    const Eigen::Isometry3d& start =
                        current_state->getGlobalLinkTransform("link_6");
                    Eigen::Vector3d dir(target.dir_x, target.dir_y, target.dir_z);
                    dir.normalize();
                    Eigen::Isometry3d T = start;
                    T.translation() += T.linear() * (dir * target.distance);
                    mg->setPoseTarget(tf2::toMsg(T), "link_6");
                } else if constexpr (std::is_same_v<T, dictionary::NoTarget>) {
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

            if (!planSingleStep(
                    step, move_group_.get(), current_state, segment_pts, segment_joint_names)) {
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
        planned_trajectory_.store(result, std::memory_order_release);
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
    std::unordered_map<std::string, dictionary::action_chunk> task_cache_;
};

} // namespace rmcs_core::controller::arm
