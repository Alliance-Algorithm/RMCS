#pragma once
#include "controller/arm/arm_action/action_step.hpp"
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
#include <tuple>
#include <variant>
#include <vector>

namespace rmcs_core::controller::arm {

class ActionMachine {
public:
    struct PlannedTrajectory {
        uint64_t request_id{0};
        bool plan_success{false};
        std::map<int, std::vector<std::vector<double>>> step_position_map;
        std::map<int, Action::Step> step_map;
    };

    struct PlanRequest {
        uint64_t request_id{0};
        std::vector<Action::Step> steps;
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

    void process(const std::vector<Action::Step>& steps_) {

        auto next          = std::make_shared<PlanRequest>();
        const auto current = plan_request_.load(std::memory_order_acquire);
        next->request_id   = current ? current->request_id + 1 : 1;
        next->steps        = steps_;
        plan_request_.store(next, std::memory_order_release);
    }

    std::shared_ptr<const PlannedTrajectory> get_trajectory() const {
        return planned_trajectory_.load(std::memory_order_acquire);
    }

private:
    bool planSingleStep(
        const Action::Step& step, moveit::planning_interface::MoveGroupInterface* move_group,
        const moveit::core::RobotStatePtr& current_state,
        std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& out_points,
        std::vector<std::string>& out_joint_names) {
        bool needs_plan = true;
        std::visit(
            [&](const auto& target) {
                using T = std::decay_t<decltype(target)>;
                if constexpr (std::is_same_v<T, Action::JointTarget>) {
                    move_group->setJointValueTarget(
                        std::map<std::string, double>{
                            {"joint_1", target.joint_1},
                            {"joint_2", target.joint_2},
                            {"joint_3", target.joint_3},
                            {"joint_4", target.joint_4},
                            {"joint_5", target.joint_5},
                            {"joint_6", target.joint_6},
                    });
                } else if constexpr (std::is_same_v<T, Action::PoseTarget>) {
                    move_group->setPoseTarget(
                        geometry_msgs::msg::Pose()
                            .set__position(
                                geometry_msgs::msg::Point()
                                    .set__x(target.x)
                                    .set__y(target.y)
                                    .set__z(target.z))
                            .set__orientation([&] {
                                tf2::Quaternion q;
                                q.setRPY(target.roll, target.pitch, target.yaw);
                                return geometry_msgs::msg::Quaternion()
                                    .set__x(q.x())
                                    .set__y(q.y())
                                    .set__z(q.z())
                                    .set__w(q.w());
                            }()),
                        "link_6");
                } else if constexpr (std::is_same_v<T, Action::LinearTarget>) {
                    const Eigen::Isometry3d& start =
                        current_state->getGlobalLinkTransform("link_6");
                    Eigen::Vector3d dir(target.dir_x, target.dir_y, target.dir_z);
                    dir.normalize();
                    Eigen::Isometry3d T = start;
                    T.translation() += T.linear() * (dir * target.distance);
                    move_group->setPoseTarget(tf2::toMsg(T), "link_6");
                } else if constexpr (std::is_same_v<T, Action::NoTarget>) {
                    out_joint_names = move_group->getJointNames();
                    trajectory_msgs::msg::JointTrajectoryPoint point;
                    current_state->copyJointGroupPositions(move_group->getName(), point.positions);
                    out_points.assign(500, point);
                    needs_plan = false;
                    double roll, pitch, yaw;
                    auto pose = move_group->getCurrentPose("link_6");
                    tf2::Matrix3x3(
                        tf2::Quaternion(
                            pose.pose.orientation.x, pose.pose.orientation.y,
                            pose.pose.orientation.z, pose.pose.orientation.w))
                        .getRPY(roll, pitch, yaw); // 直接赋值给变量

                    RCLCPP_INFO(
                        node_->get_logger(),
                        "Current pose: x=%.3f y=%.3f z=%.3f  roll=%.3f pitch=%.3f yaw=%.3f",
                        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, roll,
                        pitch, yaw);
                }
            },
            step.target());

        if (!needs_plan)
            return true;

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
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

            move_group_->clearPoseTargets();
            move_group_->clearPathConstraints();

            move_group_->setStartState(*current_state);
            move_group_->setPlanningTime(5.0);
            move_group_->setMaxVelocityScalingFactor(step.params().vel);
            move_group_->setMaxAccelerationScalingFactor(step.params().acc);
            move_group_->setGoalOrientationTolerance(step.params().tolerance_ori);
            move_group_->setGoalPositionTolerance(step.params().tolerance_pos);
            move_group_->setPlanningPipelineId(step.pipelineId());
            move_group_->setPlannerId(step.plannerId());
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
            RCLCPP_INFO(node_->get_logger(), "segment %zu plan success", i);

            const size_t start_idx = (i == 0) ? 0 : 1;
            auto& step_pos         = result->step_position_map[static_cast<int>(i)];
            for (size_t j = start_idx; j < segment_pts.size(); ++j) {
                step_pos.push_back(segment_pts[j].positions);
            }

            result->step_map.emplace(static_cast<int>(i), step);

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
};

} // namespace rmcs_core::controller::arm
