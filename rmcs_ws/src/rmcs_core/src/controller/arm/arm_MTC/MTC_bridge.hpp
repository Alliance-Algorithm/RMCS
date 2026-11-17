#pragma once

#include "controller/arm/arm_MTC/MTC_step.hpp"

#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/storage.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <rclcpp/duration.hpp>

#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace rmcs_core::controller::arm::MTC {

struct MotionSegment {
    std::string group;
    moveit_msgs::msg::RobotTrajectory trajectory;
};

struct SceneUpdate {
    std::vector<SceneOperation> operations;
};

struct ExecutionStep {
    std::string stage_path;
    std::variant<MotionSegment, GripperCommandConfig, SceneUpdate> action;
};

struct ExecutionPlan {
    std::string task_name;
    double cost = 0.0;
    moveit_msgs::msg::PlanningScene start_scene;
    std::vector<ExecutionStep> steps;
};

namespace detail {

struct StageMetadata {
    std::string path;
    StageConfig config;
    std::string group;
};

using StageMetadataMap = std::unordered_map<const moveit::task_constructor::Stage*, StageMetadata>;

class PlanBridge {
public:
    static ExecutionPlan convert(
        const std::string& task_name, const moveit::task_constructor::SolutionBase& solution,
        const StageMetadataMap& metadata) {
        ExecutionPlan plan;
        plan.task_name = task_name;
        plan.cost = solution.cost();
        append_solution(solution, metadata, plan);
        moveit_task_constructor_msgs::msg::Solution serialized;
        solution.toMsg(serialized);
        plan.start_scene = std::move(serialized.start_scene);
        return plan;
    }

private:
    static void append_solution(
        const moveit::task_constructor::SolutionBase& solution, const StageMetadataMap& metadata,
        ExecutionPlan& plan) {
        namespace mtc = moveit::task_constructor;
        if (const auto* sequence = dynamic_cast<const mtc::SolutionSequence*>(&solution)) {
            for (const auto* child : sequence->solutions())
                append_solution(*child, metadata, plan);
            return;
        }
        if (const auto* wrapped = dynamic_cast<const mtc::WrappedSolution*>(&solution)) {
            append_solution(*wrapped->wrapped(), metadata, plan);
            return;
        }
        const auto& trajectory = dynamic_cast<const mtc::SubTrajectory&>(solution);
        const auto& stage = metadata.at(solution.creator());
        std::visit(
            overloaded{
                [&](const MoveToConfig&) { append_motion(stage, trajectory, plan); },
                [&](const MoveRelativeConfig&) { append_motion(stage, trajectory, plan); },
                [&](const GripperCommandConfig& command) {
                    plan.steps.push_back({stage.path, command});
                },
                [&](const ModifyPlanningSceneConfig& scene) {
                    plan.steps.push_back({stage.path, SceneUpdate{scene.operations}});
                },
                [](const auto&) {}, // CurrentState, FixedState and LogPose export nothing.
            },
            stage.config.config);
    }

    static void append_motion(
        const StageMetadata& stage, const moveit::task_constructor::SubTrajectory& trajectory,
        ExecutionPlan& plan) {
        MotionSegment motion;
        motion.group = stage.group;
        trajectory.trajectory()->getRobotTrajectoryMsg(motion.trajectory);
        plan.steps.push_back({stage.path, std::move(motion)});
    }
};

} // namespace detail

inline std::string describe_execution_plan(const ExecutionPlan& plan) {
    std::ostringstream out;
    out << "task=" << plan.task_name << " cost=" << plan.cost << " steps=" << plan.steps.size()
        << '\n';
    for (std::size_t index = 0; index < plan.steps.size(); ++index) {
        const auto& step = plan.steps[index];
        out << index << " " << step.stage_path;
        std::visit(
            overloaded{
                [&](const MotionSegment& motion) {
                    const auto& joints = motion.trajectory.joint_trajectory;
                    out << " motion group=" << motion.group << " joints=";
                    for (const auto& name : joints.joint_names)
                        out << name << ' ';
                    out << "points=" << joints.points.size() << " duration="
                        << (joints.points.empty()
                                ? 0.0
                                : rclcpp::Duration(joints.points.back().time_from_start).seconds());
                },
                [&](const GripperCommandConfig& command) {
                    out << " gripper=" << (command.action == GripperAction::Open ? "open" : "close")
                        << " timeout=" << command.timeout;
                },
                [&](const SceneUpdate& update) {
                    out << " scene operations=" << update.operations.size();
                    for (const auto& operation : update.operations)
                        std::visit(
                            overloaded{
                                [&](const AllowCollisionsConfig& op) {
                                    out << (op.allow ? " allow " : " forbid ") << op.object
                                        << " links=";
                                    for (const auto& link : op.links)
                                        out << link << ' ';
                                },
                                [&](const ObjectCollisionsConfig& op) {
                                    out << (op.allow ? " allow " : " forbid ") << op.object
                                        << " other_object=" << op.other_object;
                                },
                                [&](const AttachObjectConfig& op) {
                                    out << " attach " << op.object << " link=" << op.link;
                                },
                                [&](const DetachObjectConfig& op) {
                                    out << " detach " << op.object << " link=" << op.link;
                                },
                            },
                            operation);
                },
            },
            step.action);
        out << '\n';
    }
    return out.str();
}

} // namespace rmcs_core::controller::arm::MTC
