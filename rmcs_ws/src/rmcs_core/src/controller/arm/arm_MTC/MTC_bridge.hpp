#pragma once

#include "controller/arm/arm_MTC/MTC_step.hpp"

#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/storage.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
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
        const auto source = metadata.find(solution.creator());
        const auto path = source != metadata.end() ? source->second.path
                        : solution.creator() ? plan.task_name + "/" + solution.creator()->name()
                                             : plan.task_name;
        const auto fail = [&](const std::string& reason) {
            throw std::runtime_error(path + ": " + reason);
        };
        if (solution.isFailure())
            fail("cannot export a failed solution");
        if (const auto* sequence = dynamic_cast<const mtc::SolutionSequence*>(&solution)) {
            for (const auto* child : sequence->solutions()) {
                if (!child)
                    fail("null subsolution");
                append_solution(*child, metadata, plan);
            }
            return;
        }
        if (const auto* wrapped = dynamic_cast<const mtc::WrappedSolution*>(&solution)) {
            if (!wrapped->wrapped())
                fail("null wrapped solution");
            append_solution(*wrapped->wrapped(), metadata, plan);
            return;
        }
        const auto* trajectory = dynamic_cast<const mtc::SubTrajectory*>(&solution);
        if (!trajectory)
            fail("unsupported solution type");
        if (source == metadata.end())
            fail("missing stage metadata");
        const auto& stage = source->second;
        std::visit(
            [&](const auto& config) {
                using Config = std::decay_t<decltype(config)>;
                if constexpr (
                    std::is_same_v<Config, MoveToConfig>
                    || std::is_same_v<Config, MoveRelativeConfig>) {
                    if (!trajectory->trajectory() || trajectory->trajectory()->empty())
                        fail("motion stage has no trajectory");
                    MotionSegment motion;
                    motion.group = stage.group;
                    trajectory->trajectory()->getRobotTrajectoryMsg(motion.trajectory);
                    plan.steps.push_back({stage.path, std::move(motion)});
                } else if constexpr (std::is_same_v<Config, GripperCommandConfig>) {
                    plan.steps.push_back({stage.path, config});
                } else if constexpr (std::is_same_v<Config, ModifyPlanningSceneConfig>) {
                    plan.steps.push_back({stage.path, SceneUpdate{config.operations}});
                } else {
                    static_assert(
                        std::is_same_v<Config, CurrentStateConfig>
                        || std::is_same_v<Config, FixedStateConfig>);
                }
            },
            stage.config.config);
    }
};

} // namespace detail

inline std::string describe_execution_plan(const ExecutionPlan& plan) {
    std::ostringstream description;
    description << "task=" << plan.task_name << " cost=" << plan.cost
                << " steps=" << plan.steps.size() << '\n';
    for (std::size_t index = 0; index < plan.steps.size(); ++index) {
        const auto& step = plan.steps[index];
        description << index << " " << step.stage_path;
        std::visit(
            [&](const auto& action) {
                using Action = std::decay_t<decltype(action)>;
                if constexpr (std::is_same_v<Action, MotionSegment>) {
                    const auto& joints = action.trajectory.joint_trajectory;
                    const auto& multi_dof = action.trajectory.multi_dof_joint_trajectory;
                    const auto duration = [](const auto& points) {
                        if (points.empty())
                            return 0.0;
                        const auto& time = points.back().time_from_start;
                        return static_cast<double>(time.sec)
                             + static_cast<double>(time.nanosec) * 1e-9;
                    };
                    description << " motion group=" << action.group << " joints=";
                    for (const auto& name : joints.joint_names)
                        description << name << ' ';
                    for (const auto& name : multi_dof.joint_names)
                        description << name << ' ';
                    description << "points=" << joints.points.size()
                                << " multi_dof_points=" << multi_dof.points.size() << " duration="
                                << std::max(duration(joints.points), duration(multi_dof.points));
                } else if constexpr (std::is_same_v<Action, GripperCommandConfig>) {
                    description << " gripper="
                                << (action.action == GripperAction::Open ? "open" : "close")
                                << " timeout=" << action.timeout;
                } else {
                    description << " scene operations=" << action.operations.size();
                    for (const auto& operation : action.operations)
                        std::visit(
                            [&](const auto& configured) {
                                using Operation = std::decay_t<decltype(configured)>;
                                if constexpr (std::is_same_v<Operation, AllowCollisionsConfig>) {
                                    description << (configured.allow ? " allow " : " forbid ")
                                                << configured.object << " links=";
                                    for (const auto& link : configured.links)
                                        description << link << ' ';
                                } else if constexpr (
                                    std::is_same_v<Operation, ObjectCollisionsConfig>) {
                                    description << (configured.allow ? " allow " : " forbid ")
                                                << configured.object
                                                << " other_object=" << configured.other_object;
                                } else {
                                    if constexpr (std::is_same_v<Operation, AttachObjectConfig>)
                                        description << " attach ";
                                    else
                                        description << " detach ";
                                    description << configured.object << " link=" << configured.link;
                                }
                            },
                            operation);
                }
            },
            step.action);
        description << '\n';
    }
    return description.str();
}

} // namespace rmcs_core::controller::arm::MTC
