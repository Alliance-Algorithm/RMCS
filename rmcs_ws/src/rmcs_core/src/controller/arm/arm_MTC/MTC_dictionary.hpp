#pragma once

#include "controller/arm/arm_MTC/MTC_step.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

namespace rmcs_core::controller::arm::MTC {

inline void validate(const TaskConfig& task) {
    const auto require = [](bool condition, const std::string& path, const std::string& reason) {
        if (!condition)
            throw std::invalid_argument(path + ": " + reason);
    };
    const auto positive = [](double value) { return std::isfinite(value) && value > 0.0; };
    const auto valid_pose = [&](const geometry_msgs::msg::PoseStamped& pose) {
        const auto& p = pose.pose.position;
        const auto& q = pose.pose.orientation;
        const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        return !pose.header.frame_id.empty() && std::isfinite(p.x) && std::isfinite(p.y)
            && std::isfinite(p.z) && std::isfinite(norm) && std::abs(norm - 1.0) < 1e-6;
    };
    require(!task.name.empty(), "task", "name is empty");
    require(!task.group.empty(), task.name, "group is empty");
    require(!task.ik_frame.empty(), task.name, "IK frame is empty");
    require(task.max_solutions > 0, task.name, "max_solutions must be positive");
    require(positive(task.timeout), task.name, "timeout must be finite and positive");

    std::vector<std::pair<const std::vector<NodeConfig>*, std::string>> pending{
        {&task.children, task.name}};
    while (!pending.empty()) {
        auto [children, path] = std::move(pending.back());
        pending.pop_back();
        require(!children->empty(), path, "container has no children");
        std::unordered_set<std::string> names;
        for (const auto& node : *children) {
            std::visit(
                [&](const auto& entry) {
                    using Entry = std::decay_t<decltype(entry)>;
                    require(!entry.name.empty(), path, "child name is empty");
                    const auto child_path = path + '/' + entry.name;
                    require(names.insert(entry.name).second, child_path, "duplicate child name");
                    if constexpr (std::is_same_v<Entry, ContainerConfig>) {
                        require(
                            entry.type == ContainerType::Serial
                                || entry.type == ContainerType::Alternatives
                                || entry.type == ContainerType::Fallbacks,
                            child_path, "unknown container type");
                        pending.emplace_back(&entry.children, child_path);
                    } else {
                        std::visit(
                            [&](const auto& stage) {
                                using Stage = std::decay_t<decltype(stage)>;
                                if constexpr (std::is_same_v<Stage, CurrentStateConfig>) {
                                    require(
                                        positive(stage.timeout), child_path,
                                        "scene timeout must be finite and positive");
                                } else if constexpr (std::is_same_v<Stage, FixedStateConfig>) {
                                    require(
                                        !stage.joints.empty(), child_path,
                                        "fixed state has no joints");
                                    for (const auto& [joint, value] : stage.joints)
                                        require(
                                            !joint.empty() && std::isfinite(value), child_path,
                                            "invalid fixed-state joint");
                                    std::unordered_set<std::string> object_ids;
                                    for (const auto& object : stage.objects) {
                                        require(
                                            !object.id.empty()
                                                && object_ids.insert(object.id).second,
                                            child_path, "empty or duplicate collision object id");
                                        require(
                                            valid_pose(object.pose), child_path,
                                            "invalid collision object pose");
                                        require(
                                            positive(object.dimensions.x)
                                                && positive(object.dimensions.y)
                                                && positive(object.dimensions.z),
                                            child_path,
                                            "collision object dimensions must be finite and "
                                            "positive");
                                    }
                                } else if constexpr (
                                    std::is_same_v<Stage, ModifyPlanningSceneConfig>) {
                                    require(
                                        !stage.operations.empty(), child_path,
                                        "planning-scene stage has no operations");
                                    for (const auto& operation : stage.operations)
                                        std::visit(
                                            [&](const auto& op) {
                                                using Operation = std::decay_t<decltype(op)>;
                                                if constexpr (
                                                    std::is_same_v<
                                                        Operation, AllowCollisionsConfig>) {
                                                    require(
                                                        !op.object.empty()
                                                            && (op.from_registry || !op.links.empty()),
                                                        child_path,
                                                        "collision allowance requires object and "
                                                        "links");
                                                    for (const auto& link : op.links)
                                                        require(
                                                            !link.empty(), child_path,
                                                            "collision link is empty");
                                                } else if constexpr (
                                                    std::is_same_v<
                                                        Operation, ObjectCollisionsConfig>) {
                                                    require(
                                                        !op.object.empty()
                                                            && !op.other_object.empty()
                                                            && op.object != op.other_object,
                                                        child_path,
                                                        "object collision allowance requires two "
                                                        "distinct nonempty object ids");
                                                } else {
                                                    require(
                                                        !op.object.empty()
                                                            && (op.from_registry || !op.link.empty()),
                                                        child_path,
                                                        "attach/detach requires object and link");
                                                    if constexpr (
                                                        std::is_same_v<
                                                            Operation, AttachObjectConfig>)
                                                        for (const auto& link : op.touch_links)
                                                            require(
                                                                !link.empty(), child_path,
                                                                "attachment touch link is empty");
                                                }
                                            },
                                            operation);
                                } else if constexpr (std::is_same_v<Stage, GripperCommandConfig>) {
                                    require(
                                        stage.action == GripperAction::Open
                                            || stage.action == GripperAction::Close,
                                        child_path, "unknown gripper command");
                                    require(
                                        positive(stage.timeout), child_path,
                                        "gripper timeout must be finite and positive");
                                } else {
                                    const auto& motion = stage.motion;
                                    require(
                                        positive(motion.velocity_scaling)
                                            && motion.velocity_scaling <= 1.0,
                                        child_path, "velocity scaling must be in (0, 1]");
                                    require(
                                        positive(motion.acceleration_scaling)
                                            && motion.acceleration_scaling <= 1.0,
                                        child_path, "acceleration scaling must be in (0, 1]");
                                    require(
                                        positive(motion.timeout), child_path,
                                        "motion timeout must be finite and positive");
                                    std::visit(
                                        [&](const auto& solver) {
                                            using Solver = std::decay_t<decltype(solver)>;
                                            if constexpr (std::is_same_v<Solver, PipelineConfig>) {
                                                require(
                                                    !solver.pipeline.empty()
                                                        && !solver.planner.empty(),
                                                    child_path, "pipeline or planner is empty");
                                            } else if constexpr (
                                                std::is_same_v<Solver, CartesianPathConfig>) {
                                                require(
                                                    positive(solver.step_size), child_path,
                                                    "Cartesian step size must be finite and "
                                                    "positive");
                                            }
                                        },
                                        motion.solver);
                                }
                                if constexpr (std::is_same_v<Stage, MoveToConfig>) {
                                    std::visit(
                                        [&](const auto& goal) {
                                            using Goal = std::decay_t<decltype(goal)>;
                                            if constexpr (std::is_same_v<Goal, std::string>) {
                                                require(
                                                    !goal.empty(), child_path,
                                                    "named goal is empty");
                                            } else if constexpr (
                                                std::is_same_v<
                                                    Goal, std::map<std::string, double>>) {
                                                require(
                                                    !goal.empty(), child_path,
                                                    "joint goal is empty");
                                                for (const auto& [joint, value] : goal)
                                                    require(
                                                        !joint.empty() && std::isfinite(value),
                                                        child_path, "invalid joint goal");
                                            } else {
                                                require(
                                                    valid_pose(goal), child_path,
                                                    "pose target is invalid or quaternion is not "
                                                    "normalized");
                                            }
                                        },
                                        stage.goal);
                                } else if constexpr (std::is_same_v<Stage, MoveRelativeConfig>) {
                                    std::visit(
                                        [&](const auto& direction) {
                                            using Direction = std::decay_t<decltype(direction)>;
                                            if constexpr (std::is_same_v<
                                                              Direction, RotationDirectionTarget>) {
                                                require(
                                                    !direction.frame_id.empty(), child_path,
                                                    "rotation frame is empty");
                                                for (double value : direction.axis_point)
                                                    require(
                                                        std::isfinite(value), child_path,
                                                        "invalid rotation axis point");
                                                require(
                                                    positive(std::hypot(
                                                        direction.axis_direction[0],
                                                        direction.axis_direction[1],
                                                        direction.axis_direction[2])),
                                                    child_path, "rotation axis must be finite and nonzero");
                                                require(
                                                      std::isfinite(stage.min_distance)
                                                          && positive(std::abs(stage.max_distance))
                                                          && std::abs(stage.max_distance) < std::numbers::pi
                                                          && std::abs(stage.min_distance)
                                                              <= std::abs(stage.max_distance)
                                                          && (stage.min_distance == 0.0
                                                              || (stage.min_distance > 0.0)
                                                                  == (stage.max_distance > 0.0)),
                                                      child_path,
                                                      "rotation requires same-sign angles with 0 <= abs(min) <= abs(max) < pi and nonzero max");
                                                require(
                                                    std::holds_alternative<CartesianPathConfig>(
                                                        stage.motion.solver),
                                                    child_path, "rotation requires CartesianPath");
                                            } else {
                                                require(
                                                    std::isfinite(stage.min_distance)
                                                        && stage.min_distance >= 0.0
                                                        && positive(stage.max_distance)
                                                        && stage.min_distance <= stage.max_distance,
                                                    child_path, "invalid relative distance range");
                                                const auto& vector = direction.vector;
                                                require(
                                                    !direction.header.frame_id.empty(), child_path,
                                                    "direction frame is empty");
                                                require(
                                                    positive(std::hypot(vector.x, vector.y, vector.z)),
                                                    child_path, "direction must be finite and nonzero");
                                            }
                                        },
                                        stage.direction);
                                    require(
                                        !std::holds_alternative<JointInterpolationConfig>(
                                            stage.motion.solver),
                                        child_path,
                                        "relative motion requires a Cartesian-capable solver");
                                }
                            },
                            entry.config);
                    }
                },
                node.config);
        }
    }
}

using ObjectBindings = std::unordered_map<std::string, std::string>;

inline void bind_task_objects(TaskConfig& task, const ObjectBindings& bindings = {}) {
    const auto bind = [&](std::string& object) {
        if (const auto found = bindings.find(object); found != bindings.end())
            object = found->second;
    };
    const auto visit = [&](auto&& self, std::vector<NodeConfig>& children) -> void {
        for (auto& node : children) {
            if (auto* container = std::get_if<ContainerConfig>(&node.config)) {
                self(self, container->children);
            } else if (auto* scene = std::get_if<ModifyPlanningSceneConfig>(
                           &std::get<StageConfig>(node.config).config)) {
                for (auto& operation : scene->operations)
                    std::visit(
                        [&](auto& options) {
                            bind(options.object);
                            if constexpr (std::is_same_v<
                                              std::decay_t<decltype(options)>, ObjectCollisionsConfig>)
                                bind(options.other_object);
                        },
                        operation);
                resolve_scene_objects(*scene);
            }
        }
    };
    visit(visit, task.children);
}

// Stores reusable stage blocks (task children without the task shell) and composes them into
// one task named after the shell, e.g. {"current_start", "pick", "place"} -> MTC_Task.
class Dictionary {
public:
    using Block = std::vector<NodeConfig>;
    using Entries = std::unordered_map<std::string, Block>;

    explicit Dictionary(Entries entries, TaskConfig shell = TaskConfig{.name = "MTC_Task"})
        : entries_(std::move(entries))
        , shell_(std::move(shell)) {
        if (!shell_.children.empty())
            throw std::invalid_argument(shell_.name + ": task shell must not have children");
        for (const auto& [name, block] : entries_) {
            if (name.empty())
                throw std::invalid_argument("MTC dictionary key is empty");
            auto task = shell_;
            task.name = name;
            task.children = block;
            validate(task);
        }
    }

    const Block& helper_find_chunk(const std::string& name) const {
        if (const auto it = entries_.find(name); it != entries_.end())
            return it->second;
        throw std::out_of_range("Unknown MTC block: " + name);
    }

    TaskConfig helper_build_chunk(
        const std::vector<std::string>& composed, const ObjectBindings& bindings = {}) const {
        if (composed.empty())
            throw std::invalid_argument(shell_.name + ": composition is empty");

        const auto is_start_state = [](const NodeConfig& node) {
            const auto* stage = std::get_if<StageConfig>(&node.config);
            return stage
                && (std::holds_alternative<CurrentStateConfig>(stage->config)
                    || std::holds_alternative<FixedStateConfig>(stage->config));
        };

        auto result = shell_;
        for (const auto& name : composed) {
            const auto& block = helper_find_chunk(name);
            for (const auto& node : block) {
                const bool first = result.children.empty();
                if (first && !is_start_state(node))
                    throw std::invalid_argument(
                        shell_.name + ": composition must start with a start-state block, got '"
                        + name + "'");
                if (!first && is_start_state(node))
                    throw std::invalid_argument(
                        shell_.name + ": block '" + name
                        + "' adds a start state after the beginning of the task");
                result.children.push_back(node);
            }
        }

        bind_task_objects(result, bindings);
        validate(result);
        return result;
    }

private:
    Entries entries_;
    TaskConfig shell_;
};

} // namespace rmcs_core::controller::arm::MTC
