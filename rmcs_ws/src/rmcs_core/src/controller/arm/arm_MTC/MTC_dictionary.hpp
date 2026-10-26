#pragma once

#include "controller/arm/arm_MTC/MTC_step.hpp"

#include <cmath>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>

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
                                                        !op.object.empty() && !op.links.empty(),
                                                        child_path,
                                                        "collision allowance requires object and "
                                                        "links");
                                                    for (const auto& link : op.links)
                                                        require(
                                                            !link.empty(), child_path,
                                                            "collision link is empty");
                                                } else {
                                                    require(
                                                        !op.object.empty() && !op.link.empty(),
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
                                    const auto& direction = stage.direction.vector;
                                    const double length =
                                        std::hypot(direction.x, direction.y, direction.z);
                                    require(
                                        !stage.direction.header.frame_id.empty(), child_path,
                                        "direction frame is empty");
                                    require(
                                        positive(length), child_path,
                                        "direction must be finite and nonzero");
                                    require(
                                        std::isfinite(stage.min_distance)
                                            && stage.min_distance >= 0.0
                                            && positive(stage.max_distance)
                                            && stage.min_distance <= stage.max_distance,
                                        child_path, "invalid relative distance range");
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

class Dictionary {
public:
    using Entries = std::unordered_map<std::string, TaskConfig>;

    explicit Dictionary(Entries entries)
        : entries_(std::move(entries)) {
        for (const auto& [name, task] : entries_) {
            if (name.empty())
                throw std::invalid_argument("MTC dictionary key is empty");
            validate(task);
        }
    }

    const TaskConfig& at(const std::string& name) const {
        if (const auto it = entries_.find(name); it != entries_.end())
            return it->second;
        throw std::out_of_range("Unknown MTC task: " + name);
    }

private:
    Entries entries_;
};

} // namespace rmcs_core::controller::arm::MTC
