#pragma once

#include "controller/arm/arm_MTC/MTC_step.hpp"

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace rmcs_core::controller::arm::MTC {

using ObjectBindings = std::unordered_map<std::string, std::string>;
using StageTargetOverride = std::variant<geometry_msgs::msg::PoseStamped, RotationDirectionTarget>;
using StageTargetOverrides = std::unordered_map<std::string, StageTargetOverride>;

// Calls visit(StageConfig&) for every stage in the tree, depth first.
template <typename Visit>
void for_each_stage(std::vector<NodeConfig>& children, const Visit& visit) {
    for (auto& node : children) {
        if (auto* container = std::get_if<ContainerConfig>(&node.config))
            for_each_stage(container->children, visit);
        else
            visit(std::get<StageConfig>(node.config));
    }
}

// Renames generic object ids (e.g. "unit") to concrete obstacles, then fills registry links.
inline void bind_task_objects(TaskConfig& task, const ObjectBindings& bindings = {}) {
    const auto bind = [&](std::string& object) {
        if (const auto found = bindings.find(object); found != bindings.end())
            object = found->second;
    };
    for_each_stage(task.children, [&](StageConfig& stage) {
        auto* scene = std::get_if<ModifyPlanningSceneConfig>(&stage.config);
        if (!scene)
            return;
        for (auto& operation : scene->operations)
            std::visit(
                overloaded{
                    [&](ObjectCollisionsConfig& op) {
                        bind(op.object);
                        bind(op.other_object);
                    },
                    [&](auto& op) { bind(op.object); },
                },
                operation);
        resolve_scene_objects(*scene);
    });
}

// Stores reusable stage blocks (task children without the task shell) and composes them into
// one task named after the shell, e.g. {"current_start", "pick", "place"} -> MTC_Task.
class Dictionary {
public:
    using Block = std::vector<NodeConfig>;
    using Entries = std::unordered_map<std::string, Block>;

    explicit Dictionary(Entries entries, TaskConfig shell = TaskConfig{.name = "MTC_Task"})
        : entries_(std::move(entries))
        , shell_(std::move(shell)) {}

    const Block& helper_find_chunk(const std::string& name) const {
        if (const auto it = entries_.find(name); it != entries_.end())
            return it->second;
        throw std::out_of_range("Unknown MTC block: " + name);
    }

    TaskConfig helper_build_chunk(
        const std::vector<std::string>& composed, const ObjectBindings& bindings = {},
        const StageTargetOverrides& targets = {}) const {
        auto result = shell_;
        for (const auto& name : composed) {
            const auto& block = helper_find_chunk(name);
            result.children.insert(result.children.end(), block.begin(), block.end());
        }
        bind_task_objects(result, bindings);
        for_each_stage(result.children, [&](StageConfig& stage) {
            const auto found = targets.find(stage.name);
            if (found == targets.end())
                return;
            std::visit(
                overloaded{
                    [&](const geometry_msgs::msg::PoseStamped& pose) {
                        std::get<MoveToConfig>(stage.config).goal = pose;
                    },
                    [&](const RotationDirectionTarget& rotation) {
                        std::get<MoveRelativeConfig>(stage.config).direction = rotation;
                    },
                },
                found->second);
        });
        return result;
    }

private:
    Entries entries_;
    TaskConfig shell_;
};

} // namespace rmcs_core::controller::arm::MTC
