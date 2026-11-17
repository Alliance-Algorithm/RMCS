#pragma once

#include "controller/arm/arm_MTC/MTC_step.hpp"

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace rmcs_core::controller::arm::MTC::detail {

// Operations must already be resolved (see bind_task_objects / resolve_scene_objects).
inline void apply_scene_operations(
    const planning_scene::PlanningScenePtr& scene, const ModifyPlanningSceneConfig& config,
    const std::string& stage_name) {
    const auto attachment = [&](const std::string& object, const std::string& link,
                                std::uint8_t operation, const std::vector<std::string>& touch) {
        moveit_msgs::msg::AttachedCollisionObject message;
        message.link_name = link;
        message.object.id = object;
        message.object.operation = operation;
        message.touch_links = touch;
        if (!scene->processAttachedCollisionObjectMsg(message))
            throw std::runtime_error(stage_name + ": failed to attach/detach " + object);
    };
    auto& acm = scene->getAllowedCollisionMatrixNonConst();
    for (const auto& operation : config.operations)
        std::visit(
            overloaded{
                [&](const AllowCollisionsConfig& op) {
                    for (const auto& link : op.links)
                        acm.setEntry(op.object, link, op.allow);
                },
                [&](const ObjectCollisionsConfig& op) {
                    acm.setEntry(op.object, op.other_object, op.allow);
                },
                [&](const AttachObjectConfig& op) {
                    attachment(
                        op.object, op.link, moveit_msgs::msg::CollisionObject::ADD,
                        op.touch_links);
                },
                [&](const DetachObjectConfig& op) {
                    attachment(op.object, op.link, moveit_msgs::msg::CollisionObject::REMOVE, {});
                },
            },
            operation);
}

} // namespace rmcs_core::controller::arm::MTC::detail
