#pragma once

#include "controller/arm/arm_MTC/MTC_step.hpp"

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/attached_body.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>

#include <stdexcept>
#include <string>
#include <type_traits>
#include <variant>

namespace rmcs_core::controller::arm::MTC::detail {

inline void validate_scene_operation_links(
    const ModifyPlanningSceneConfig& config, const moveit::core::RobotModel& model,
    const std::string& stage_name) {
    for (const auto& operation : config.operations)
        std::visit(
            [&](const auto& options) {
                const auto check_link = [&](const std::string& link) {
                    if (!model.hasLinkModel(link))
                        throw std::invalid_argument(
                            stage_name + ": unknown robot link " + link + " for object "
                            + options.object);
                };
                using Operation = std::decay_t<decltype(options)>;
                if constexpr (std::is_same_v<Operation, AllowCollisionsConfig>) {
                    for (const auto& link : options.links)
                        check_link(link);
                } else if constexpr (!std::is_same_v<Operation, ObjectCollisionsConfig>) {
                    check_link(options.link);
                    if constexpr (std::is_same_v<Operation, AttachObjectConfig>)
                        for (const auto& link : options.touch_links)
                            check_link(link);
                }
            },
            operation);
}

inline void apply_scene_operations(
    const planning_scene::PlanningScenePtr& scene, ModifyPlanningSceneConfig config,
    const std::string& stage_name) {
    if (!scene)
        throw std::invalid_argument(stage_name + ": planning scene is null");
    resolve_scene_objects(config);
    validate_scene_operation_links(config, *scene->getRobotModel(), stage_name);
    for (const auto& operation : config.operations)
        std::visit(
            [&](const auto& options) {
                const auto fail = [&](const std::string& reason) {
                    throw std::runtime_error(
                        stage_name + ": " + reason + " [" + options.object + ']');
                };
                using Operation = std::decay_t<decltype(options)>;
                const auto* attached = scene->getCurrentState().getAttachedBody(options.object);
                const bool in_world = scene->getWorld()->hasObject(options.object);
                if constexpr (std::is_same_v<Operation, AllowCollisionsConfig>) {
                    if (!in_world && !attached)
                        fail("collision object is absent from the planning scene");
                    for (const auto& link : options.links)
                        scene->getAllowedCollisionMatrixNonConst().setEntry(
                            options.object, link, options.allow);
                } else if constexpr (std::is_same_v<Operation, ObjectCollisionsConfig>) {
                    if (options.object.empty() || options.other_object.empty()
                        || options.object == options.other_object)
                        fail("object collision allowance requires two distinct nonempty object ids");
                    if (!in_world && !attached)
                        fail("collision object is absent from the planning scene");
                    if (!scene->getWorld()->hasObject(options.other_object)
                        && !scene->getCurrentState().hasAttachedBody(options.other_object))
                        fail("other collision object is absent from the planning scene: "
                             + options.other_object);
                    scene->getAllowedCollisionMatrixNonConst().setEntry(
                        options.object, options.other_object, options.allow);
                } else {
                    moveit_msgs::msg::AttachedCollisionObject message;
                    message.link_name = options.link;
                    message.object.id = options.object;
                    if constexpr (std::is_same_v<Operation, AttachObjectConfig>) {
                        if (attached)
                            fail("collision object is already attached");
                        if (!in_world)
                            fail("cannot attach an object absent from the planning scene");
                        message.object.operation = moveit_msgs::msg::CollisionObject::ADD;
                        message.touch_links = options.touch_links;
                    } else {
                        if (!attached)
                            fail("cannot detach an object that is not attached");
                        if (attached->getAttachedLinkName() != options.link)
                            fail(
                                "collision object is attached to " + attached->getAttachedLinkName()
                                + ", not " + options.link);
                        message.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
                    }
                    if (!scene->processAttachedCollisionObjectMsg(message))
                        fail("failed to apply attachment operation");
                }
            },
            operation);
}

} // namespace rmcs_core::controller::arm::MTC::detail
