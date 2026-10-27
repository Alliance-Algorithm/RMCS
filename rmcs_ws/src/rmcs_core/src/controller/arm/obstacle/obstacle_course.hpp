#pragma once

#include "moveit_msgs/msg/link_padding.hpp"
#include "obstacle.hpp"
#include "obstacle_coordinates_map.hpp"
#include "obstacle_parameter_map.hpp"
#include <algorithm>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <moveit/collision_detection/collision_matrix.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace rmcs_core::controller::arm::obstacle {

class ObstacleCourse {

public:
    explicit ObstacleCourse(
        moveit::planning_interface::MoveGroupInterface* move_group,
        const ObstacleCoordinatesMap& obstacle_coordinates_map)
        : logger_(rclcpp::get_logger("obstacle_course")) {

        if (!move_group || !move_group->getRobotModel())
            throw std::invalid_argument("Obstacle course requires a MoveGroup robot model");
        frame_id_ = move_group->getPlanningFrame();

        model_ = move_group->getRobotModel();
        const auto parameters = make_obstacle_parameter_map(obstacle_coordinates_map);
        if (!model_->hasLinkModel("link_6"))
            throw std::invalid_argument("Unknown padding link: link_6");
        for (const auto& [id, config] : parameters) {
            if (!config.attach_link.empty() && !model_->hasLinkModel(config.attach_link))
                throw std::invalid_argument("Unknown attach link for obstacle: " + id);
            for (const auto& link : config.touch_links)
                if (!model_->hasLinkModel(link))
                    throw std::invalid_argument("Unknown touch link for obstacle: " + id + ": " + link);
        }

        // A dedicated query node avoids spinning the controller's existing executor.
        query_node_ = std::make_shared<rclcpp::Node>(
            "obstacle_scene_query",
            move_group->getNode()->get_namespace(),
            rclcpp::NodeOptions()
                .use_global_arguments(false)
                .context(move_group->getNode()->get_node_base_interface()->get_context()));
        scene_client_ = query_node_->create_client<moveit_msgs::srv::GetPlanningScene>(
            "get_planning_scene");
        apply_scene_client_ = query_node_->create_client<moveit_msgs::srv::ApplyPlanningScene>(
            "apply_planning_scene");

        moveit_msgs::msg::PlanningScene ps;
        ps.is_diff = true;
        ps.robot_state.is_diff = true;
        ps.link_padding.push_back(
            moveit_msgs::msg::LinkPadding().set__link_name("link_6").set__padding(0.02));
        if (!apply_scene(ps))
            throw std::runtime_error("Failed to apply obstacle link padding");

        for (const auto& [id, config] : parameters)
            add_collision(id, config);
        const auto scene = sync_scene();
        const auto padding = std::find_if(
            scene.link_padding.begin(), scene.link_padding.end(), [](const auto& value) {
                return value.link_name == "link_6" && std::abs(value.padding - 0.02) < 1e-9;
            });
        if (padding == scene.link_padding.end())
            throw std::runtime_error("Obstacle link padding was not applied");
        for (const auto& [id, config] : parameters) {
            auto* obstacle = find(id);
            if (obstacle->attached()) {
                const auto attached = std::find_if(
                    scene.robot_state.attached_collision_objects.begin(),
                    scene.robot_state.attached_collision_objects.end(),
                    [&id](const auto& object) { return object.object.id == id; });
                if (attached == scene.robot_state.attached_collision_objects.end()
                    || !obstacle->attachable() || attached->link_name != obstacle->attach_link()
                    || std::any_of(scene.world.collision_objects.begin(),
                        scene.world.collision_objects.end(),
                        [&id](const auto& object) { return object.id == id; }))
                    throw std::runtime_error("Unexpected initial attachment for obstacle: " + id);
                continue;
            }
            obstacle->set_pose(config.pose);
            obstacle->set_operation(config.initial_operation);
            if (!apply_world(*obstacle))
                throw std::runtime_error("Failed to initialize obstacle: " + id);
        }
    }

    bool scene_uncertain() const { return scene_uncertain_; }

    bool set_operation(const std::string& id, CollisionObjectOperation operation) {
        auto* obstacle = find_unattached(id);
        if (!obstacle)
            return false;

        obstacle->set_operation(operation);
        return apply_world(*obstacle);
    }

    bool set_pose(const std::string& id, Pose pose) {
        auto* obstacle = find_unattached(id);
        if (!obstacle)
            return false;

        obstacle->set_pose(pose);
        return apply_world(*obstacle);
    }

    // Blocking scene queries and application: call only from the non-RT ArmController worker.
    bool set_collisions(
        const std::string& id, const std::vector<std::string>& links, bool allow,
        const std::function<bool()>& can_apply = {}) {
        if (links.empty()) {
            RCLCPP_WARN(logger_, "collision links must not be empty: %s", id.c_str());
            return false;
        }
        for (const auto& link : links) {
            if (!model_->hasLinkModel(link)) {
                RCLCPP_WARN(logger_, "unknown collision link: %s", link.c_str());
                return false;
            }
        }

        const auto before = sync_scene();
        const bool in_world = std::any_of(
            before.world.collision_objects.begin(), before.world.collision_objects.end(),
            [&id](const auto& object) { return object.id == id; });
        const bool attached = std::any_of(before.robot_state.attached_collision_objects.begin(),
            before.robot_state.attached_collision_objects.end(),
            [&id](const auto& object) { return object.object.id == id; });
        if (!in_world && !attached) {
            RCLCPP_WARN(logger_, "collision object is not in the scene: %s", id.c_str());
            return false;
        }

        collision_detection::AllowedCollisionMatrix acm(before.allowed_collision_matrix);
        for (const auto& link : links)
            acm.setEntry(id, link, allow);
        moveit_msgs::msg::PlanningScene diff;
        diff.is_diff = true;
        diff.robot_state.is_diff = true;
        acm.getMessage(diff.allowed_collision_matrix);
        if (!apply_scene(diff, can_apply))
            return false;
        const auto after = sync_scene();
        const collision_detection::AllowedCollisionMatrix actual(after.allowed_collision_matrix);
        for (const auto& link : links) {
            collision_detection::AllowedCollision::Type entry;
            if (!actual.getEntry(id, link, entry)
                || entry != (allow ? collision_detection::AllowedCollision::ALWAYS
                                   : collision_detection::AllowedCollision::NEVER)) {
                RCLCPP_WARN(logger_, "collision entry was not applied: %s: %s",
                    id.c_str(), link.c_str());
                return false;
            }
        }
        return true;
    }

    // The obstacle must already be in the world (ADD/APPEND/MOVE)
    bool attach(const std::string& id, const std::function<bool()>& can_apply = {}) {
        auto* obstacle = find_attachable(id);
        if (!obstacle)
            return false;
        const auto before = sync_scene();
        if (obstacle->attached()) {
            RCLCPP_WARN(logger_, "obstacle already attached: %s", id.c_str());
            return false;
        }

        if (std::none_of(before.world.collision_objects.begin(), before.world.collision_objects.end(),
                [&id](const auto& object) { return object.id == id; })) {
            RCLCPP_WARN(logger_, "obstacle is not in the world: %s", id.c_str());
            return false;
        }

        moveit_msgs::msg::PlanningScene diff;
        diff.is_diff = true;
        diff.robot_state.is_diff = true;
        diff.robot_state.attached_collision_objects.push_back(obstacle->export_attach());
        if (!apply_scene(diff, can_apply))
            return false;
        const auto after = sync_scene();
        const auto attached = std::find_if(after.robot_state.attached_collision_objects.begin(),
            after.robot_state.attached_collision_objects.end(),
            [&id](const auto& object) { return object.object.id == id; });
        const bool in_world = std::any_of(
            after.world.collision_objects.begin(), after.world.collision_objects.end(),
            [&id](const auto& object) { return object.id == id; });
        if (attached == after.robot_state.attached_collision_objects.end()
            || attached->link_name != obstacle->attach_link() || in_world) {
            RCLCPP_WARN(logger_, "failed to attach obstacle: %s", id.c_str());
            return false;
        }
        return true;
    }

    // The obstacle stays in the world where it was released
    bool detach(const std::string& id, const std::function<bool()>& can_apply = {}) {
        auto* obstacle = find_attachable(id);
        if (!obstacle)
            return false;
        const auto before = sync_scene();
        const auto attached = std::find_if(before.robot_state.attached_collision_objects.begin(),
            before.robot_state.attached_collision_objects.end(),
            [&id](const auto& object) { return object.object.id == id; });
        if (attached == before.robot_state.attached_collision_objects.end()
            || attached->link_name != obstacle->attach_link()) {
            RCLCPP_WARN(logger_, "obstacle not attached: %s", id.c_str());
            return false;
        }

        moveit_msgs::msg::PlanningScene diff;
        diff.is_diff = true;
        diff.robot_state.is_diff = true;
        diff.robot_state.attached_collision_objects.push_back(obstacle->export_detach());
        if (!apply_scene(diff, can_apply))
            return false;

        const auto after = sync_scene();
        const bool in_world = std::any_of(
            after.world.collision_objects.begin(), after.world.collision_objects.end(),
            [&id](const auto& object) { return object.id == id; });
        if (obstacle->attached() || !in_world) {
            RCLCPP_WARN(logger_, "failed to detach obstacle: %s", id.c_str());
            return false;
        }
        return true;
    }

    // Detach, then place the obstacle at a known pose
    bool detach(
        const std::string& id, Pose place_pose,
        const std::function<bool()>& can_apply = {}) {
        if (!detach(id, can_apply))
            return false;
        auto* obstacle = find_unattached(id);
        if (!obstacle)
            return false;
        obstacle->set_pose(place_pose);
        return apply_world(*obstacle, can_apply);
    }

    void apply_collision() {
        const auto before = sync_scene();
        std::vector<moveit_msgs::msg::CollisionObject> objects;
        objects.reserve(obstacles_.size());

        for (auto& [id, obstacle] : obstacles_) {
            if (obstacle->attached())
                continue;
            auto object = obstacle->export_collision();
            if (object.operation == moveit_msgs::msg::CollisionObject::REMOVE
                && world_matches(before, object))
                continue;
            objects.push_back(std::move(object));
        }
        if (objects.empty())
            return;
        moveit_msgs::msg::PlanningScene diff;
        diff.is_diff = true;
        diff.robot_state.is_diff = true;
        diff.world.collision_objects = objects;
        const bool applied = apply_scene(diff);
        const auto scene = sync_scene();
        for (const auto& object : objects)
            if (!world_matches(scene, object))
                throw std::runtime_error("Obstacle batch postcondition failed: " + object.id);
        if (!applied)
            throw std::runtime_error("Failed to apply obstacle batch");
    }

private:
    bool apply_scene(
        moveit_msgs::msg::PlanningScene scene,
        const std::function<bool()>& can_apply = {}) {
        using namespace std::chrono_literals;
        if (scene_uncertain_)
            throw std::runtime_error(
                "Obstacle planning scene uncertain; restart/reconciliation required");
        if (can_apply && !can_apply())
            return false;
        const bool available = apply_scene_client_->wait_for_service(5s);
        if (can_apply && !can_apply())
            return false;
        if (!available)
            throw std::runtime_error("Obstacle planning scene apply service unavailable");
        auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
        request->scene = std::move(scene);
        if (can_apply && !can_apply())
            return false;
        // Until acknowledged, a submitted mutation may still be applied remotely.
        scene_uncertain_ = true;
        auto future = apply_scene_client_->async_send_request(request);
        try {
            if (rclcpp::spin_until_future_complete(query_node_, future, 5s)
                != rclcpp::FutureReturnCode::SUCCESS)
                throw std::runtime_error(
                    "Obstacle planning scene apply incomplete; restart/reconciliation required");
            if (!future.get()->success)
                throw std::runtime_error(
                    "Obstacle planning scene apply failed; restart/reconciliation required");
        } catch (...) {
            apply_scene_client_->remove_pending_request(future);
            throw;
        }
        scene_uncertain_ = false;
        return true;
    }

    Obstacle* find(const std::string& id) {
        auto it = obstacles_.find(id);
        return it == obstacles_.end() ? nullptr : it->second.get();
    }

    Obstacle* find_unattached(const std::string& id) {
        auto* obstacle = find(id);
        if (!obstacle) {
            RCLCPP_WARN(logger_, "unknown obstacle: %s", id.c_str());
            return nullptr;
        }
        sync_scene();
        if (obstacle->attached()) {
            RCLCPP_WARN(logger_, "obstacle is attached, detach it first: %s", id.c_str());
            return nullptr;
        }
        return obstacle;
    }

    Obstacle* find_attachable(const std::string& id) {
        auto* obstacle = find(id);
        if (!obstacle) {
            RCLCPP_WARN(logger_, "unknown obstacle: %s", id.c_str());
            return nullptr;
        }
        if (!obstacle->attachable()) {
            RCLCPP_WARN(logger_, "obstacle is not attachable: %s", id.c_str());
            return nullptr;
        }
        return obstacle;
    }

    // Query failures must not be mistaken for an empty world or a successful detach.
    moveit_msgs::msg::PlanningScene sync_scene() {
        using namespace std::chrono_literals;
        if (!scene_client_->wait_for_service(5s))
            throw std::runtime_error("Obstacle planning scene query service unavailable");
        auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        using Components = moveit_msgs::msg::PlanningSceneComponents;
        request->components.components = Components::WORLD_OBJECT_GEOMETRY
            | Components::ROBOT_STATE_ATTACHED_OBJECTS | Components::LINK_PADDING_AND_SCALING
            | Components::ALLOWED_COLLISION_MATRIX;
        auto future = scene_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(query_node_, future, 5s)
            != rclcpp::FutureReturnCode::SUCCESS) {
            scene_client_->remove_pending_request(future);
            throw std::runtime_error("Failed to query obstacle planning scene");
        }
        auto scene = future.get()->scene;
        for (auto& [id, obstacle] : obstacles_) {
            const auto attached = std::find_if(scene.robot_state.attached_collision_objects.begin(),
                scene.robot_state.attached_collision_objects.end(),
                [&id](const auto& object) { return object.object.id == id; });
            const auto world = std::find_if(
                scene.world.collision_objects.begin(), scene.world.collision_objects.end(),
                [&id](const auto& object) { return object.id == id; });
            if (attached != scene.robot_state.attached_collision_objects.end()) {
                obstacle->set_attached(true);
            } else if (world != scene.world.collision_objects.end()) {
                obstacle->sync_world(*world);
            } else {
                obstacle->set_attached(false);
            }
        }
        return scene;
    }

    bool world_matches(
        const moveit_msgs::msg::PlanningScene& scene,
        const moveit_msgs::msg::CollisionObject& object) const {
        const bool in_world = std::any_of(
            scene.world.collision_objects.begin(), scene.world.collision_objects.end(),
            [&object](const auto& value) { return value.id == object.id; });
        const bool attached = std::any_of(scene.robot_state.attached_collision_objects.begin(),
            scene.robot_state.attached_collision_objects.end(),
            [&object](const auto& value) { return value.object.id == object.id; });
        return !attached && in_world == (object.operation != moveit_msgs::msg::CollisionObject::REMOVE);
    }

    bool apply_world(Obstacle& obstacle, const std::function<bool()>& can_apply = {}) {
        const auto object = obstacle.export_collision();
        if (object.operation == moveit_msgs::msg::CollisionObject::REMOVE
            && world_matches(sync_scene(), object))
            return true;
        moveit_msgs::msg::PlanningScene diff;
        diff.is_diff = true;
        diff.robot_state.is_diff = true;
        diff.world.collision_objects.push_back(object);
        if (!apply_scene(diff, can_apply))
            return false;
        const auto scene = sync_scene();
        return world_matches(scene, object);
    }

    void add_collision(const std::string& id, const ObstacleConfig& config) {
        auto [it, inserted] = obstacles_.emplace(
            id,
            std::make_unique<Obstacle>(frame_id_, id, config.attach_link, config.touch_links));
        if (!inserted) {
            RCLCPP_WARN(logger_, "duplicate obstacle id: %s", id.c_str());
            return;
        }
        Eigen::Vector3d scale{config.scale[0], config.scale[1], config.scale[2]};
        it->second->mesh_load(config.mesh, scale);
        it->second->set_pose_compensation(config.compensation);
        it->second->set_pose(config.pose);
    }

    rclcpp::Logger logger_;

    moveit::core::RobotModelConstPtr model_;

    rclcpp::Node::SharedPtr query_node_;
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_client_;
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_client_;
    bool scene_uncertain_ = false;

    std::string frame_id_;

    std::map<std::string, std::unique_ptr<Obstacle>> obstacles_;
};
} // namespace rmcs_core::controller::arm::obstacle
