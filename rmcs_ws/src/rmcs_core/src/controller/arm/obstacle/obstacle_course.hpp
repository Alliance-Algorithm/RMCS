#pragma once

#include "moveit_msgs/msg/link_padding.hpp"
#include "obstacle.hpp"
#include "obstacle_coordinates_map.hpp"
#include <memory>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <numbers>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <string>

namespace rmcs_core::controller::arm::obstacle {

class ObstacleCourse {

public:
    explicit ObstacleCourse(
        moveit::planning_interface::MoveGroupInterface* move_group,
        const ObstacleCoordinatesMap& obstacle_coordinates_map)
        : logger_(rclcpp::get_logger("obstacle_course")) {

        frame_id_ = move_group->getPlanningFrame();

        moveit_msgs::msg::PlanningScene ps;
        ps.is_diff = true;
        ps.robot_state.is_diff = true;
        ps.link_padding.push_back(
            moveit_msgs::msg::LinkPadding().set__link_name("link_6").set__padding(0.02));
        planning_scene_.applyPlanningScene(ps);

        Eigen::Vector3d mm2m_scale = {0.001, 0.001, 0.001};

        add_collision(
            "energy_unit_left_front", "energy_unit", mm2m_scale,
            obstacle_coordinates_map.get_pose("energy_unit_compensation"),
            obstacle_coordinates_map.get_pose("energy_unit_left_front_store"));
        add_collision(
            "energy_unit_left_back", "energy_unit", mm2m_scale,
            obstacle_coordinates_map.get_pose("energy_unit_compensation"),
            obstacle_coordinates_map.get_pose("energy_unit_left_back_store"));
        add_collision(
            "energy_unit_right_back", "energy_unit", mm2m_scale,
            obstacle_coordinates_map.get_pose("energy_unit_compensation"),
            obstacle_coordinates_map.get_pose("energy_unit_right_back_store"));
        add_collision(
            "energy_unit_right_front", "energy_unit", mm2m_scale,
            obstacle_coordinates_map.get_pose("energy_unit_compensation"),
            obstacle_coordinates_map.get_pose("energy_unit_right_front_store"));
        add_collision(
            "gimbal", "gimbal", mm2m_scale,
            obstacle_coordinates_map.get_pose("gimbal_compensation"),
            obstacle_coordinates_map.get_pose("gimbal"));

        set_operation("energy_unit_left_back", CollisionObjectOperation::ADD);
        set_operation("energy_unit_right_back", CollisionObjectOperation::ADD);
        set_operation("energy_unit_right_front", CollisionObjectOperation::ADD);
        set_operation("energy_unit_left_front", CollisionObjectOperation::ADD);
        set_operation("gimbal", CollisionObjectOperation::ADD);
    }

    void set_operation(const std::string& id, CollisionObjectOperation operation) {
        auto* obstacle = find(id);

        if (!obstacle) {
            RCLCPP_WARN(logger_, "unknown obstacle: %s", id.c_str());
            return;
        }
        obstacle->set_operation(operation);
        planning_scene_.applyCollisionObject(obstacle->export_collision());
    }

    void set_pose(const std::string& id, Pose pose) {
        auto* obstacle = find(id);

        if (!obstacle) {
            RCLCPP_WARN(logger_, "unknown obstacle: %s", id.c_str());
            return;
        }
        obstacle->set_pose(pose);
        planning_scene_.applyCollisionObject(obstacle->export_collision());
    }

    void apply_collision() {
        std::vector<moveit_msgs::msg::CollisionObject> objects;
        objects.reserve(obstacles_.size());

        for (auto& [id, obstacle] : obstacles_) {
            objects.push_back(obstacle->export_collision());
        }
        planning_scene_.applyCollisionObjects(objects);
    }

private:
    Obstacle* find(const std::string& id) {
        auto it = obstacles_.find(id);
        return it == obstacles_.end() ? nullptr : it->second.get();
    }

    void add_collision(
        const std::string& id, const std::string& mesh, Eigen::Vector3d& scale, Pose compensation,
        Pose pose = {}) {
        auto [it, inserted] = obstacles_.emplace(id, std::make_unique<Obstacle>(frame_id_, id));
        if (!inserted) {
            RCLCPP_WARN(logger_, "duplicate obstacle id: %s", id.c_str());
            return;
        }
        it->second->mesh_load(mesh, scale);
        it->second->set_pose_compensation(compensation);
        it->second->set_pose(pose);
    }

    rclcpp::Logger logger_;

    moveit::planning_interface::PlanningSceneInterface planning_scene_;

    std::string frame_id_;

    std::map<std::string, std::unique_ptr<Obstacle>> obstacles_;
};
} // namespace rmcs_core::controller::arm::obstacle
