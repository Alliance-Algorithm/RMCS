#pragma once
#include "obstacle.hpp"
#include <memory>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <string>

namespace rmcs_core::controller::arm::obstacle {

class ObstacleCourse {

public:
    explicit ObstacleCourse(moveit::planning_interface::MoveGroupInterface* move_group)
        : logger_(rclcpp::get_logger("obstacle_course")) {
        move_group_ = move_group;
        frame_id_ = move_group_->getPlanningFrame();

        add_collision("energy_unit_left_front", "energy_unit");
        add_collision("energy_unit_left_back", "energy_unit");
        add_collision("energy_unit_right_back", "energy_unit");
        add_collision("energy_unit_right_front", "energy_unit");
    }

    void set_operation(const std::string& id, CollisionObjectOperation operation) {
        auto* obstacle = find(id);

        if (!obstacle) {
            RCLCPP_WARN(logger_, "unknown obstacle: %s", id.c_str());
            return;
        }
        obstacle->set_operation(operation);
    }

    void set_pose(const std::string& id, Pose pose) {
        auto* obstacle = find(id);

        if (!obstacle) {
            RCLCPP_WARN(logger_, "unknown obstacle: %s", id.c_str());
            return;
        }
        obstacle->set_pose(pose);
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

    void add_collision(const std::string& id, const std::string& mesh, Pose pose = {}) {
        auto [it, inserted] = obstacles_.emplace(id, std::make_unique<Obstacle>(frame_id_, id));
        if (!inserted) {
            RCLCPP_WARN(logger_, "duplicate obstacle if: %s", id.c_str());
            return;
        }
        it->second->mesh_load(mesh);
        it->second->set_pose(pose);
    }

    rclcpp::Logger logger_;

    moveit::planning_interface::MoveGroupInterface* move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_;

    std::string frame_id_;

    std::map<std::string, std::unique_ptr<Obstacle>> obstacles_;
};
} // namespace rmcs_core::controller::arm::obstacle
