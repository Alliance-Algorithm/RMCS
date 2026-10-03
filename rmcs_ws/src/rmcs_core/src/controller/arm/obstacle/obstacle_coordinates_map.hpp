#pragma once

#include "obstacle.hpp"
#include <map>
#include <memory>
#include <rclcpp/logger.hpp>
#include <string>

namespace rmcs_core::controller::arm::obstacle {
class ObstacleCoordinatesMap {
public:
    ObstacleCoordinatesMap()
        : logger_(rclcpp::get_logger("obstacle_coordinates_map")) {

        add_pose("energy_unit_compensation", {-0.0475, -0.0475, -0.0725, 0, 0, 0});

        add_pose("gimbal_compensation", {0, 0.5399, -0.264 + 0.004, 0, 0, 0});

        add_pose(
            "energy_unit_left_front_store", {-0.23138, 0.12818, 0.084 + 0.0725 + 0.004, 0, 0, 0});
        add_pose(
            "energy_unit_left_back_store", {-0.35338, 0.19618, 0.084 + 0.0725 + 0.004, 0, 0, 0});
        add_pose(
            "energy_unit_right_back_store", {-0.35338, -0.19618, 0.084 + 0.0725 + 0.004, 0, 0, 0});
        add_pose(
            "energy_unit_right_front_store", {-0.23138, -0.12818, 0.084 + 0.0725 + 0.004, 0, 0, 0});

        add_pose("gimbal", {0, 0, 0, 0, 0, 0});
        /*TODO: add poses for islands
                add_pose("energy_unit_left_front_island", {0, 0, 0, 0, 0, 0});
                add_pose("energy_unit_left_back_island", {0, 0, 0, 0, 0, 0});
                add_pose("energy_unit_right_back_island", {0, 0, 0, 0, 0, 0});
                add_pose("energy_unit_right_front_island", {0, 0, 0, 0, 0, 0});

                add_pose("energy_unit_left_front_island_2", {0, 0, 0, 0, 0, 0});
                add_pose("energy_unit_left_front_island_2", {0, 0, 0, 0, 0, 0});
        */
    }

    Pose get_pose(const std::string& id) const {
        const auto* pose = find_pose(id);
        if (!pose) {
            RCLCPP_WARN(logger_, "unknown pose: %s", id.c_str());
            return Pose{0, 0, 0, 0, 0, 0};
        }
        return *pose;
    }

private:
    const Pose* find_pose(const std::string& id) const {
        auto it = pose_map_.find(id);
        return it == pose_map_.end() ? nullptr : it->second.get();
    }

    void add_pose(std::string id, Pose pose) {
        auto [it, inserted] = pose_map_.emplace(id, std::make_unique<Pose>(pose));
        if (!inserted) {
            RCLCPP_WARN(logger_, "duplicate pose id: %s", id.c_str());
            return;
        }
    }

    rclcpp::Logger logger_;
    std::map<std::string, std::unique_ptr<Pose>> pose_map_;
};
}; // namespace rmcs_core::controller::arm::obstacle