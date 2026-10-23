#pragma once

#include "controller/arm/obstacle/obstacle_coordinates_map.hpp"
#include "controller/arm/obstacle/obstacle_types.hpp"

#include <map>
#include <stdexcept>
#include <string>

namespace rmcs_core::controller::arm::obstacle {

inline auto make_obstacle_parameter_map(const ObstacleCoordinatesMap& coordinates) {
    using ParameterMap = std::map<std::string, ObstacleConfig>;
    return ParameterMap{
        {"energy_unit_left_front",
         ObstacleConfig{
             .mesh = "energy_unit",
             .scale = {0.001, 0.001, 0.001},
             .compensation = coordinates.get_pose("energy_unit_compensation"),
             .pose = coordinates.get_pose("energy_unit_left_front_store"),
             .initial_operation = CollisionObjectOperation::IDLE,
             .touch_links = {"link_6"},
             .attach_link = "link_6"}},
        {"energy_unit_left_back",
         ObstacleConfig{
             .mesh = "energy_unit",
             .scale = {0.001, 0.001, 0.001},
             .compensation = coordinates.get_pose("energy_unit_compensation"),
             .pose = coordinates.get_pose("energy_unit_left_back_store"),
             .initial_operation = CollisionObjectOperation::IDLE,
             .touch_links = {"link_6"},
             .attach_link = "link_6"}},
        {"energy_unit_right_back",
         ObstacleConfig{
             .mesh = "energy_unit",
             .scale = {0.001, 0.001, 0.001},
             .compensation = coordinates.get_pose("energy_unit_compensation"),
             .pose = coordinates.get_pose("energy_unit_right_back_store"),
             .initial_operation = CollisionObjectOperation::IDLE,
             .touch_links = {"link_6"},
             .attach_link = "link_6"}},
        {"energy_unit_right_front",
         ObstacleConfig{
             .mesh = "energy_unit",
             .scale = {0.001, 0.001, 0.001},
             .compensation = coordinates.get_pose("energy_unit_compensation"),
             .pose = coordinates.get_pose("energy_unit_right_front_store"),
             .initial_operation = CollisionObjectOperation::IDLE,
             .touch_links = {"link_6"},
             .attach_link = "link_6"}},
        {"gimbal",
         ObstacleConfig{
             .mesh = "gimbal",
             .scale = {0.001, 0.001, 0.001},
             .compensation = coordinates.get_pose("gimbal_compensation"),
             .pose = coordinates.get_pose("gimbal"),
             .initial_operation = CollisionObjectOperation::ADD,
             .touch_links = {"link_6"},
             .attach_link = "link_6"}},
        {"exchange",
         ObstacleConfig{
             .mesh = "exchange",
             .scale = {0.001, 0.001, 0.001},
             .compensation = coordinates.get_pose("exchange_compensation"),
             .pose = {-1, 0, 0, 0, 0, 0},
             .initial_operation = CollisionObjectOperation::ADD,
             .touch_links = {"link_6"},
             .attach_link = "link_6"}},
        {"pillar",
         ObstacleConfig{
             .mesh = "pillar",
             .scale = {0.001, 0.001, 0.001},
             .compensation = coordinates.get_pose("pillar_compensation"),
             .pose = {-1, 0, 0, 0, 0, 0},
             .initial_operation = CollisionObjectOperation::ADD,
             .touch_links = {"link_6"},
             .attach_link = "link_6"}},
    };
}

inline auto make_obstacle_parameter_map() {
    return make_obstacle_parameter_map(ObstacleCoordinatesMap{});
}

inline const auto& obstacle_parameter_map() {
    static const auto parameters = make_obstacle_parameter_map();
    return parameters;
}

inline const ObstacleConfig& get_obstacle_config(const std::string& name) {
    const auto& parameters = obstacle_parameter_map();
    if (const auto entry = parameters.find(name); entry != parameters.end())
        return entry->second;
    throw std::invalid_argument("Unknown obstacle: " + name);
}

} // namespace rmcs_core::controller::arm::obstacle
