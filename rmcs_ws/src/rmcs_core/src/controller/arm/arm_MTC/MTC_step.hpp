#pragma once

#include "controller/arm/obstacle/obstacle_parameter_map.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <cstddef>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace rmcs_core::controller::arm::MTC {

struct PoseTarget {
    std::string frame_id;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

struct DirectionTarget {
    std::string frame_id;
    double dir_x = 0.0;
    double dir_y = 0.0;
    double dir_z = 0.0;
};

inline geometry_msgs::msg::PoseStamped makePoseGoal(PoseTarget target) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = std::move(target.frame_id);
    pose.pose.position.x = target.x;
    pose.pose.position.y = target.y;
    pose.pose.position.z = target.z;
    tf2::Quaternion orientation;
    orientation.setRPY(target.roll, target.pitch, target.yaw);
    pose.pose.orientation.x = orientation.x();
    pose.pose.orientation.y = orientation.y();
    pose.pose.orientation.z = orientation.z();
    pose.pose.orientation.w = orientation.w();
    return pose;
}

inline geometry_msgs::msg::Vector3 makeVector3(double value_x, double value_y, double value_z) {
    geometry_msgs::msg::Vector3 vector;
    vector.x = value_x;
    vector.y = value_y;
    vector.z = value_z;
    return vector;
}

inline geometry_msgs::msg::Vector3Stamped makeDirection(DirectionTarget target) {
    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = std::move(target.frame_id);
    direction.vector = makeVector3(target.dir_x, target.dir_y, target.dir_z);
    return direction;
}

struct PipelineConfig {
    std::string pipeline = "ompl";
    std::string planner = "BiTRRTkConfigDefault";
};

struct JointInterpolationConfig {};

struct CartesianPathConfig {
    double step_size = 0.01;
};

using SolverConfig = std::variant<PipelineConfig, JointInterpolationConfig, CartesianPathConfig>;

struct MotionConfig {
    SolverConfig solver = PipelineConfig{};
    double velocity_scaling = 0.3;
    double acceleration_scaling = 0.1;
    double timeout = 5.0;
};

struct CurrentStateConfig {
    double timeout = 3.0;
};

struct CollisionObjectConfig {
    std::string id;
    geometry_msgs::msg::PoseStamped pose;
    geometry_msgs::msg::Vector3 dimensions;
};

struct FixedStateConfig {
    std::map<std::string, double> joints;
    std::vector<CollisionObjectConfig> objects;
};

using MoveToGoal =
    std::variant<std::string, std::map<std::string, double>, geometry_msgs::msg::PoseStamped>;

struct MoveToConfig {
    MoveToGoal goal;
    MotionConfig motion;
    std::string group;
    std::string ik_frame;
    bool use_ik_frame = true;
};

struct MoveRelativeConfig {
    geometry_msgs::msg::Vector3Stamped direction;
    double min_distance = 0.0;
    double max_distance = 0.1;
    MotionConfig motion{.solver = PipelineConfig{"pilz_industrial_motion_planner", "LIN"}};
    std::string group;
    std::string ik_frame;
};

struct AllowCollisionsConfig {
    std::string object;
    std::vector<std::string> links;
    bool allow = true;
};

struct AttachObjectConfig {
    std::string object;
    std::string link;
    std::vector<std::string> touch_links{};
};

struct DetachObjectConfig {
    std::string object;
    std::string link;
};

using SceneOperation = std::variant<AllowCollisionsConfig, AttachObjectConfig, DetachObjectConfig>;

struct ModifyPlanningSceneConfig {
    std::vector<SceneOperation> operations;
};

enum class GripperAction { Open, Close };

struct GripperCommandConfig {
    GripperAction action = GripperAction::Open;
    double timeout = 3.0;
};

struct StageConfig {
    std::string name;
    std::variant<
        CurrentStateConfig, FixedStateConfig, MoveToConfig, MoveRelativeConfig,
        ModifyPlanningSceneConfig, GripperCommandConfig>
        config;
};

class Step {
public:
    static StageConfig makeCurrentState(std::string name, CurrentStateConfig config) {
        return {std::move(name), std::move(config)};
    }

    static StageConfig makeFixedState(std::string name, FixedStateConfig config) {
        return {std::move(name), std::move(config)};
    }

    static StageConfig makeMoveTo(std::string name, MoveToConfig config) {
        return {std::move(name), std::move(config)};
    }

    static StageConfig makeMoveRelative(std::string name, MoveRelativeConfig config) {
        return {std::move(name), std::move(config)};
    }

    static StageConfig makeModifyPlanningScene(std::string name, ModifyPlanningSceneConfig config) {
        return {std::move(name), std::move(config)};
    }

    static StageConfig makeOpenGripper(std::string name, double timeout = 3.0) {
        return {std::move(name), GripperCommandConfig{GripperAction::Open, timeout}};
    }

    static StageConfig makeCloseGripper(std::string name, double timeout = 3.0) {
        return {std::move(name), GripperCommandConfig{GripperAction::Close, timeout}};
    }

    static StageConfig makeAllowCollisions(std::string object) {
        auto name = "allow_collisions_" + object;
        return makeAllowCollisions(std::move(name), std::move(object));
    }

    static StageConfig makeAllowCollisions(std::string name, std::string object) {
        const auto& config = obstacle::get_obstacle_config(object);
        return makeModifyPlanningScene(
            std::move(name),
            ModifyPlanningSceneConfig{
                .operations = {AllowCollisionsConfig{
                    .object = std::move(object), .links = config.touch_links, .allow = true}}});
    }

    static StageConfig makeForbidCollisions(std::string object) {
        auto name = "forbid_collisions_" + object;
        return makeForbidCollisions(std::move(name), std::move(object));
    }

    static StageConfig makeForbidCollisions(std::string name, std::string object) {
        const auto& config = obstacle::get_obstacle_config(object);
        return makeModifyPlanningScene(
            std::move(name),
            ModifyPlanningSceneConfig{
                .operations = {AllowCollisionsConfig{
                    .object = std::move(object), .links = config.touch_links, .allow = false}}});
    }

    static StageConfig makeAttachObject(std::string object) {
        auto name = "attach_" + object;
        return makeAttachObject(std::move(name), std::move(object));
    }

    static StageConfig makeAttachObject(std::string name, std::string object) {
        const auto& config = obstacle::get_obstacle_config(object);
        if (config.attach_link.empty())
            throw std::invalid_argument(name + ": obstacle is not attachable: " + object);
        return makeModifyPlanningScene(
            std::move(name), ModifyPlanningSceneConfig{
                                 .operations = {AttachObjectConfig{
                                     .object = std::move(object),
                                     .link = config.attach_link,
                                     .touch_links = config.touch_links}}});
    }

    static StageConfig makeDetachObject(std::string object) {
        auto name = "detach_" + object;
        return makeDetachObject(std::move(name), std::move(object));
    }

    static StageConfig makeDetachObject(std::string name, std::string object) {
        const auto& config = obstacle::get_obstacle_config(object);
        if (config.attach_link.empty())
            throw std::invalid_argument(name + ": obstacle is not attachable: " + object);
        return makeModifyPlanningScene(
            std::move(name), ModifyPlanningSceneConfig{
                                 .operations = {DetachObjectConfig{
                                     .object = std::move(object), .link = config.attach_link}}});
    }
};

enum class ContainerType { Serial, Alternatives, Fallbacks };

struct NodeConfig;

struct ContainerConfig {
    std::string name;
    ContainerType type = ContainerType::Serial;
    std::vector<NodeConfig> children;
};

struct NodeConfig {
    std::variant<StageConfig, ContainerConfig> config;

    NodeConfig(StageConfig stage)
        : config(std::move(stage)) {}
    NodeConfig(ContainerConfig container)
        : config(std::move(container)) {}
};

struct TaskConfig {
    std::string name;
    std::string group = "alliance_arm";
    std::string ik_frame = "link_6";
    std::vector<NodeConfig> children;
    std::size_t max_solutions = 1;
    double timeout = 30.0;
};

} // namespace rmcs_core::controller::arm::MTC
