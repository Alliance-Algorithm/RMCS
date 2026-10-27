#pragma once

#include "obstacle_types.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/mesh.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <filesystem>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace rmcs_core::controller::arm::obstacle {

class Obstacle {
public:
    explicit Obstacle(
        std::string frame_id, std::string id, std::string attach_link = {},
        std::vector<std::string> touch_links = {})
        : logger_(rclcpp::get_logger("obstacle_" + id))
        , attach_link_(std::move(attach_link))
        , touch_links_(std::move(touch_links)) {
        collision_object_.header.frame_id = std::move(frame_id);
        collision_object_.id = std::move(id);
    }

    std::string id() const { return collision_object_.id; }

    bool attachable() const { return !attach_link_.empty(); }

    const std::string& attach_link() const { return attach_link_; }

    bool attached() const { return attached_; }

    void set_attached(bool attached) { attached_ = attached; }

    // Pose reported by the planning scene, compensation already included
    void set_world_pose(const geometry_msgs::msg::Pose& pose) { pose_ = pose; }

    void sync_world(const moveit_msgs::msg::CollisionObject& object) {
        collision_object_ = object;
        pose_ = object.pose;
        // A full world snapshot is a replacement, not a pending append or removal.
        collision_object_operation_ = CollisionObjectOperation::ADD;
        attached_ = false;
    }

    void set_pose(Pose pose) {
        pose.x += pose_compensation_.x;
        pose.y += pose_compensation_.y;
        pose.z += pose_compensation_.z;
        pose.roll += pose_compensation_.roll;
        pose.pitch += pose_compensation_.pitch;
        pose.yaw += pose_compensation_.yaw;
        pose_solver(pose);
    }

    void set_pose_compensation(Pose pose_compensation) { pose_compensation_ = pose_compensation; }

    void set_operation(CollisionObjectOperation collision_object_operation) {
        collision_object_operation_ = collision_object_operation;
    }

    void mesh_load(const std::string& filename, const Eigen::Vector3d& scale) {
        const auto path =
            std::filesystem::path(ament_index_cpp::get_package_share_directory("rmcs_core"))
            / "meshes" / (filename + ".stl");

        const std::string uri = "file://" + path.string();

        std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(uri, scale));
        if (!mesh || mesh->vertex_count == 0 || mesh->triangle_count == 0)
            throw std::runtime_error("Failed to load nonempty obstacle mesh: " + uri);

        RCLCPP_INFO(logger_, "Mesh loading successful!");

        shapes::ShapeMsg shape_msg;
        if (!shapes::constructMsgFromShape(mesh.get(), shape_msg))
            throw std::runtime_error("Failed to construct obstacle mesh message: " + uri);

        RCLCPP_INFO(logger_, "Construct msg successful!");

        collision_object_.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(shape_msg));
        geometry_msgs::msg::Pose mesh_pose;
        mesh_pose.orientation.w = 1.0;
        collision_object_.mesh_poses.push_back(mesh_pose);
    }

    auto export_collision() {

        using CollisionObject = moveit_msgs::msg::CollisionObject;

        if (collision_object_operation_ == CollisionObjectOperation::ADD
            || collision_object_operation_ == CollisionObjectOperation::APPEND
            || collision_object_operation_ == CollisionObjectOperation::MOVE) {
            if (collision_object_.meshes.empty() && collision_object_.primitives.empty()
                && collision_object_.planes.empty())
                throw std::runtime_error("Missing obstacle geometry: " + id());
            for (const auto& mesh : collision_object_.meshes)
                if (mesh.vertices.empty() || mesh.triangles.empty())
                    throw std::runtime_error("Empty obstacle mesh: " + id());
            for (const auto& primitive : collision_object_.primitives)
                if (primitive.dimensions.empty())
                    throw std::runtime_error("Empty obstacle primitive: " + id());
        }

        std::string collision_object_operation_str;

        collision_object_.set__pose(pose_);

        switch (collision_object_operation_) {
        case CollisionObjectOperation::ADD:
            collision_object_.operation = CollisionObject::ADD;
            collision_object_operation_str = "ADD";
            break;
        case CollisionObjectOperation::APPEND:
            collision_object_.operation = CollisionObject::APPEND;
            collision_object_operation_str = "APPEND";
            break;
        case CollisionObjectOperation::MOVE:
            collision_object_.operation = CollisionObject::MOVE;
            collision_object_operation_str = "MOVE";
            break;
        case CollisionObjectOperation::REMOVE:
        case CollisionObjectOperation::IDLE:
        default:
            collision_object_.operation = CollisionObject::REMOVE;
            collision_object_operation_str = "REMOVE";
            break;
        }

        RCLCPP_INFO(logger_, "collision operation: %s", collision_object_operation_str.c_str());

        return collision_object_;
    }

    // No geometry: MoveIt moves the existing world object onto the link
    auto export_attach() const {
        moveit_msgs::msg::AttachedCollisionObject attached_object;
        attached_object.link_name = attach_link_;
        attached_object.object.header.frame_id = collision_object_.header.frame_id;
        attached_object.object.id = collision_object_.id;
        attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
        attached_object.touch_links = touch_links_;

        RCLCPP_INFO(logger_, "attach operation: ADD to %s", attach_link_.c_str());

        return attached_object;
    }

    // MoveIt returns the detached object to the world at its current pose
    auto export_detach() const {
        moveit_msgs::msg::AttachedCollisionObject attached_object;
        attached_object.link_name = attach_link_;
        attached_object.object.id = collision_object_.id;
        attached_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

        RCLCPP_INFO(logger_, "attach operation: REMOVE from %s", attach_link_.c_str());

        return attached_object;
    }

private:
    void pose_solver(Pose pose) {
        double half_roll = pose.roll / 2;
        double half_pitch = pose.pitch / 2;
        double half_yaw = pose.yaw / 2;
        double x = pose.x;
        double y = pose.y;
        double z = pose.z;

        auto cr = cos(half_roll);
        auto cp = cos(half_pitch);
        auto cy = cos(half_yaw);
        auto sr = sin(half_roll);
        auto sp = sin(half_pitch);
        auto sy = sin(half_yaw);

        pose_.orientation.set__w(cr * cp * cy + sr * sp * sy);
        pose_.orientation.set__x(sr * cp * cy - cr * sp * sy);
        pose_.orientation.set__y(cr * sp * cy + sr * cp * sy);
        pose_.orientation.set__z(cr * cp * sy - sr * sp * cy);

        pose_.position.set__x(x);
        pose_.position.set__y(y);
        pose_.position.set__z(z);
    }

    rclcpp::Logger logger_;

    std::string attach_link_;
    std::vector<std::string> touch_links_;
    bool attached_{false};

    Pose pose_compensation_{0, 0, 0, 0, 0, 0};

    geometry_msgs::msg::Pose pose_;
    CollisionObjectOperation collision_object_operation_{CollisionObjectOperation::IDLE};
    moveit_msgs::msg::CollisionObject collision_object_;
};

} // namespace rmcs_core::controller::arm::obstacle
