#pragma once
#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/mesh.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <string>

namespace rmcs_core::controller::arm::obstacle {

enum class CollisionObjectOperation {
    IDLE,
    ADD,
    APPEND,
    MOVE,
    REMOVE,
};

struct Pose {
    double x, y, z, roll, pitch, yaw;
};

class Obstacle {
public:
    explicit Obstacle(std::string frame_id, std::string id)
        : logger_(rclcpp::get_logger("obstacle_" + id)) {
        collision_object_.header.frame_id = std::move(frame_id);
        collision_object_.id = std::move(id);
    }

    std::string id() const { return collision_object_.id; }

    void set_pose(Pose pose) { pose_solver(pose); }

    void set_operation(CollisionObjectOperation collision_object_operation) {
        collision_object_operation_ = collision_object_operation;
    }

    void mesh_load(const std::string& filename) {
        const auto path =
            std::filesystem::path(ament_index_cpp::get_package_share_directory("rmcs_core"))
            / "meshes" / (filename + ".stl");

        std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(path.string()));
        if (!mesh) {
            RCLCPP_INFO(logger_, "Mesh failed to load!");
            return;
        }

        shapes::ShapeMsg shape_msg;
        if (!shapes::constructMsgFromShape(mesh.get(), shape_msg)) {
            RCLCPP_INFO(logger_, "Failed to construct msg from shape!");
            return;
        }

        collision_object_.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(shape_msg));
    }

    auto export_collision() {

        using CollisionObject = moveit_msgs::msg::CollisionObject;

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

    geometry_msgs::msg::Pose pose_;
    CollisionObjectOperation collision_object_operation_{CollisionObjectOperation::IDLE};
    moveit_msgs::msg::CollisionObject collision_object_;
};

} // namespace rmcs_core::controller::arm::obstacle