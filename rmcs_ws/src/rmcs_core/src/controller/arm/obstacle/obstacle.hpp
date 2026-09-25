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
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <string>
namespace rmcs_core::controller::arm::obstracle {

enum struct CollisionObjectStatus {
    ADD,
    APPEND,
    MOVE,
    REMOVE,
};

struct Pose {
    double x, y, z, roll, pitch, yaw;
};

class Obstacle
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit Obstacle(std::string frame_id, std::string id)
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        collision_object_.header.frame_id = std::move(frame_id);
        collision_object_.id = std::move(id);
    }
    void set_pose(Pose pose) { pose_solver(pose); }
    void set_enable(bool enable) { enable_ = enable ? true : false; }
    void mesh_load(const std::string& filename) {
        const auto path =
            std::filesystem::path(ament_index_cpp::get_package_share_directory("rmcs_core"))
            / "meshs" / filename;

        std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(path.string()));
        if (!mesh) {
            RCLCPP_INFO(this->get_logger(), "Mesh failed to load!");
            return;
        }

        shapes::ShapeMsg shape_msg;
        if (!shapes::constructMsgFromShape(mesh.get(), shape_msg)) {
            RCLCPP_INFO(this->get_logger(), "Failed to construct msg from shape!");
            return;
        }

        collision_object_.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(shape_msg));
    }

    auto export_collision() {

        using CollisionObject = moveit_msgs::msg::CollisionObject;

        collision_object_.set__pose(pose_);
        switch (collision_object_status_)
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

    geometry_msgs::msg::Pose pose_;
    CollisionObjectStatus collision_object_status_;
    moveit_msgs::msg::CollisionObject collision_object_;

    bool enable_{false};
};

} // namespace rmcs_core::controller::arm::obstracle