#pragma once

#include "obstacle/obstacle.hpp"
#include <Eigen/Geometry>
namespace rmcs_core::controller::arm {

class AxisTransform {
public:
    struct Result {
        Eigen::Vector3d point;
        Eigen::Vector3d direction;
    };

    AxisTransform(Eigen::Vector3d direction, Eigen::Vector3d point)
        : direction_(direction)
        , point_(point) {}

    // Radians, fixed-axis roll then pitch then yaw; translate after rotating.
    Result transform(obstacle::Pose pose) const {
        double x = pose.x;
        double y = pose.y;
        double z = pose.z;
        double roll = pose.roll;
        double pitch = pose.pitch;
        double yaw = pose.yaw;
        const Eigen::Quaterniond rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                                          * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                                          * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        return {
            .point = rotation * point_ + Eigen::Vector3d(x, y, z),
            .direction = (rotation * direction_).normalized()};
    }

private:
    Eigen::Vector3d direction_;
    Eigen::Vector3d point_;
};

} // namespace rmcs_core::controller::arm
