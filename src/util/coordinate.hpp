#pragma once

#include <Eigen/Dense>

namespace world_exe::util::coordinate {

static inline Eigen::Vector3d opencv2ros_position(const Eigen::Vector3d& position) {
    return Eigen::Vector3d(position.z(), -position.x(), -position.y());
}

// clangd-format off
static inline Eigen::Matrix3d opencv2ros_rotation(const Eigen::Matrix3d& rotation) {
    Eigen::Matrix3d t;
    t << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    return t * rotation * t.transpose();
}
// clangd-format on

static inline Eigen::Vector3d ros2opencv_position(const Eigen::Vector3d& position) {
    return Eigen::Vector3d(-position.y(), -position.z(), position.x());
}

static inline Eigen::Matrix3d ros2opencv_rotation(const Eigen::Matrix3d& rotation) {
    // clangd-format off
    Eigen::Matrix3d t;
    t << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    // clangd-format on
    return t.transpose() * rotation * t;
}

}