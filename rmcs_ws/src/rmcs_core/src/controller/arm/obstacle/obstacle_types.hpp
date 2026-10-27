#pragma once

#include <array>
#include <string>
#include <vector>

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

struct ObstacleConfig {
    std::string mesh;
    std::array<double, 3> scale{0.001, 0.001, 0.001};
    Pose compensation{};
    Pose pose{};
    CollisionObjectOperation initial_operation = CollisionObjectOperation::IDLE;
    std::vector<std::string> touch_links;
    std::string attach_link; // empty: the obstacle cannot be attached
};

} // namespace rmcs_core::controller::arm::obstacle
