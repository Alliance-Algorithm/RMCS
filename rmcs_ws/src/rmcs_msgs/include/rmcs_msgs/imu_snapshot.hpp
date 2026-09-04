#pragma once

#include <eigen3/Eigen/Geometry>
#include <rmcs_msgs/board_clock.hpp>

namespace rmcs_msgs {

struct ImuSnapshot {
    Eigen::Quaterniond orientation;
    Eigen::Vector3d gyro_body;
    rmcs_msgs::BoardClock::time_point timestamp;
};

} // namespace rmcs_msgs
