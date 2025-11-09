#pragma once

#include "data/time_stamped.hpp"

#include <Eigen/Eigen>
#include <ctime>
#include <opencv2/core/core.hpp>

namespace world_exe::data {
struct FireControl {
    data::TimeStamp time_stamp;
    /// 以ROS系，光轴方向为x, 上为z,
    /// 四个点为： 1->左上 2->右上 3->右下 4->左下,
    /// 点实际坐标为opencv系下的像素坐标
    Eigen::Vector3d gimbal_dir;
    bool fire_allowance = false;
};
}