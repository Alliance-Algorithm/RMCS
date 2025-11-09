#pragma once

#include "enum/armor_id.hpp"
#include <opencv2/core/core.hpp>

namespace world_exe::data {
struct ArmorImageSpacing {
    enumeration::ArmorIdFlag id = enumeration::ArmorIdFlag::Unknow;
    /// 以ROS系，光轴方向为x, 上为z,
    /// 四个点为： 1->左上 2->右上 3->右下 4->左下,
    /// 点实际坐标为opencv系下的像素坐标
    std::vector<cv::Point2d> image_points = { {}, {}, {}, {} };
    bool isLargeArmor;
};
}