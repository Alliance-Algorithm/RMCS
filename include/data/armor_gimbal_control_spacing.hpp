#pragma once

#include "enum/armor_id.hpp"

#include <eigen3/Eigen/Eigen>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace world_exe::data {
struct ArmorGimbalControlSpacing {
    enumeration::ArmorIdFlag id = enumeration::ArmorIdFlag::Unknow;
    /// 以ROS系，光轴方向为x, 上为z,
    /// 四个点为： 1->左上 2->右上 3->右下 4->左下,
    /// 点实际坐标云台控制向量坐标系下的笛卡尔坐标，单位米（m）,
    /// 原点位于光心
    Eigen::Vector3d position;
    /// 基础以控制向量坐标系
    /// 装甲以控制向量坐标系，数字向车为x, 短轴数字向上为z
    /// 表示 基础-》装甲板
    Eigen::Quaterniond orientation;
};
}