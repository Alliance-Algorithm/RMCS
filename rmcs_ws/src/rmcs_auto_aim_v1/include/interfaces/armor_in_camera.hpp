#pragma once

#include "data/armor_camera_spacing.hpp"
#include "data/time_stamped.hpp"
#include "enum/armor_id.hpp"
#include "data/time_stamped.hpp"
#include <opencv2/core/mat.hpp>


namespace world_exe::interfaces {

/**
 * @brief
    某一确定时刻
    相机坐标系下
    装甲板集合
 */
class IArmorInCamera {
public:
    /// 获取时间戳，标志其内容装甲板的准确时间点
    virtual const data::TimeStamp& GetTimeStamp() const = 0;

    /// 获取某个车辆ID的装甲板集合
    virtual const std::vector<data::ArmorCameraSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const = 0;

    virtual ~IArmorInCamera() = default;
};
}