#pragma once

#include "data/armor_gimbal_control_spacing.hpp"
#include "enum/armor_id.hpp"
#include "data/time_stamped.hpp"
#include <opencv2/core/mat.hpp>

namespace world_exe::interfaces {

/**
 * @brief
    某一确定时刻
    云台控制指令使用的坐标系（世界坐标系）下
    装甲板集合
 */
class IArmorInGimbalControl {

public:
    /// 获取时间戳，标志其内容装甲板的准确时间点
    virtual const data::TimeStamp& GetTimeStamp() const = 0;

    /// 获取某个车辆ID的装甲板集合
    virtual const std::vector<data::ArmorGimbalControlSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const = 0;

    virtual ~IArmorInGimbalControl() = default;
};
}