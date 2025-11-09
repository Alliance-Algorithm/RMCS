#pragma once

#include "armor_in_gimbal_control.hpp"
#include "data/time_stamped.hpp"
#include "enum/armor_id.hpp"
#include <ctime>

namespace world_exe::interfaces {
/**
 * @brief 预测器，通常通过某一个时刻的状态向量x 及其状态转移矩阵: x' = Ax + Bu 获取未来状态向量x'
 * 继承自这个接口的类一般是Record, 创建后内容不会改变
 */
class IPredictor {
public:
    /**
     * @brief 获取当前预测器可以进行预测的所有车辆ID
     *
     * @return const enumeration::ArmorIdFlag&
     */
    virtual const enumeration::ArmorIdFlag& GetId() const = 0;
    /**
     * @brief 求解出未来某个时间的装甲板集合
     *
     * @param time_stamp 未来的某个时间点
     * @return const IArmorInGimbalControl&
     */
    virtual std::shared_ptr<IArmorInGimbalControl> Predictor(
        const data::TimeStamp& time_stamp) const = 0;

    virtual ~IPredictor() = default;
};
}