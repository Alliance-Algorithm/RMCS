#pragma once

#include "armor_in_gimbal_control.hpp"
#include "data/time_stamped.hpp"
#include "enum/armor_id.hpp"
#include "interfaces/predictor.hpp"
#include <ctime>
#include <memory>

namespace world_exe::interfaces {

/**
 * @brief 类似IPredictor但是其内容不保证稳定
 *
 */
class ITargetPredictor {
public:
    /**
     * @brief
     *
     * @param id
     * @param time_stamp
     * @return std::shared_ptr<interfaces::IArmorInGimbalControl>
     */
    virtual std::shared_ptr<interfaces::IArmorInGimbalControl> Predict(
        const enumeration::ArmorIdFlag& id, const data::TimeStamp& time_stamp) = 0;

    /**
     * @brief 按照传入的id生成IPredictor
     *
     * @param id
     * @return const IPredictor&
     */
    virtual std::shared_ptr<IPredictor> GetPredictor(const enumeration::ArmorIdFlag& id) const = 0;

    virtual ~ITargetPredictor() = default;
};
}