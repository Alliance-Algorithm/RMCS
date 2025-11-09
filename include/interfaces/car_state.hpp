#pragma once

#include "enum/car_id.hpp"
#include <opencv2/core/mat.hpp>

namespace world_exe::interfaces {

/**
 * @brief
 ** 确定自瞄系统中某个ID代表的车辆是否可以进行云台锁定、开火
 * @todo 命名鬼才，可能要改下命名
 */
class ICarState {

public:
    /**
     * @brief
        获取可以开火的车辆ID
        示例： 某辆车在镜头中出现的时间够长，确定已经被自瞄系统追踪上
     */
    virtual const enumeration::CarIDFlag& GetAllowdToFires() const = 0;

    virtual ~ICarState() = default;
};
}