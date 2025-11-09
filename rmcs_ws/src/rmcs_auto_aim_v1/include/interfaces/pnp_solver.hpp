#pragma once

#include "interfaces/armor_in_camera.hpp"
#include "interfaces/armor_in_image.hpp"
namespace world_exe::interfaces {

/**
 * @brief Pnp求解器
          指一切可以通过2维装甲板求解3维装甲板的算法器
 */
class IPnpSolver {
public:
    /**
     * @brief 求解装甲板的pnp问题
     * @param std::shared_ptr<interfaces::IArmorInImage> 二维装甲板
     * @return std::shared_ptr<world_exe::interfaces::IArmorInCamera> 三维装甲板
     */
    virtual std::shared_ptr<world_exe::interfaces::IArmorInCamera> SolvePnp(
        std::shared_ptr<interfaces::IArmorInImage>) = 0;

    virtual ~IPnpSolver() = default;
};
}