#pragma once

#include "enum/car_id.hpp"
#include "interfaces/armor_in_image.hpp"
#include <memory>
#include <opencv2/core/mat.hpp>

namespace world_exe::interfaces {

/**
 * @brief 识别器
 */
class IIdentifier {
public:
    /**
     * @brief 识别传入画面中所有的装甲板及其ID
     *
     * @param input_image 相机捕捉到的画面
     * @return const std::tuple<const std::shared_ptr<IArmorInImage>, enumeration::CarIDFlag>
     */
    virtual const std::tuple<const std::shared_ptr<IArmorInImage>, enumeration::CarIDFlag> identify(
        const cv::Mat& input_image) = 0;

    virtual ~IIdentifier() = default;
};
}