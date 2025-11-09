#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

namespace world_exe::interfaces {

/**
 * @brief 可以进行可视化调试，提供绘制方法
 */
class IDrawable {
public:
    virtual void Draw(cv::InputArray, cv::OutputArray) = 0;
    virtual void Draw(cv::InputOutputArray)            = 0;

    virtual ~IDrawable() = default;
};
}