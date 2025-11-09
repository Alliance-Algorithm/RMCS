#pragma once

#include "data/time_stamped.hpp"

#include <ctime>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>

namespace world_exe::data {
struct MatStamped {
    data::TimeStamp stamp{};
    cv::Mat mat {};
    
    MatStamped()                                = default;
    MatStamped(const MatStamped&)               = delete;
    MatStamped& operator=(const MatStamped&)    = delete;
    
    MatStamped(cv::Mat&& image, data::TimeStamp stamp)
    {
        mat = image;
        stamp = stamp;
    }

    void Load(const cv::Mat& image, data::TimeStamp now)
    {
        image.copyTo(mat);
        stamp = now;
    }
};
}