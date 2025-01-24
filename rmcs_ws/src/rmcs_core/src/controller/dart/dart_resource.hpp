
#pragma once

#include <opencv2/core/mat.hpp>

namespace rmcs_core::controller::dart {

enum class ControllMode { Ban, Enable, Auto, Manual };

struct CameraFrame {
    cv::Mat image;
    long id;

    void init() {
        image = cv::Mat(720, 1440, CV_8UC3, cv::Scalar(0, 0, 0));
        id    = -1;
    }
};

} // namespace rmcs_core::controller::dart