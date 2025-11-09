#pragma once

#include <opencv2/core/mat.hpp>

namespace world_exe::tests::mock {
class MockHikCameraProfile{
    public:
        static const cv::Mat& get_intrinsic_parameters(){
            static cv::Mat intrinsic = (cv::Mat_<double>(3,3) << 800, 0, 320, 0, 800, 240, 0, 0, 1);
            return intrinsic;
        }
        static const cv::Mat& get_distortion_parameters(){
            static cv::Mat distortion = (cv::Mat_<double>(1,5) << 0.1, -0.05, 0, 0, 0);
            return distortion;
            
        }
};
}