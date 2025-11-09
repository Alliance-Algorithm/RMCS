#pragma once

#include <memory>
#include <opencv2/core/mat.hpp>
namespace world_exe::parameters {
class HikCameraProfile {
public:
    static void set_intrinsic_matrix(const double& fx, const double& fy, const double& cx,
        const double& cy, const double& k1, const double& k2, const double& k3);
    static void set_width_height(const int& width, const int& height);
    static const cv::Mat& get_intrinsic_parameters();
    static const cv::Mat& get_distortion_parameters();
    static const int& get_width();
    static const int& get_height();

private:
    struct Impl;

    static std::unique_ptr<Impl> impl_;
};
}