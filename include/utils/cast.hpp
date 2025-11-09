#pragma once

#include "data/armor_camera_spacing.hpp"
#include "data/armor_image_spaceing.hpp"
#include <opencv2/core/mat.hpp>
namespace world_exe::util::cast {
void armor_3d_camera_to_armor_2d_image(const data::ArmorCameraSpacing& armor3d,
    const cv::Mat& intrinsic_parameters, const cv::Mat& distortion_parameters,
    const std::vector<cv::Point3d>& points_in_armor_spacing, data::ArmorImageSpacing& out_armor_2d);

}