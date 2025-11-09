#pragma once

#include "interfaces/armor_in_camera.hpp"
#include "interfaces/armor_in_gimbal_control.hpp"
#include "interfaces/armor_in_image.hpp"
#include <opencv2/core/mat.hpp>
namespace world_exe::util::visualization {
void draw_armor_in_image(const interfaces::IArmorInImage& image, cv::Mat& InputOutputArray);

void draw_armor_in_camera(const interfaces::IArmorInCamera& camera_armor,
    const cv::Mat& intrinsic_parameters, const cv::Mat& distortion_parameters,
    const std::vector<cv::Point3d>& points_in_armor_spacing, cv::Mat& in_out_mat);
void draw_armor_in_gimbal(
    const world_exe::interfaces::IArmorInGimbalControl& camera_armor, const cv::Mat& intrinsic_parameters,
    const cv::Mat& distortion_parameters, const std::vector<cv::Point3d>& points_in_armor_spacing,
    const Eigen::Affine3d gimal_to_camera,
    cv::Mat& in_out_mat);
}
    