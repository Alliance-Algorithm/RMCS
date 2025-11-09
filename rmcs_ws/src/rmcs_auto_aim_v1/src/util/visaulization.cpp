#include "data/armor_camera_spacing.hpp"
#include "data/armor_image_spaceing.hpp"
#include "enum/armor_id.hpp"
#include "interfaces/armor_in_camera.hpp"
#include "interfaces/armor_in_gimbal_control.hpp"
#include "utils/cast.hpp"
#include "utils/visualization.hpp"
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <cstdio>
#include <enum/enum_tools.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <string>
void draw_armor_2d(const world_exe::data::ArmorImageSpacing& armor, cv::Mat& in_out_mat) {
    cv::circle(in_out_mat, armor.image_points[0], 10, cv::Scalar(0, 255, 0), 1);
    cv::line(in_out_mat, armor.image_points[0], armor.image_points[1], cv::Scalar(255, 0, 0));
    cv::line(in_out_mat, armor.image_points[1], armor.image_points[3], cv::Scalar(0, 0, 255));
    cv::line(in_out_mat, armor.image_points[1], armor.image_points[2], cv::Scalar(0, 255, 0), 5);
    cv::line(in_out_mat, armor.image_points[2], armor.image_points[3], cv::Scalar(0, 255, 0));
    cv::line(in_out_mat, armor.image_points[3], armor.image_points[0], cv::Scalar(0, 255, 0), 5);
}

std::string get_enum_name(int i) {
    std::string txt = "Unknow";
    using namespace world_exe::enumeration;
    switch (static_cast<world_exe::enumeration::ArmorIdFlag>(1 << i)) {

    case ArmorIdFlag::Hero:
        txt = enum_name<ArmorIdFlag::Hero>();
        break;
    case ArmorIdFlag::Engineer:
        txt = enum_name<ArmorIdFlag::Engineer>();
        break;
    case ArmorIdFlag::InfantryIII:
        txt = enum_name<ArmorIdFlag::InfantryIII>();
        break;
    case ArmorIdFlag::InfantryIV:
        txt = enum_name<ArmorIdFlag::InfantryIV>();
        break;
    case ArmorIdFlag::InfantryV:
        txt = enum_name<ArmorIdFlag::InfantryV>();
        break;
    case ArmorIdFlag::Sentry:
        txt = enum_name<ArmorIdFlag::Sentry>();
        break;
    case ArmorIdFlag::Outpost:
        txt = enum_name<ArmorIdFlag::Outpost>();
        break;
    case ArmorIdFlag::Base:
        txt = enum_name<ArmorIdFlag::Base>();
        break;
    default:
        txt = "Unknow";
    };
    return txt;
}
// line from 1 -> 2 -> 4, green circle for 1, blue for 1->2 ref for 2 -> 4
void world_exe::util::visualization ::draw_armor_in_image(
    const interfaces::IArmorInImage& image_armors, cv::Mat& in_out_mat) {
    for (int i = 0; i < static_cast<uint32_t>(enumeration::ArmorIdFlag::Count); i++)
        for (const auto& armor :
            image_armors.GetArmors(static_cast<enumeration::ArmorIdFlag>(1 << i))) {
            draw_armor_2d(armor, in_out_mat);
            cv::putText(in_out_mat, get_enum_name(i), armor.image_points[0] + cv::Point2d(-10, -10),
                1, 1, cv::Scalar(255, 255, 255));
        }
}
// line from 1 -> 2 -> 4, green circle for 1, blue for 1->2 ref for 2 -> 4
void world_exe::util::visualization ::draw_armor_in_camera(
    const interfaces::IArmorInCamera& camera_armor, const cv::Mat& intrinsic_parameters,
    const cv::Mat& distortion_parameters, const std::vector<cv::Point3d>& points_in_armor_spacing,
    cv::Mat& in_out_mat) {
    for (int i = 0; i < static_cast<uint32_t>(enumeration::ArmorIdFlag::Count); i++)
        for (const auto& armor :
            camera_armor.GetArmors(static_cast<enumeration::ArmorIdFlag>(1 << i))) {
            data::ArmorImageSpacing img_armor { };
            world_exe::util::cast::armor_3d_camera_to_armor_2d_image(armor, intrinsic_parameters,
                distortion_parameters, points_in_armor_spacing, img_armor);
            draw_armor_2d(img_armor, in_out_mat);
            cv::putText(in_out_mat, get_enum_name(i),
                img_armor.image_points[0] + cv::Point2d(-10, -10), 1, 1, cv::Scalar(255, 255, 255));
        }
}

void world_exe::util::visualization::draw_armor_in_gimbal(
    const interfaces::IArmorInGimbalControl& camera_armor, const cv::Mat& intrinsic_parameters,
    const cv::Mat& distortion_parameters, const std::vector<cv::Point3d>& points_in_armor_spacing,
    const Eigen::Affine3d gimal_to_camera, cv::Mat& in_out_mat) {
    for (int i = 0; i < static_cast<uint32_t>(world_exe::enumeration::ArmorIdFlag::Count); i++)
        for (const auto& armor_gimbal :
            camera_armor.GetArmors(static_cast<world_exe::enumeration::ArmorIdFlag>(1 << i))) {
            world_exe::data::ArmorImageSpacing img_armor { };
            world_exe::data::ArmorCameraSpacing armor { armor_gimbal.id,
                gimal_to_camera * armor_gimbal.position,
                Eigen::Quaterniond {
                    gimal_to_camera.rotation() * armor_gimbal.orientation.toRotationMatrix() } };
            world_exe::util::cast::armor_3d_camera_to_armor_2d_image(armor, intrinsic_parameters,
                distortion_parameters, points_in_armor_spacing, img_armor);
            draw_armor_2d(img_armor, in_out_mat);
            cv::putText(in_out_mat, get_enum_name(i),
                img_armor.image_points[0] + cv::Point2d(-10, -10), 1, 1, cv::Scalar(255, 255, 255));
        }
}