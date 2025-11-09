#include "utils/cast.hpp"
#include "enum/armor_id.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <enum/enum_tools.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

// 注意cv坐标系和ros的不同，我们传入传出都是ROS
void world_exe::util::cast ::armor_3d_camera_to_armor_2d_image(
    const data::ArmorCameraSpacing& armor3d, const cv::Mat& intrinsic_parameters,
    const cv::Mat& distortion_parameters, const std::vector<cv::Point3d>& points_in_armor_spacing,
    data::ArmorImageSpacing& out_armor_2d) {

    out_armor_2d.id           = armor3d.id;
    out_armor_2d.isLargeArmor = (static_cast<int>(armor3d.id)
                                    & (static_cast<int>(enumeration::ArmorIdFlag::Hero)
                                        | static_cast<int>(enumeration::ArmorIdFlag::Base)))
        != 0;
    std::vector<cv::Point3d> point3ds {};
    for (const auto& point : points_in_armor_spacing) {
        Eigen::Vector3d point_eigen { point.x, point.y, point.z };

        point_eigen = armor3d.orientation * point_eigen;
        point_eigen = point_eigen + armor3d.position;

        point3ds.emplace_back(-point_eigen.y(), -point_eigen.z(), point_eigen.x());
    }
    cv::projectPoints(point3ds, cv::Vec3d::zeros(), cv::Vec3d::zeros(), intrinsic_parameters,
        distortion_parameters, out_armor_2d.image_points);
}