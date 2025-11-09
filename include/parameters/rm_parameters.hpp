#pragma once

#include <opencv2/core/types.hpp>
#include <vector>
namespace world_exe::parameters {
class Robomaster {
public:
    inline constexpr static const double NormalArmorWidth = 0.134, NormalArmorHeight = 0.056,
                                         LargerArmorWidth = 0.230, LargerArmorHeight = 0.056;

    inline static const std::vector<cv::Point3d> LargeArmorObjectPointsRos = {
        cv::Point3d(0, 0.5 * LargerArmorWidth, 0.5 * LargerArmorHeight),
        cv::Point3d(0, -0.5 * LargerArmorWidth, 0.5 * LargerArmorHeight),
        cv::Point3d(0, -0.5 * LargerArmorWidth, -0.5 * LargerArmorHeight),
        cv::Point3d(0, 0.5 * LargerArmorWidth, -0.5 * LargerArmorHeight)
    };

    inline static const std::vector<cv::Point3d> NormalArmorObjectPointsRos = {
        cv::Point3d(0, 0.5 * NormalArmorWidth, 0.5 * NormalArmorHeight),
        cv::Point3d(0, -0.5 * NormalArmorWidth, 0.5 * NormalArmorHeight),
        cv::Point3d(0, -0.5 * NormalArmorWidth, -0.5 * NormalArmorHeight),
        cv::Point3d(0, 0.5 * NormalArmorWidth, -0.5 * NormalArmorHeight)
    };
    inline static const std::vector<cv::Point3d> LargeArmorObjectPointsOpencv = {
        cv::Point3d(-0.5 * LargerArmorWidth, -0.5 * LargerArmorHeight, 0),
        cv::Point3d(0.5 * LargerArmorWidth, -0.5 * LargerArmorHeight, 0),
        cv::Point3d(0.5 * LargerArmorWidth, 0.5 * LargerArmorHeight, 0),
        cv::Point3d(-0.5 * LargerArmorWidth, 0.5 * LargerArmorHeight, 0)
    };

    inline static const std::vector<cv::Point3d> NormalArmorObjectPointsOpencv = {
        cv::Point3d(-0.5 * NormalArmorWidth, -0.5 * NormalArmorHeight, 0),
        cv::Point3d(0.5 * NormalArmorWidth, -0.5 * NormalArmorHeight, 0),
        cv::Point3d(0.5 * NormalArmorWidth, 0.5 * NormalArmorHeight, 0),
        cv::Point3d(-0.5 * NormalArmorWidth, 0.5 * NormalArmorHeight, 0)
    };
};
}