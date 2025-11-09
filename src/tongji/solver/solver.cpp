#include "solver.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <memory>
#include <tuple>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <yaml-cpp/yaml.h>

#include "data/armor_camera_spacing.hpp"
#include "data/armor_image_spaceing.hpp"
#include "data/time_stamped.hpp"
#include "enum/armor_id.hpp"
#include "parameters/profile.hpp"
#include "parameters/rm_parameters.hpp"
#include "solved_armor.hpp"
#include "tongji/solver/reprojection_util.hpp"
#include "util/coordinate.hpp"
#include "util/index.hpp"
#include "util/math.hpp"
namespace world_exe::tongji::solver {

class Solver::Impl {
public:
    explicit Impl()
        : R_camera2gimbal_(Eigen::Matrix3d::Zero())
        , t_camera2gimbal_(Eigen::Vector3d::Zero())
        , reprojection_util_(std::make_unique<ReprojectionUtil>()) { }

    std::shared_ptr<world_exe::interfaces::IArmorInCamera> EstimateAllArmorPoses(
        std::shared_ptr<interfaces::IArmorInImage> armors_in_image) {
        std::vector<data::ArmorCameraSpacing> armor_plates;
        if (!armors_in_image) return nullptr;
        for (int i = 0; i < static_cast<int>(enumeration::ArmorIdFlag::Count); i++) {
            const auto& armor_id = util::enumeration::GetArmorIdFlag(i);
            const auto& armors   = armors_in_image->GetArmors(armor_id);

            for (const auto& armor : armors) {
                auto solved_armor = EstimatePose(armor);
                armor_plates.emplace_back(solved_armor);
            }
        }
        return std::make_shared<SolvedArmor>(armor_plates, armors_in_image->GetTimeStamp());
    }

    auto SetCamera2Gimbal(
        const Eigen::Matrix3d& R_camera2gimbal, const Eigen::Vector3d& t_camera2gimbal) -> void {
        R_camera2gimbal_ = R_camera2gimbal;
        t_camera2gimbal_ = t_camera2gimbal;
    }

    auto Camera2Gimbal(const Eigen::Vector3d& xyz_in_camera) const -> const auto {
        Eigen::Vector3d xyz_in_gimbal =
            R_camera2gimbal_.transpose() * xyz_in_camera + t_camera2gimbal_;
        return xyz_in_gimbal;
    }

    auto CalculateOptimizeYaw(const data::ArmorImageSpacing& armor_in_image,
        const Eigen::Vector3d& armor_xyz_in_gimbal, const double& gimbal_yaw,
        const double& initial_armor_yaw_in_gimbal) const -> const double {
        constexpr double SEARCH_RANGE = 140; // degree
        const auto yaw0 = util::math::clamp_pm_pi(gimbal_yaw - SEARCH_RANGE / 2 * CV_PI / 180.0);

        auto min_error = 1e10;
        auto best_yaw  = initial_armor_yaw_in_gimbal;

        auto pitch =
            (armor_in_image.id == enumeration::ArmorIdFlag::Outpost) ? -15.0 * CV_PI / 180.0 : 15.0;

        for (int i = 0; i < SEARCH_RANGE; i++) {
            double yaw = util::math::clamp_pm_pi(yaw0 + i * CV_PI / 180.0);

            auto error = reprojection_util_->CalculateReprojectionError(R_camera2gimbal_,
                t_camera2gimbal_, armor_in_image, armor_xyz_in_gimbal, yaw, pitch,
                (i - SEARCH_RANGE / 2) * CV_PI / 180.0);

            if (error < min_error) {
                min_error = error;
                best_yaw  = yaw;
            }
        }
        return best_yaw;
    }

    const data::TimeStamp GetTimeStamp() const {
        return data::TimeStamp(std::chrono::steady_clock::now().time_since_epoch());
    }

private:
    data::ArmorCameraSpacing EstimatePose(
        const world_exe::data::ArmorImageSpacing& armor_in_image) const {
        const auto& [xyz_in_camera, R_armor2camera] = EstimatePnp(armor_in_image);

        data::ArmorCameraSpacing pose;
        pose.id          = armor_in_image.id;
        pose.orientation = Eigen::Quaterniond(R_armor2camera).normalized();
        pose.position    = xyz_in_camera;
        return pose;
    }

    auto EstimatePnp(const world_exe::data::ArmorImageSpacing& armor_in_image) const
        -> const std::tuple<Eigen::Vector3d, Eigen::Matrix3d> {
        const auto& object_points = armor_in_image.isLargeArmor
            ? parameters::Robomaster::LargeArmorObjectPointsOpencv
            : parameters::Robomaster::NormalArmorObjectPointsOpencv;

        cv::Vec3d rvec, tvec;
        cv::solvePnP(object_points, armor_in_image.image_points,
            parameters::HikCameraProfile::get_intrinsic_parameters(),
            parameters::HikCameraProfile::get_distortion_parameters(), rvec, tvec, false,
            cv::SOLVEPNP_IPPE);

        // 1. P_C_cv -> P_C_ros (位置)
        Eigen::Vector3d xyz_in_camera_cv;
        cv::cv2eigen(tvec, xyz_in_camera_cv);
        const auto xyz_in_camera = util::coordinate::opencv2ros_position(xyz_in_camera_cv);

        // 2. R_A->C_cv -> R_A->C_ros (姿态)
        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);
        Eigen::Matrix3d R_armor2camera_cv;
        cv::cv2eigen(rmat, R_armor2camera_cv);
        const auto R_armor2camera = util::coordinate::opencv2ros_rotation(R_armor2camera_cv);

        return { xyz_in_camera, R_armor2camera };
    }

private:
    Eigen::Matrix3d R_camera2gimbal_;
    Eigen::Vector3d t_camera2gimbal_;

    std::unique_ptr<ReprojectionUtil> reprojection_util_;
};

Solver::Solver()
    : pimpl_(std::make_unique<Impl>()) { }
Solver::~Solver() = default;

std::shared_ptr<world_exe::interfaces::IArmorInCamera> Solver::SolvePnp(
    std::shared_ptr<interfaces::IArmorInImage> armors_in_image) {
    return pimpl_->EstimateAllArmorPoses(armors_in_image);
}
void Solver::SetCamera2Gimbal(
    const Eigen::Matrix3d& R_camera2gimbal, const Eigen::Vector3d& t_camera2gimbal) {
    pimpl_->SetCamera2Gimbal(R_camera2gimbal, t_camera2gimbal);
}

auto Solver::CalculateOptimizeYaw(const data::ArmorImageSpacing& armor_in_image,
    const Eigen::Vector3d& armor_xyz_in_gimbal, const double& gimbal_yaw,
    const double& initial_armor_yaw_in_gimbal) const -> const double {
    return pimpl_->CalculateOptimizeYaw(
        armor_in_image, armor_xyz_in_gimbal, gimbal_yaw, initial_armor_yaw_in_gimbal);
}

auto Solver::Camera2Gimbal(const Eigen::Vector3d& xyz_in_camera) const -> const auto {
    return pimpl_->Camera2Gimbal(xyz_in_camera);
}
}
