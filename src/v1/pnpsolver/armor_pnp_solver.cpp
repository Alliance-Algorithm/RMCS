#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

#include "armor_pnp_solver.hpp"
#include "interfaces/armor_in_camera.hpp"
#include "data/time_stamped.hpp"
#include "parameters/profile.hpp"
#include "util/index.hpp"

using namespace world_exe::v1::pnpsolver;

class ArmorIPPEPnPSolver::Impl {
public:
    Impl(const std::vector<cv::Point3d>& LargeArmorObjectPoints,

        const std::vector<cv::Point3d>& NormalArmorObjectPoints)
        : LargeArmorObjectPoints_(LargeArmorObjectPoints)
        , NormalArmorObjectPoints_(NormalArmorObjectPoints) { }
    std::optional<const world_exe::data::ArmorCameraSpacing> Solve(
        const world_exe::data::ArmorImageSpacing& armors) {

        cv::Mat rvec, tvec;
        auto& objectPoints =
            armors.isLargeArmor ? LargeArmorObjectPoints_ : NormalArmorObjectPoints_;
#pragma warning "Better set matrix by initializer"
        if (armors.image_points.size() == 4
            && cv::solvePnP(objectPoints, armors.image_points,
                world_exe::parameters::HikCameraProfile::get_intrinsic_parameters(),
                world_exe::parameters::HikCameraProfile::get_distortion_parameters(), rvec, tvec,
                false, cv::SOLVEPNP_IPPE)) {

            Eigen::Vector3d position = { tvec.at<double>(2), -tvec.at<double>(0),
                -tvec.at<double>(1) };
            position                 = position;
            if (position.norm() > MaxArmorDistance) {
                return {};
            }

            Eigen::Vector3d rvec_eigen  = { rvec.at<double>(2), -rvec.at<double>(0),
                 -rvec.at<double>(1) };
            Eigen::Quaterniond rotation = Eigen::Quaterniond { Eigen::AngleAxisd {
                rvec_eigen.norm(), rvec_eigen.normalized() } };

            return { { armors.id, position, rotation } };
        }
        return {};
    }

private:
    inline constexpr static const double MaxArmorDistance = 15.0;

    const std::vector<cv::Point3d>& LargeArmorObjectPoints_;

    const std::vector<cv::Point3d>& NormalArmorObjectPoints_;
};
class ArmorInCamera final : public world_exe::interfaces::IArmorInCamera {
public:
    const world_exe::data::TimeStamp& GetTimeStamp() const override { return time_stampe; }
    const std::vector<world_exe::data::ArmorCameraSpacing>& GetArmors(
        const world_exe::enumeration::ArmorIdFlag& armor_id) const override {
        return armors[world_exe::util::enumeration::GetIndex(armor_id)];
    }

    world_exe::data::TimeStamp time_stampe{};
    std::vector<world_exe::data::ArmorCameraSpacing>
        armors[static_cast<int>(world_exe::enumeration::ArmorIdFlag::Count)];

    ~ArmorInCamera() = default;
};

std::shared_ptr<world_exe::interfaces::IArmorInCamera> ArmorIPPEPnPSolver::SolvePnp(
    std::shared_ptr<world_exe::interfaces::IArmorInImage> armors) {
    std::shared_ptr<ArmorInCamera> armors_ = std::make_shared<ArmorInCamera>();
    for (int i = 0; i < static_cast<int>(world_exe::enumeration::ArmorIdFlag::Count); i++) {
        armors_->armors[i].clear();
        for (const auto& armor : armors->GetArmors(static_cast<enumeration::ArmorIdFlag>(1 << i))) {
            const auto& armor_in_camera = pimpl_->Solve(armor);
            if (armor_in_camera.has_value())
                armors_->armors[i].emplace_back(std::move(armor_in_camera.value()));
        }
    }
    return armors_;
}

ArmorIPPEPnPSolver::ArmorIPPEPnPSolver(const std::vector<cv::Point3d>& LargeArmorObjectPoints,
    const std::vector<cv::Point3d>& NormalArmorObjectPoints)
    : pimpl_(std::make_unique<Impl>(LargeArmorObjectPoints, NormalArmorObjectPoints)) { }

ArmorIPPEPnPSolver::~ArmorIPPEPnPSolver() = default;