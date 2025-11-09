#pragma once
#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>

#include "interfaces/armor_in_image.hpp"
#include "interfaces/pnp_solver.hpp"

namespace world_exe::v1::pnpsolver {

class ArmorIPPEPnPSolver final : public interfaces::IPnpSolver {
public:
    ArmorIPPEPnPSolver(const std::vector<cv::Point3d>& LargeArmorObjectPointsOpencv,
        const std::vector<cv::Point3d>& NormalArmorObjectPointsOpencv);
    ~ArmorIPPEPnPSolver();
    void set_time_point(const std::chrono::nanoseconds& time_point);

    std::shared_ptr<world_exe::interfaces::IArmorInCamera> SolvePnp(
        std::shared_ptr<interfaces::IArmorInImage> armor) override;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    std::chrono::nanoseconds time_point_;
};
}