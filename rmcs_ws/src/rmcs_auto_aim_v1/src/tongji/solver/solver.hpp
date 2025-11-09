#pragma once

#include "interfaces/pnp_solver.hpp"

namespace world_exe::tongji::solver {

class Solver final : public interfaces::IPnpSolver {
public:
    explicit Solver();
    ~Solver();

    std::shared_ptr<world_exe::interfaces::IArmorInCamera> SolvePnp(
        std::shared_ptr<interfaces::IArmorInImage> armors) override;

    void SetCamera2Gimbal(
        const Eigen::Matrix3d& R_camera2gimbal, const Eigen::Vector3d& t_camera2gimbal);

    auto Camera2Gimbal(const Eigen::Vector3d& xyz_in_camera) const -> const auto;
    auto CalculateOptimizeYaw(const data::ArmorImageSpacing& armor_in_image,
        const Eigen::Vector3d& armor_xyz_in_gimbal, const double& gimbal_yaw,
        const double& initial_armor_yaw_in_gimbal) const -> const double;

    Solver(const Solver&)                = delete;
    Solver& operator=(const Solver&)     = delete;
    Solver(Solver&&) noexcept            = default;
    Solver& operator=(Solver&&) noexcept = default;

private:
    class Impl;

    std::unique_ptr<Impl> pimpl_;
};

}
