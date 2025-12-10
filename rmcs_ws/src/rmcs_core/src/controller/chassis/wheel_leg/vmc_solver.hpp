#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>

namespace rmcs_core::controller::chassis {
class VmcSolver {
public:
    VmcSolver(double l1, double l2, double l5)
        : l1_(l1)
        , l2_(l2)
        , l3_(l2)
        , l4_(l1)
        , l5_(l5) {}

    Eigen::Vector2d update(double phi1, double phi4) {
        if (std::isnan(phi1) || std::isnan(phi4)) {
            reset();
            return {0.0, 0.0};
        }

        calculate_five_link_solution(phi1, phi4);

        return get_leg_posture();
    }

    Eigen::Vector2d get_joint_torque(double F, double Tp) {
        return joint_torque_matrix_ * Eigen::Vector2d{F, Tp};
    }

    Eigen::Vector2d get_virtual_torque(double T1, double T2) {
        return joint_torque_matrix_.inverse() * Eigen::Vector2d{T1, T2};
    }

private:
    void reset() {
        tilt_angle_ = nan_;
        leg_length_ = nan_;
    }

    void calculate_five_link_solution(double phi1, double phi4) {
        // The coordinate system and variable definitions are referenced from the Zhihu article:
        // https://zhuanlan.zhihu.com/p/613007726
        auto xb = l1_ * std::cos(phi1), yb = l1_ * std::sin(phi1);
        auto xd = l5_ + l4_ * std::cos(phi4), yd = l4_ * std::sin(phi4);

        auto lbd = std::sqrt((xd - xb) * (xd - xb) + (yd - yb) * (yd - yb));

        auto a = 2 * l2_ * (xd - xb), b = 2 * l2_ * (yd - yb),
             c = l2_ * l2_ + lbd * lbd - l3_ * l3_;

        auto phi2 = 2 * std::atan2(b + std::sqrt(a * a + b * b - c * c), (a + c)),
             phi3 = std::atan2((yb - yd) + l2_ * std::sin(phi2), (xb - xd) + l2_ * std::cos(phi2));

        auto xc = l1_ * std::cos(phi1) + l2_ * std::cos(phi2),
             yc = l1_ * std::sin(phi1) + l2_ * std::sin(phi2);

        auto phi0 = std::atan2(yc, xc - l5_ / 2.0);

        tilt_angle_ = pi_ / 2.0 - phi0;
        leg_length_ = std::sqrt((xc - l5_ / 2.0) * (xc - l5_ / 2.0) + yc * yc);

        leg_posture_ = {leg_length_, tilt_angle_};

        auto j11 = l1_ * std::sin(phi0 - phi3) * std::sin(phi1 - phi2) / std::sin(phi3 - phi2),
             j12 = l4_ * std::sin(phi0 - phi2) * std::sin(phi3 - phi4) / std::sin(phi3 - phi2),
             j21 = l1_ * std::cos(phi0 - phi3) * std::sin(phi1 - phi2)
                 / (leg_length_ * std::sin(phi3 - phi2)),
             j22 = l4_ * std::cos(phi0 - phi2) * std::sin(phi3 - phi4)
                 / (leg_length_ * std::sin(phi3 - phi2));

        auto jacobian_matrix = Eigen::Matrix2d{
            {j11, j12},
            {j21, j22}
        };
        auto rotation_matrix = Eigen::Rotation2Dd(phi0 - pi_ / 2.0);
        Eigen::Matrix2d transform_matrix = Eigen::Vector2d{-1 / leg_length_, 1}.asDiagonal();

        joint_torque_matrix_ =
            jacobian_matrix.transpose() * rotation_matrix * transform_matrix.transpose();
    }

    Eigen::Vector2d get_leg_posture() const { return Eigen::Vector2d{leg_length_, tilt_angle_}; }

    static constexpr double inf_ = std::numeric_limits<double>::infinity();
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    static constexpr double pi_ = std::numbers::pi;

    const double l1_, l2_, l3_, l4_, l5_;

    double leg_length_, tilt_angle_;

    Eigen::Vector2d leg_posture_;
    Eigen::Matrix2d joint_torque_matrix_;
};
} // namespace rmcs_core::controller::chassis