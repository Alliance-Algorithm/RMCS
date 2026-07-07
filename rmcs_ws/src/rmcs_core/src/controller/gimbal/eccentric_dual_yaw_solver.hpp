#pragma once

#include <cmath>
#include <limits>
#include <numbers>

#include <eigen3/Eigen/Geometry>
#include <rmcs_description/sentry_description.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::gimbal {

class EccentricDualYawSolver {
public:
    struct Error {
        double bottom_yaw = kNaN_;
        double top_yaw = kNaN_;
        double pitch = kNaN_;
        bool valid = false;
    };

    class Operation {
        friend class EccentricDualYawSolver;
        virtual auto update(EccentricDualYawSolver& solver) const -> Error = 0;
    };

    auto update(const Operation& op) -> Error { return op.update(*this); }
    auto enabled() const -> bool { return enabled_; }

    class SetDisabled : public Operation {
    private:
        auto update(EccentricDualYawSolver& s) const -> Error override {
            s.enabled_ = false;
            return {kNaN_, kNaN_, kNaN_, false};
        }
    };

    class AutoAim : public Operation {
    public:
        AutoAim(
            const rmcs_description::Tf& tf, double top_yaw_angle,
            const Eigen::Vector3d& control_direction, Eigen::Vector3d robot_center,
            double upper_pitch, double lower_pitch)
            : tf_(tf)
            , top_yaw_angle_(top_yaw_angle)
            , dir_(control_direction.normalized())
            , center_(std::move(robot_center))
            , upper_(upper_pitch)
            , lower_(lower_pitch) {}

    private:
        auto update(EccentricDualYawSolver& s) const -> Error override {
            using namespace rmcs_description;

            const auto dir_gcl =
                fast_tf::cast<GimbalCenterLink>(OdomGimbalImu::DirectionVector{dir_}, tf_);
            const auto ctr_gcl =
                fast_tf::cast<GimbalCenterLink>(OdomGimbalImu::Position{center_}, tf_);
            const auto cam_gcl = fast_tf::lookup_transform<GimbalCenterLink, CameraLink>(tf_);
            const auto brl_gcl = fast_tf::cast<GimbalCenterLink>(
                PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, tf_);
            const auto btm_gcl = fast_tf::cast<GimbalCenterLink>(
                BottomYawLink::DirectionVector{Eigen::Vector3d::UnitX()}, tf_);

            const Eigen::Vector3d dir = normalize(*dir_gcl);
            const Eigen::Vector3d brl = normalize(*brl_gcl);
            const Eigen::Vector3d btm = normalize(*btm_gcl);
            const Eigen::Vector3d diff = *ctr_gcl - cam_gcl.translation();

            const double center_azimuth =
                (diff.head<2>().norm() > kEpsilon) ? std::atan2(diff.y(), diff.x()) : 0.0;
            const double barrel_azimuth = std::atan2(dir.y(), dir.x());
            const double barrel_pitch = std::atan2(-dir.z(), std::hypot(dir.x(), dir.y()));
            const double current_btm = std::atan2(btm.y(), btm.x());
            const double current_brl = std::atan2(-brl.z(), std::hypot(brl.x(), brl.y()));
            const double current_top = limit_rad(top_yaw_angle_);

            const double bottom_error = limit_rad(center_azimuth - current_btm);
            const double desired_top = limit_rad(barrel_azimuth - center_azimuth);
            const double top_error = limit_rad(desired_top - current_top);
            const double desired_pitch = std::clamp(barrel_pitch, upper_, lower_);
            const double pitch_error = limit_rad(desired_pitch - current_brl);

            s.enabled_ = true;
            return {bottom_error, top_error, pitch_error, true};
        }

        const rmcs_description::Tf& tf_;
        double top_yaw_angle_;
        Eigen::Vector3d dir_, center_;
        double upper_, lower_;
    };

    class Navigation : public Operation {
    public:
        Navigation(
            double top_yaw_angle, Eigen::Vector2d toward, double current_bottom_yaw,
            double current_pitch_yaw, double fallback_bottom, double fallback_pitch, double upper,
            double lower)
            : top_yaw_angle_(top_yaw_angle)
            , toward_(std::move(toward))
            , current_bottom_(current_bottom_yaw)
            , current_pitch_(current_pitch_yaw)
            , fallback_bottom_(fallback_bottom)
            , fallback_pitch_(fallback_pitch)
            , upper_(upper)
            , lower_(lower) {}

    private:
        auto update(EccentricDualYawSolver& s) const -> Error override {
            double bx = std::isfinite(toward_.x()) ? toward_.x() : fallback_bottom_;
            double by = std::isfinite(toward_.y()) ? toward_.y() : fallback_pitch_;
            by = std::clamp(by, upper_, lower_);

            const double current_top = limit_rad(top_yaw_angle_);

            s.enabled_ = true;
            return {
                limit_rad(bx - current_bottom_),
                limit_rad(0.0 - current_top),
                limit_rad(by - current_pitch_),
                true,
            };
        }
        double top_yaw_angle_;
        Eigen::Vector2d toward_;
        double current_bottom_, current_pitch_, fallback_bottom_, fallback_pitch_, upper_, lower_;
    };

private:
    static constexpr double kNaN_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kEpsilon = 1e-9;

    static auto limit_rad(double a) -> double {
        constexpr double p = std::numbers::pi_v<double>;
        while (a > p)
            a -= 2.0 * p;
        while (a <= -p)
            a += 2.0 * p;
        return a;
    }

    static auto normalize(const Eigen::Vector3d& v) -> Eigen::Vector3d {
        double n = v.norm();
        if (n > kEpsilon)
            return v / n;
        return Eigen::Vector3d::UnitX();
    }

    bool enabled_ = false;
};

} // namespace rmcs_core::controller::gimbal
