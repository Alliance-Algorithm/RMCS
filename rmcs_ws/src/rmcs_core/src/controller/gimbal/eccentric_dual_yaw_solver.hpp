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

    explicit EccentricDualYawSolver(rmcs_executor::Component& component) {
        component.register_input("/tf", tf_);
        component.register_input("/gimbal/top_yaw/angle", top_yaw_angle_);
    }

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
            const Eigen::Vector3d& control_direction, Eigen::Vector3d robot_center,
            double upper_pitch, double lower_pitch)
            : dir_(control_direction.normalized())
            , center_(std::move(robot_center))
            , upper_(upper_pitch)
            , lower_(lower_pitch) {}

    private:
        auto update(EccentricDualYawSolver& s) const -> Error override {
            using namespace rmcs_description;
            const auto& tf = *s.tf_;

            const auto dir_gcl =
                fast_tf::cast<GimbalCenterLink>(OdomGimbalImu::DirectionVector{dir_}, tf);
            const auto ctr_gcl =
                fast_tf::cast<GimbalCenterLink>(OdomGimbalImu::Position{center_}, tf);
            const auto cam_gcl = fast_tf::lookup_transform<GimbalCenterLink, CameraLink>(tf);
            const auto brl_gcl = fast_tf::cast<GimbalCenterLink>(
                PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, tf);
            const auto btm_gcl = fast_tf::cast<GimbalCenterLink>(
                BottomYawLink::DirectionVector{Eigen::Vector3d::UnitX()}, tf);

            const Eigen::Vector3d dir = normalize(*dir_gcl);
            const Eigen::Vector3d brl = normalize(*brl_gcl);
            const Eigen::Vector3d btm = normalize(*btm_gcl);
            const Eigen::Vector3d diff = *ctr_gcl - cam_gcl.translation();

            const double center_azimuth =
                (diff.head<2>().norm() > 1e-9) ? std::atan2(diff.y(), diff.x()) : 0.0;
            const double barrel_azimuth = std::atan2(dir.y(), dir.x());
            const double barrel_pitch = std::atan2(-dir.z(), std::hypot(dir.x(), dir.y()));
            const double current_btm = std::atan2(btm.y(), btm.x());
            const double current_brl = std::atan2(-brl.z(), std::hypot(brl.x(), brl.y()));
            const double current_top = limit_rad(*s.top_yaw_angle_);

            const double bottom_error = limit_rad(center_azimuth - current_btm);
            const double desired_top = limit_rad(barrel_azimuth - center_azimuth);
            const double top_error = limit_rad(desired_top - current_top);
            const double desired_pitch = std::clamp(barrel_pitch, upper_, lower_);
            const double pitch_error = limit_rad(desired_pitch - current_brl);

            s.enabled_ = true;
            return {bottom_error, top_error, pitch_error, true};
        }

        Eigen::Vector3d dir_, center_;
        double upper_, lower_;
    };

    class Navigation : public Operation {
    public:
        Navigation(
            Eigen::Vector2d toward, double stored_bottom, double stored_pitch, double upper,
            double lower)
            : toward_(std::move(toward))
            , sb_(stored_bottom)
            , sp_(stored_pitch)
            , upper_(upper)
            , lower_(lower) {}

    private:
        auto update(EccentricDualYawSolver& s) const -> Error override {
            double bx = std::isfinite(toward_.x()) ? toward_.x() : sb_;
            double by = std::isfinite(toward_.y()) ? toward_.y() : sp_;
            by = std::clamp(by, upper_, lower_);
            s.enabled_ = true;
            return {limit_rad(bx - sb_), 0.0, limit_rad(by - sp_), true};
        }
        Eigen::Vector2d toward_;
        double sb_, sp_, upper_, lower_;
    };

private:
    static constexpr double kNaN_ = std::numeric_limits<double>::quiet_NaN();

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
        if (n > 1e-9)
            return v / n;
        return Eigen::Vector3d::UnitX();
    }

    rmcs_executor::Component::InputInterface<rmcs_description::Tf> tf_;
    rmcs_executor::Component::InputInterface<double> top_yaw_angle_;

    bool enabled_ = false;
};

} // namespace rmcs_core::controller::gimbal
