#pragma once

#include <cmath>

#include <iostream>
#include <limits>
#include <utility>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/eigen_structured_bindings.hpp>

namespace rmcs_core::controller::gimbal {
using namespace rmcs_description;

class TwoAxisGimbalSolver {
    class Operation {
        friend class TwoAxisGimbalSolver;

        virtual PitchLink::DirectionVector update(TwoAxisGimbalSolver& super) const = 0;
        // Modifies super.control_enabled_ in the method.
        // Returns the new control direction (in PitchLink) to be used for control.
    };

public:
    TwoAxisGimbalSolver(rmcs_executor::Component& component, double upper_limit, double lower_limit)
        : upper_limit_(std::cos(upper_limit), -std::sin(upper_limit))
        , lower_limit_(std::cos(lower_limit), -std::sin(lower_limit)) {

        component.register_input("/gimbal/pitch/angle", gimbal_pitch_angle_);
        component.register_input("/tf", tf_);
    }

    class SetDisabled : public Operation {
        PitchLink::DirectionVector update(TwoAxisGimbalSolver& super) const override {
            super.control_enabled_ = false;
            return {};
        }
    };

    class SetToLevel : public Operation {
        PitchLink::DirectionVector update(TwoAxisGimbalSolver& super) const override {
            auto odom_dir = fast_tf::cast<OdomImu>(
                PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, *super.tf_);
            if (odom_dir->x() == 0 || odom_dir->y() == 0)
                return {};

            super.control_enabled_ = true;
            odom_dir->z() = 0;
            auto dir = fast_tf::cast<PitchLink>(odom_dir, *super.tf_);
            dir->normalize();
            return dir;
        }
    };

    class SetControlDirection : public Operation {
    public:
        explicit SetControlDirection(OdomImu::DirectionVector target)
            : target_(std::move(target)) {}

    private:
        PitchLink::DirectionVector update(TwoAxisGimbalSolver& super) const override {
            super.control_enabled_ = true;
            return fast_tf::cast<PitchLink>(target_, *super.tf_);
        }

        OdomImu::DirectionVector target_;
    };

    class SetControlShift : public Operation {
    public:
        SetControlShift(double yaw_shift, double pitch_shift)
            : yaw_shift_(yaw_shift)
            , pitch_shift_(pitch_shift) {}

    private:
        PitchLink::DirectionVector update(TwoAxisGimbalSolver& super) const override {
            PitchLink::DirectionVector dir;

            if (!super.control_enabled_) {
                super.control_enabled_ = true;
                dir = PitchLink::DirectionVector{Eigen::Vector3d::UnitX()};
            } else {
                dir = fast_tf::cast<PitchLink>(super.control_direction_, *super.tf_);
            }

            auto yaw_transform = Eigen::AngleAxisd{yaw_shift_, Eigen::Vector3d::UnitZ()};
            auto pitch_transform = Eigen::AngleAxisd{pitch_shift_, Eigen::Vector3d::UnitY()};

            return PitchLink::DirectionVector{pitch_transform * (yaw_transform * (*dir))};
        }

        double yaw_shift_, pitch_shift_;
    };

    struct AngleError {
        double yaw_angle_error, pitch_angle_error;
    };

    AngleError update(const Operation& operation) {
        update_yaw_axis();

        PitchLink::DirectionVector control_direction = operation.update(*this);
        if (!control_enabled_)
            return {nan_, nan_};

        auto [control_direction_yaw_link, pitch] = pitch_link_to_yaw_link(control_direction);

        clamp_control_direction(control_direction_yaw_link);
        if (!control_enabled_)
            return {nan_, nan_};

        control_direction_ =
            fast_tf::cast<OdomImu>(yaw_link_to_pitch_link(control_direction_yaw_link, pitch), *tf_);
        return calculate_control_errors(control_direction_yaw_link, pitch);
    }

    bool enabled() const { return control_enabled_; }

private:
    void update_yaw_axis() {
        auto yaw_axis =
            fast_tf::cast<PitchLink>(YawLink::DirectionVector{Eigen::Vector3d::UnitZ()}, *tf_);
        *yaw_axis_filtered_ += 0.1 * (*fast_tf::cast<OdomImu>(yaw_axis, *tf_));
        yaw_axis_filtered_->normalize();
    }

    auto pitch_link_to_yaw_link(const PitchLink::DirectionVector& dir) const
        -> std::pair<YawLink::DirectionVector, Eigen::Vector2d> {

        std::pair<YawLink::DirectionVector, Eigen::Vector2d> result;
        auto& [dir_yaw_link, pitch] = result;

        auto yaw_axis = fast_tf::cast<PitchLink>(yaw_axis_filtered_, *tf_);
        pitch = {yaw_axis->z(), yaw_axis->x()};
        pitch.normalized();

        const auto& [x, y, z] = *dir;
        dir_yaw_link = {x * pitch.x() - z * pitch.y(), y, x * pitch.y() + z * pitch.x()};

        return result;
    }

    static PitchLink::DirectionVector
        yaw_link_to_pitch_link(const YawLink::DirectionVector& dir, const Eigen::Vector2d& pitch) {

        const auto& [x, y, z] = *dir;
        return {x * pitch.x() + z * pitch.y(), y, -x * pitch.y() + z * pitch.x()};
    }

    void clamp_control_direction(YawLink::DirectionVector& control_direction) {
        const auto& [x, y, z] = *control_direction;

        Eigen::Vector2d projection{x, y};
        double norm = projection.norm();
        if (norm > 0)
            projection /= norm;
        else {
            control_enabled_ = false;
            return;
        }

        if (z > upper_limit_.y())
            *control_direction << upper_limit_.x() * projection, upper_limit_.y();
        else if (z < lower_limit_.y())
            *control_direction << lower_limit_.x() * projection, lower_limit_.y();
    }

    static AngleError calculate_control_errors(
        const YawLink::DirectionVector& control_direction, const Eigen::Vector2d& pitch) {
        const auto& [x, y, z] = *control_direction;
        const auto& [c, s] = pitch;

        AngleError result;
        result.yaw_angle_error = std::atan2(y, x);
        double x_projected = std::sqrt(x * x + y * y);
        result.pitch_angle_error = -std::atan2(z * c - x_projected * s, z * s + x_projected * c);

        return result;
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    const Eigen::Vector2d upper_limit_, lower_limit_;

    rmcs_executor::Component::InputInterface<double> gimbal_pitch_angle_;
    rmcs_executor::Component::InputInterface<Tf> tf_;

    OdomImu::DirectionVector yaw_axis_filtered_{Eigen::Vector3d::UnitZ()};

    bool control_enabled_ = false;
    OdomImu::DirectionVector control_direction_;
};

} // namespace rmcs_core::controller::gimbal