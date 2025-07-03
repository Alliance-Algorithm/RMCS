#include <cmath>

#include <limits>
#include <utility>

#include <eigen3/Eigen/Dense>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::gimbal {
using namespace rmcs_description;

class TwoAxisGimbalSolver {
    class Operation {
        friend class TwoAxisGimbalSolver;
        virtual PitchLink::DirectionVector update(TwoAxisGimbalSolver& super) const = 0;
    };

public:
    TwoAxisGimbalSolver(rmcs_executor::Component& component, double upper_limit, double lower_limit)
        : upper_limit_(upper_limit + std::numbers::pi / 2)
        , lower_limit_(lower_limit + std::numbers::pi / 2) {
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

        PitchLink::DirectionVector dir = operation.update(*this);
        if (!control_enabled_)
            return {nan_, nan_};

        clamp_control_direction(dir);
        if (!control_enabled_)
            return {nan_, nan_};

        control_direction_ = fast_tf::cast<OdomImu>(dir, *tf_);
        return calculate_control_errors(dir);
    }

private:
    void update_yaw_axis() {
        auto yaw_axis =
            fast_tf::cast<PitchLink>(YawLink::DirectionVector{Eigen::Vector3d::UnitZ()}, *tf_);
        *yaw_axis_filtered_ += 0.1 * (*fast_tf::cast<OdomImu>(yaw_axis, *tf_));
        yaw_axis_filtered_->normalize();
    }

    void clamp_control_direction(PitchLink::DirectionVector& dir) {
        dir->normalized();
        auto yaw_axis = fast_tf::cast<PitchLink>(yaw_axis_filtered_, *tf_);

        auto cos_angle = yaw_axis->dot(*dir);
        if (cos_angle == 1 || cos_angle == -1) {
            control_enabled_ = false;
            return;
        }

        auto angle = std::acos(cos_angle);
        if (angle < upper_limit_)
            *dir =
                Eigen::AngleAxisd{upper_limit_, (yaw_axis->cross(*dir)).normalized()} * (*yaw_axis);
        else if (angle > lower_limit_)
            *dir =
                Eigen::AngleAxisd{lower_limit_, (yaw_axis->cross(*dir)).normalized()} * (*yaw_axis);
    }

    AngleError calculate_control_errors(PitchLink::DirectionVector& dir) {
        auto yaw_axis = fast_tf::cast<PitchLink>(yaw_axis_filtered_, *tf_);
        double pitch = -std::atan2(yaw_axis->x(), yaw_axis->z());

        double &x = dir->x(), &y = dir->y(), &z = dir->z();
        double sp = std::sin(pitch), cp = std::cos(pitch);
        double a = x * cp + z * sp;
        double b = std::sqrt(y * y + a * a);

        AngleError result;
        result.yaw_angle_error = std::atan2(y, a);
        result.pitch_angle_error =
            -std::atan2(z * cp * cp - x * cp * sp + sp * b, -z * cp * sp + x * sp * sp + cp * b);

        return result;
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    double upper_limit_, lower_limit_;

    rmcs_executor::Component::InputInterface<double> gimbal_pitch_angle_;
    rmcs_executor::Component::InputInterface<Tf> tf_;

    OdomImu::DirectionVector yaw_axis_filtered_{Eigen::Vector3d::UnitZ()};

    bool control_enabled_ = false;
    OdomImu::DirectionVector control_direction_;
};

} // namespace rmcs_core::controller::gimbal