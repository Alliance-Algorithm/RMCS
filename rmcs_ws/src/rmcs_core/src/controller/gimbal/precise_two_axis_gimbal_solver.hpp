#pragma once

#include <cmath>

#include <limits>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::gimbal {
using namespace rmcs_description;

class PreciseTwoAxisGimbalSolver {
    struct Operation {
        // update(PreciseTwoAxisGimbalSolver& super) -> double;
        // Modifies super.control_pitch_angle_ in the method.
        // Returns the yaw shift to be applied.
    };

public:
    PreciseTwoAxisGimbalSolver(
        rmcs_executor::Component& component, double upper_limit, double lower_limit)
        : upper_limit_(upper_limit)
        , lower_limit_(lower_limit) {
        component.register_input("/gimbal/pitch/angle", gimbal_pitch_angle_);
        component.register_input("/tf", tf_);
    }

    struct SetControlDirection : Operation {
        friend class PreciseTwoAxisGimbalSolver;

        explicit SetControlDirection(OdomImu::DirectionVector target)
            : target_(std::move(target)) {}

    private:
        double update(PreciseTwoAxisGimbalSolver& super) const {
            if (!super.tf_.ready()) {
                super.control_pitch_angle_ = PreciseTwoAxisGimbalSolver::nan_;
                return PreciseTwoAxisGimbalSolver::nan_;
            }

            auto dir_yaw_link = fast_tf::cast<YawLink>(target_, *super.tf_);
            Eigen::Vector3d dir = *dir_yaw_link;

            const double norm = dir.norm();
            if (norm < kEps) {
                super.control_pitch_angle_ = PreciseTwoAxisGimbalSolver::nan_;
                return PreciseTwoAxisGimbalSolver::nan_;
            }
            dir /= norm;

            const double xy_norm = std::hypot(dir.x(), dir.y());

            // 在 YawLink 中，yaw 是相对当前总 yaw 的增量
            const double yaw_shift = std::atan2(dir.y(), dir.x());

            // 机械 pitch 约定：水平为 0，低头为正，所以是负号
            const double pitch_angle = -std::atan2(dir.z(), std::max(xy_norm, kEps));

            super.set_pitch_angle(pitch_angle);
            return yaw_shift;
        }

        static constexpr double kEps = 1e-9;
        OdomImu::DirectionVector target_;
    };

    struct SetDisabled : Operation {
        friend class PreciseTwoAxisGimbalSolver;

    private:
        static double update(PreciseTwoAxisGimbalSolver& super) {
            super.control_pitch_angle_ = nan_;
            return nan_;
        }
    };

    struct SetControlPitch : Operation {
        friend class PreciseTwoAxisGimbalSolver;
        explicit SetControlPitch(double control_pitch_angle)
            : control_pitch_angle_(control_pitch_angle) {}

    private:
        double update(PreciseTwoAxisGimbalSolver& super) const {
            super.set_pitch_angle(control_pitch_angle_);
            return 0.0;
        }

        double control_pitch_angle_;
    };

    struct SetControlShift : Operation {
        friend class PreciseTwoAxisGimbalSolver;
        SetControlShift(double yaw_shift, double pitch_shift)
            : yaw_shift_(yaw_shift)
            , pitch_shift_(pitch_shift) {}

    private:
        double update(PreciseTwoAxisGimbalSolver& super) const {
            if (std::isnan(super.control_pitch_angle_))
                super.control_pitch_angle_ = *super.gimbal_pitch_angle_;
            super.set_pitch_angle(super.control_pitch_angle_ + pitch_shift_);
            return yaw_shift_;
        }

        double yaw_shift_, pitch_shift_;
    };

    struct ControlAngle {
        double yaw_shift, pitch_angle;
    };

    template <typename T>
    requires std::is_base_of_v<Operation, T> ControlAngle update(const T& operation) {
        ControlAngle result;
        result.yaw_shift = operation.update(*this);
        result.pitch_angle = control_pitch_angle_;
        return result;
    }

    bool enabled() const { return !std::isnan(control_pitch_angle_); }

private:
    void set_pitch_angle(double angle) {
        // `upper_limit_` is numerically less than `lower_limit_`.
        control_pitch_angle_ = std::clamp(angle, upper_limit_, lower_limit_);
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    rmcs_executor::Component::InputInterface<double> gimbal_pitch_angle_;
    rmcs_executor::Component::InputInterface<rmcs_description::Tf> tf_;

    const double upper_limit_, lower_limit_;

    double control_pitch_angle_ = nan_;
};

} // namespace rmcs_core::controller::gimbal