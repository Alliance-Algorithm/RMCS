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
    }

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

    const double upper_limit_, lower_limit_;

    double control_pitch_angle_ = nan_;
};

} // namespace rmcs_core::controller::gimbal