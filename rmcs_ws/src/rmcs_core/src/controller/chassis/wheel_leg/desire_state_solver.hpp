
#include "rmcs_msgs/wheel_leg_state.hpp"
#include <algorithm>

namespace rmcs_core::controller::chassis {

class DesireStateSolver {
public:
    explicit DesireStateSolver(double desire_leg_length)
        : initial_leg_length_(desire_leg_length) {
        reset();
    }

    void reset() {
        desire_leg_length_ = initial_leg_length_;
        desire_roll_angle_ = 0.0;
    }

    rmcs_msgs::WheelLegState update(const double& vx, const double& vz, const double& wz) {
        // desire_leg_length_ += vz * leg_length_sensitivity_;

        // distance :always 0 during velocity control
        desire_state_.distance = 0.0;
        desire_state_.velocity = 0.0;

        // yaw angle:
        desire_state_.yaw_angle = 0.0;
        desire_state_.yaw_velocity = 0.0;

        return desire_state_;
    }

    double update_desire_leg_length(const double min, const double max) {
        desire_leg_length_ = std::clamp(desire_leg_length_, min, max);

        return desire_leg_length_;
    }

    double update_desire_roll_angle() const { return desire_roll_angle_; }

private:
    constexpr static double leg_length_sensitivity_ = 0.00001;

    constexpr static double dt_ = 0.001;

    const double initial_leg_length_;

    double desire_leg_length_;
    double desire_roll_angle_;

    rmcs_msgs::WheelLegState desire_state_;
};
} // namespace rmcs_core::controller::chassis