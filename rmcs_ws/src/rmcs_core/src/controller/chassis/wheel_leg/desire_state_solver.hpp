
#include "rmcs_msgs/wheel_leg_state.hpp"

namespace rmcs_core::controller::chassis {

class DesireStateSolver {
public:
    explicit DesireStateSolver(double desire_leg_length)
        : desire_leg_length_(desire_leg_length) {}

    rmcs_msgs::WheelLegState update(const double& vx, const double& vz, const double& wz) {
        desire_leg_length_ += vz * leg_length_sensitivity_;

        // distance :always 0 during velocity control
        desire_state_.distance = 0;
        desire_state_.velocity = vx;

        // yaw angle:
        desire_state_.yaw_angle = 0;
        desire_state_.yaw_velocity = wz;

        return desire_state_;
    }

    // void update()

    void update_current_distance() {}

    void update_current_yaw_angle() {}

    double update_desire_leg_length() const { return desire_leg_length_; }

    double update_desire_roll_angle() const { return desire_roll_angle_; }

private:
    void reset_all_controls() {
        desire_leg_length_ = 0.15;
        desire_roll_angle_ = 0.0;
    }

    constexpr static double leg_length_sensitivity_ = 0.0003;

    constexpr static double dt_ = 0.001;

    double desire_leg_length_;
    double desire_roll_angle_;

    rmcs_msgs::WheelLegState desire_state_;
};
} // namespace rmcs_core::controller::chassis