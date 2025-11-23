#pragma once

namespace rmcs_msgs {

struct WheelLegState {
    double distance;
    double velocity;
    double yaw_angle;
    double yaw_velocity;
    double left_tilt_angle;
    double left_tilt_velocity;
    double right_tilt_angle;
    double right_tilt_velocity;
    double body_pitch_angle;
    double body_pitch_velocity;
};

} // namespace rmcs_msgs