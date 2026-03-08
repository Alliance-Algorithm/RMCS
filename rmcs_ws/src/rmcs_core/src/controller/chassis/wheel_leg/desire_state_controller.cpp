#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/wheel_leg_state.hpp>
#include <rmcs_utility/eigen_structured_bindings.hpp>

namespace rmcs_core::controller::chassis {

class DesireStateController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DesireStateController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/chassis/yaw/velocity", chassis_yaw_velocity_imu_);
        register_input("/chassis/control_velocity", chassis_control_velocity_);
        // register_input("/chassis/control_power_limit", power_limit_);

        register_input("/chassis/leg/extended", is_leg_extended_);

        register_output("/chassis/desire_state", desire_state_);
        register_output("/chassis/desire_leg_length", desire_leg_length_);
        register_output("/chassis/desire_roll_angle", desire_roll_angle_);

        reset();
    }

    void reset() {
        *desire_state_ = reset_state();
        *desire_leg_length_ = nan_;
        *desire_roll_angle_ = nan_;
    }

    void update() override {
        auto control_velocity = calculate_chassis_control_velocity();
        auto desire_state = update_desire_state(control_velocity);

        *desire_state_ = desire_state;
    }

private:
    Eigen::Vector3d calculate_chassis_control_velocity() {
        Eigen::Vector3d chassis_control_velocity;
        chassis_control_velocity = chassis_control_velocity_->vector;
        return chassis_control_velocity;
    }

    rmcs_msgs::WheelLegState update_desire_state(const Eigen::Vector3d& control_velocity) {
        rmcs_msgs::WheelLegState desire_state;
        // x-axis translational velocity, z-axis vertical velocity, z-axis angular velocity
        // auto& [vx, vz, wz] = control_velocity;

        // desire_leg_length_ += vz * leg_length_sensitivity_;
        if (*is_leg_extended_) {
            *desire_leg_length_ = 0.25;
        } else {
            *desire_leg_length_ = 0.16;
        }

        // Distance :always 0 during velocity control
        desire_state.distance = 0.0; // else measure.distance
        desire_state.velocity = 0.0;

        // Yaw angle:
        desire_state.yaw_angle = 0.0; // else measure.yaw
        desire_state.yaw_velocity = 0.0;

        // Leg tilt angle:
        desire_state.left_tilt_angle = -0.1;
        desire_state.right_tilt_angle = -0.1;

        // Pitch angle:
        desire_state.body_pitch_angle = 0.0;
        desire_state.body_pitch_velocity = 0.0;

        return desire_state;
    }

    void update_jump_state() {}

    void update_climb_state() {}

    constexpr static rmcs_msgs::WheelLegState reset_state() {
        return rmcs_msgs::WheelLegState{nan_, nan_, nan_, nan_, nan_, nan_, nan_, nan_, nan_, nan_};
    }

    constexpr static double nan_ = std::numeric_limits<double>::quiet_NaN();

    constexpr static double leg_length_sensitivity_ = 0.00001;

    constexpr static double dt_ = 0.001;

    // Leg length limits
    static constexpr double max_leg_length_ = 0.36;
    static constexpr double min_leg_length_ = 0.13;

    // Maximum control velocities
    static constexpr double translational_velocity_max_ = 2.0;
    static constexpr double angular_velocity_max_ = 2.0;

    InputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;

    InputInterface<double> chassis_yaw_velocity_imu_;
    InputInterface<double> power_limit_;

    InputInterface<bool> is_leg_extended_;

    OutputInterface<double> desire_leg_length_;
    OutputInterface<double> desire_roll_angle_;

    OutputInterface<rmcs_msgs::WheelLegState> desire_state_;

    // rmcs_msgs::WheelLegState chassis_mode_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::DesireStateController, rmcs_executor::Component)