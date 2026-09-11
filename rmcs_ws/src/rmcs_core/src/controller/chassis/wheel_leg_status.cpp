#include <cmath>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::chassis {

class WheelLegStatus
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit WheelLegStatus()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , vehicle_radius_(get_parameter("vehicle_radius").as_double())
        , wheel_radius_(get_parameter("wheel_radius").as_double()) {
        register_input("/wheel_leg/left_wheel/velocity", left_wheel_velocity_);
        register_input("/wheel_leg/right_wheel/velocity", right_wheel_velocity_);

        register_output("/chassis/velocity", chassis_velocity_, 0.0, 0.0, 0.0);
    }

    void update() override {
        const auto wheel_velocities = calculate_wheel_velocities();
        chassis_velocity_->vector = calculate_chassis_velocity(wheel_velocities);
    }

private:
    Eigen::Vector2d calculate_wheel_velocities() const {
        return {
            *left_wheel_velocity_, //
            *right_wheel_velocity_ //
        };
    }

    Eigen::Vector3d calculate_chassis_velocity(const Eigen::Vector2d& wheel_velocities) const {
        const double forward_velocity =
            wheel_radius_ * (wheel_velocities.x() + wheel_velocities.y()) / 2.0;

        double yaw_rate = 0.0;
        const double half_track = 2.0 * vehicle_radius_;
        if (std::abs(half_track) > epsilon_)
            yaw_rate = wheel_radius_ * (wheel_velocities.y() - wheel_velocities.x()) / half_track;

        return {forward_velocity, 0.0, yaw_rate};
    }

    static constexpr double epsilon_ = 1e-6;

    const double vehicle_radius_;
    const double wheel_radius_;

    InputInterface<double> left_wheel_velocity_;
    InputInterface<double> right_wheel_velocity_;

    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_velocity_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::WheelLegStatus, rmcs_executor::Component)
