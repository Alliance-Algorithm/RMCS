#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::chassis {

class SteeringWheelStatus
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit SteeringWheelStatus()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , vehicle_radius_(get_parameter("vehicle_radius").as_double())
        , wheel_radius_(get_parameter("wheel_radius").as_double()) {
        register_input("/chassis/left_front_steering/angle", left_front_steering_angle_);
        register_input("/chassis/left_back_steering/angle", left_back_steering_angle_);
        register_input("/chassis/right_back_steering/angle", right_back_steering_angle_);
        register_input("/chassis/right_front_steering/angle", right_front_steering_angle_);

        register_input("/chassis/left_front_wheel/velocity", left_front_wheel_velocity_);
        register_input("/chassis/left_back_wheel/velocity", left_back_wheel_velocity_);
        register_input("/chassis/right_back_wheel/velocity", right_back_wheel_velocity_);
        register_input("/chassis/right_front_wheel/velocity", right_front_wheel_velocity_);

        register_output("/chassis/velocity", chassis_velocity_, 0.0, 0.0, 0.0);
    }

    void update() override {
        auto steering_status = calculate_steering_status();
        auto wheel_velocities = calculate_wheel_velocities();
        chassis_velocity_->vector = calculate_chassis_velocity(steering_status, wheel_velocities);
    }

private:
    struct SteeringStatus {
        Eigen::Vector4d angle, cos_angle, sin_angle;
    };

    SteeringStatus calculate_steering_status() const {
        SteeringStatus steering_status;

        steering_status.angle = {
            *left_front_steering_angle_, //
            *left_back_steering_angle_,  //
            *right_back_steering_angle_, //
            *right_front_steering_angle_ //
        };
        steering_status.angle.array() -= std::numbers::pi / 4;
        steering_status.cos_angle = steering_status.angle.array().cos();
        steering_status.sin_angle = steering_status.angle.array().sin();

        return steering_status;
    }

    Eigen::Vector4d calculate_wheel_velocities() const {
        return {
            *left_front_wheel_velocity_, //
            *left_back_wheel_velocity_,  //
            *right_back_wheel_velocity_, //
            *right_front_wheel_velocity_ //
        };
    }

    Eigen::Vector3d calculate_chassis_velocity(
        const SteeringStatus& steering_status, const Eigen::Vector4d& wheel_velocities) const {
        Eigen::Vector3d velocity;
        double one_quarter_r = wheel_radius_ / 4.0;
        velocity.x() = one_quarter_r * wheel_velocities.dot(steering_status.cos_angle);
        velocity.y() = one_quarter_r * wheel_velocities.dot(steering_status.sin_angle);
        velocity.z() = -one_quarter_r / vehicle_radius_
                     * (-wheel_velocities[0] * steering_status.sin_angle[0]
                        + wheel_velocities[1] * steering_status.cos_angle[1]
                        + wheel_velocities[2] * steering_status.sin_angle[2]
                        - wheel_velocities[3] * steering_status.cos_angle[3]);
        return velocity;
    }

    const double vehicle_radius_;
    const double wheel_radius_;

    InputInterface<double> left_front_steering_angle_;
    InputInterface<double> left_back_steering_angle_;
    InputInterface<double> right_back_steering_angle_;
    InputInterface<double> right_front_steering_angle_;

    InputInterface<double> left_front_wheel_velocity_;
    InputInterface<double> left_back_wheel_velocity_;
    InputInterface<double> right_back_wheel_velocity_;
    InputInterface<double> right_front_wheel_velocity_;

    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_velocity_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::SteeringWheelStatus, rmcs_executor::Component)
