#include <algorithm>
#include <array>
#include <string>

#include <eigen3/Eigen/Geometry>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "wheel_leg/joint_coordinates.hpp"
#include "wheel_leg/jump_v4.hpp"

namespace rmcs_core::controller::chassis {

class WheelLegStateEstimator
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    WheelLegStateEstimator()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        for (std::size_t i = 0; i < wheel_leg::kJointNames.size(); ++i) {
            const auto prefix = std::string{"/wheel_leg/"} + wheel_leg::kJointNames[i];
            register_input(prefix + "/angle", angle_[i]);
            register_input(prefix + "/velocity", velocity_[i]);
        }
        register_input("/wheel_leg/imu/quaternion", imu_quaternion_);
        register_output("/wheel_leg/state/position", position_, wheel_leg::JointVector{});
        register_output("/wheel_leg/state/velocity", velocity_output_, wheel_leg::JointVector{});
        register_output("/wheel_leg/state/gravity", gravity_, Eigen::Vector3d::Zero());
    }

    void update() override {
        wheel_leg::JointVector angle, velocity;
        for (std::size_t i = 0; i < angle.size(); ++i) {
            angle[i] = *angle_[i];
            velocity[i] = *velocity_[i];
        }
        auto q = wheel_leg::model_position(angle);
        const auto legs =
            wheel_leg::continuous_legs({q[0], q[1], q[2], q[3]}, wheel_leg::jump_v4::kBounds);
        std::copy(legs.begin(), legs.end(), q.begin());
        *position_ = q;
        *velocity_output_ = wheel_leg::model_velocity(velocity);
        *gravity_ = imu_quaternion_->conjugate() * -Eigen::Vector3d::UnitZ();
    }

private:
    std::array<InputInterface<double>, 6> angle_, velocity_;
    InputInterface<Eigen::Quaterniond> imu_quaternion_;

    OutputInterface<wheel_leg::JointVector> position_, velocity_output_;
    OutputInterface<Eigen::Vector3d> gravity_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegStateEstimator, rmcs_executor::Component)
