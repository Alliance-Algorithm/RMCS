#include <algorithm>
#include <cmath>

#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::gimbal {

using namespace rmcs_description;

class PitchGravityFeedforward
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    PitchGravityFeedforward()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        get_parameter_or("pitch_gravity_ff_gain", gain_, 0.);
        get_parameter_or("pitch_gravity_ff_phase", phase_, 0.);
        register_input("/tf", tf_);
        register_output("/gimbal/pitch/torque_ff", torque_, 0.);
    }

    void update() override {
        const auto direction =
            fast_tf::cast<OdomImu>(PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, *tf_);
        const double pitch = std::asin(std::clamp(direction->z(), -1., 1.));
        *torque_ = gain_ * std::sin(pitch - phase_);
    }

private:
    InputInterface<Tf> tf_;
    OutputInterface<double> torque_;
    double gain_ = 0., phase_ = 0.;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::PitchGravityFeedforward, rmcs_executor::Component)
