#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/gimbal/yaw_friction_feedforward.hpp"

namespace rmcs_core::controller::gimbal {

class YawFrictionFeedforward final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    YawFrictionFeedforward()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        config_.coulomb = get_parameter_or<double>("yaw_coulomb_ff", 0.);
        config_.static_friction = get_parameter_or<double>("yaw_static_friction_ff", 0.);
        config_.viscous = get_parameter_or<double>("yaw_viscous_ff", 0.);
        config_.stribeck_velocity = get_parameter_or<double>("yaw_stribeck_velocity", .003);
        config_.smoothing_velocity = get_parameter_or<double>("yaw_ff_velocity_smoothing", .05);
        config_.scale = get_parameter_or<double>("yaw_friction_ff_scale", 0.);
        config_.torque_limit = get_parameter_or<double>("yaw_ff_torque_limit", .8);
        config_.slew_rate = get_parameter_or<double>("yaw_ff_slew_rate", 100.);
        config_.validate();
        total_torque_limit_ = get_parameter_or<double>("yaw_total_torque_limit", 4.5);
        if (!std::isfinite(total_torque_limit_) || total_torque_limit_ <= 0.)
            throw std::invalid_argument("Invalid yaw total torque limit");

        register_input("/gimbal/yaw/control_velocity", velocity_reference_);
        register_input("/gimbal/yaw/pid_control_torque", pid_torque_);
        register_output("/gimbal/yaw/torque_ff", feedforward_torque_, 0.);
        register_output(
            "/gimbal/yaw/control_torque", control_torque_,
            std::numeric_limits<double>::quiet_NaN());
    }

    void update() override {
        if (!std::isfinite(*pid_torque_) || !std::isfinite(*velocity_reference_)) {
            compensation_.reset();
            first_update_ = true;
            last_update_time_ = {};
            *feedforward_torque_ = 0.;
            *control_torque_ = std::numeric_limits<double>::quiet_NaN();
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        const double dt =
            first_update_ ? .001 : std::chrono::duration<double>(now - last_update_time_).count();
        first_update_ = false;
        last_update_time_ = now;
        *feedforward_torque_ = compensation_.update(*velocity_reference_, dt, config_);
        *control_torque_ = std::clamp(
            *pid_torque_ + *feedforward_torque_, -total_torque_limit_, total_torque_limit_);
    }

private:
    InputInterface<double> velocity_reference_, pid_torque_;
    OutputInterface<double> feedforward_torque_, control_torque_;

    YawFrictionCompensation::Config config_;
    YawFrictionCompensation compensation_;
    std::chrono::steady_clock::time_point last_update_time_{};
    double total_torque_limit_ = 4.5;
    bool first_update_ = true;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::YawFrictionFeedforward, rmcs_executor::Component)
