#include "controller/gimbal/yaw_friction_feedforward.hpp"

#include <chrono>
#include <limits>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

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
        total_limit_ = get_parameter_or<double>("yaw_total_torque_limit", 4.5);
        if (!std::isfinite(total_limit_) || total_limit_ <= 0.)
            throw std::invalid_argument("Invalid Yaw total torque limit");
        register_input("/gimbal/yaw/control_velocity", velocity_);
        register_input("/gimbal/yaw/pid_control_torque", pid_);
        register_output("/gimbal/yaw/torque_ff", feedforward_, 0.);
        register_output(
            "/gimbal/yaw/control_torque", torque_, std::numeric_limits<double>::quiet_NaN());
    }

    void update() override {
        const auto now = std::chrono::steady_clock::now();
        const double dt = first_ ? .001 : std::chrono::duration<double>(now - last_).count();
        first_ = false;
        last_ = now;
        if (!std::isfinite(*pid_) || !std::isfinite(*velocity_)) {
            compensation_.reset();
            *feedforward_ = 0.;
            *torque_ = std::numeric_limits<double>::quiet_NaN();
            return;
        }
        *feedforward_ = compensation_.update(*velocity_, dt, config_);
        *torque_ = std::clamp(*pid_ + *feedforward_, -total_limit_, total_limit_);
    }

private:
    YawFrictionCompensation::Config config_;
    YawFrictionCompensation compensation_;
    InputInterface<double> velocity_, pid_;
    OutputInterface<double> feedforward_, torque_;
    std::chrono::steady_clock::time_point last_{};
    double total_limit_ = 4.5;
    bool first_ = true;
};

} // namespace rmcs_core::controller::gimbal

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::YawFrictionFeedforward, rmcs_executor::Component)
