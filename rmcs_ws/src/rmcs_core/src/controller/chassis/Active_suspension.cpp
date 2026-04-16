#include <algorithm>
#include <cstdint>
#include <cmath>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::chassis {

class ActiveSuspension
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ActiveSuspension()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
            {
        register_input("/chassis/left_front_joint/target_angle", left_front_joint_target_angle_);
        register_input("/chassis/left_back_joint/target_angle", left_back_joint_target_angle_);
        register_input("/chassis/right_front_joint/target_angle", right_front_joint_target_angle_);
        register_input("/chassis/right_back_joint/target_angle", right_back_joint_target_angle_);

        register_input("/chassis/left_front_joint/angle", left_front_joint_angle_);
        register_input("/chassis/left_back_joint/angle", left_back_joint_angle_);
        register_input("/chassis/right_front_joint/angle", right_front_joint_angle_);
        register_input("/chassis/right_back_joint/angle", right_back_joint_angle_);

        switch_torque_limit_ = get_parameter_or("switch_torque_limit", 200.0);
        steady_torque_limit_ = get_parameter_or("steady_torque_limit", 50.0);

        register_output(
            "/chassis/left_front_joint/torque_limit", left_front_joint_torque_limit_,
            steady_torque_limit_);
        register_output(
            "/chassis/left_back_joint/torque_limit", left_back_joint_torque_limit_,
            steady_torque_limit_);
        register_output(
            "/chassis/right_front_joint/torque_limit", right_front_joint_torque_limit_,
            steady_torque_limit_);
        register_output(
            "/chassis/right_back_joint/torque_limit", right_back_joint_torque_limit_,
            steady_torque_limit_);

    }

    void update() override {
        update_joint_torque_limit_(
            *left_front_joint_target_angle_, left_front_joint_switch_state_,
            *left_front_joint_angle_, *left_front_joint_torque_limit_);
        update_joint_torque_limit_(
            *left_back_joint_target_angle_, left_back_joint_switch_state_,
            *left_back_joint_angle_, *left_back_joint_torque_limit_);
        update_joint_torque_limit_(
            *right_front_joint_target_angle_, right_front_joint_switch_state_,
            *right_front_joint_angle_, *right_front_joint_torque_limit_);
        update_joint_torque_limit_(
            *right_back_joint_target_angle_, right_back_joint_switch_state_,
            *right_back_joint_angle_, *right_back_joint_torque_limit_);
    }

private:
    struct JointSwitchState {
        double last_target_angle = 0.0;
        std::uint64_t ticks_since_switch = 0;
        bool switch_active = false;
    };

    double switch_torque_limit_ = 200.0;
    double steady_torque_limit_ = 50.0;
    static constexpr double kTargetAngleEpsilon = 1e-4;
    static constexpr double kAngleErrorTorqueGain = 1.0;
    static constexpr double kUpdatePeriodSeconds = 1.0 / 1000.0;
    static constexpr std::uint64_t kHighTorqueHoldTicks = 1000;
    static constexpr double kTorqueDecayRate = 8.0;

    double steady_torque_limit_from_angle_(double target_angle, double current_angle) const {
        return std::max(
            0.0, steady_torque_limit_ + kAngleErrorTorqueGain * (target_angle - current_angle));
    }

    void update_joint_torque_limit_(
        double current_target_angle,
        JointSwitchState& switch_state,
        double current_angle,
        double& torque_limit) {
        const double steady_torque_limit =
            steady_torque_limit_from_angle_(current_target_angle, current_angle);

        if (std::abs(current_target_angle - switch_state.last_target_angle) > kTargetAngleEpsilon) {
            switch_state.last_target_angle = current_target_angle;
            switch_state.ticks_since_switch = 0;
            switch_state.switch_active = true;
        } else if (switch_state.switch_active) {
            ++switch_state.ticks_since_switch;
        }

        if (!switch_state.switch_active) {
            torque_limit = steady_torque_limit;
            return;
        }

        if (switch_state.ticks_since_switch <= kHighTorqueHoldTicks) {
            torque_limit = switch_torque_limit_;
            return;
        }

        const double decay_per_tick = std::exp(-kTorqueDecayRate * kUpdatePeriodSeconds);
        const double decay =
            std::pow(decay_per_tick, switch_state.ticks_since_switch - kHighTorqueHoldTicks);
        torque_limit = std::max(
            steady_torque_limit,
            steady_torque_limit
                + ((switch_torque_limit_ - steady_torque_limit) * decay));
    }

    InputInterface<double> left_front_joint_target_angle_;
    InputInterface<double> left_back_joint_target_angle_;
    InputInterface<double> right_front_joint_target_angle_;
    InputInterface<double> right_back_joint_target_angle_;

    InputInterface<double> left_front_joint_angle_;
    InputInterface<double> left_back_joint_angle_;
    InputInterface<double> right_front_joint_angle_;
    InputInterface<double> right_back_joint_angle_;

    OutputInterface<double> left_front_joint_torque_limit_;
    OutputInterface<double> left_back_joint_torque_limit_;
    OutputInterface<double> right_front_joint_torque_limit_;
    OutputInterface<double> right_back_joint_torque_limit_;

    JointSwitchState left_front_joint_switch_state_;
    JointSwitchState left_back_joint_switch_state_;
    JointSwitchState right_front_joint_switch_state_;
    JointSwitchState right_back_joint_switch_state_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::ActiveSuspension, rmcs_executor::Component)
