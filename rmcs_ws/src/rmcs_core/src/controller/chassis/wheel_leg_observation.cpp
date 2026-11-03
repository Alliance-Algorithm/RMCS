#include "wheel_leg/policy/selection.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <optional>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>
#include <string>
#include <variant>
#include <vector>
namespace rmcs_core::controller::chassis {
class WheelLegObservation final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegObservation()
        : Node{get_component_name(), node::options()}
        , policy_(policy_parameters(get_parameter("policy").as_string())) {
        if (policy_.name == kWheelbipe.name)
            adapter_.emplace<Wheelbipe>();
        register_input("/wheel_leg/state/position", q_);
        register_input("/wheel_leg/state/velocity", dq_);
        register_input("/wheel_leg/state/gravity", gravity_);
        register_input("/wheel_leg/state/valid", valid_);
        register_input("/wheel_leg/imu/angular_velocity", gyro_);
        register_input("/wheel_leg/rl/input/enable", enable_);
        register_input("/wheel_leg/rl/input/command", command_);
        register_input("/wheel_leg/rl/input/height", height_);
        register_input("/wheel_leg/rl/applied_action", applied_);
        register_input("/wheel_leg/rl/action_applied", action_applied_);
        register_input("/wheel_leg/rl/state", state_);
        register_input("/chassis/reset_count", reset_);
        register_input("/predefined/update_count", tick_);
        register_input("/predefined/update_rate", rate_);
        register_output("/wheel_leg/rl/input/frame", frame_, std::vector<double>(35, 0.));
        register_output("/wheel_leg/rl/input/frame_sequence", sequence_, std::size_t{0});
    }
    void update() override {
        if (*reset_ != last_reset_ || (!*enable_ && was_enabled_)
            || (*state_ == 3 && last_state_ != 3)) {
            std::visit([](auto& a) { a.reset(); }, adapter_);
            last_tick_.reset();
            std::fill(frame_->begin(), frame_->end(), 0.);
            *sequence_ = 0;
            last_reset_ = *reset_;
        }
        was_enabled_ = *enable_;
        last_state_ = *state_;
        if (!*enable_ || !*valid_)
            return;
        if (!schedule_checked_) {
            if (*rate_ != 1000.)
                throw std::invalid_argument("Wheel-leg observation requires a 1000 Hz executor");
            schedule_checked_ = true;
        }
        const auto divisor = static_cast<std::size_t>(1000. / policy_.policy_hz);
        if (last_tick_ && *tick_ - *last_tick_ < divisor)
            return;
        last_tick_ = *tick_;
        const auto history =
            *action_applied_
                ? *applied_
                : (policy_.name == kWheelbipe.name ? JointVector{} : measured_policy_history(*q_));
        const auto frame = std::visit(
            [&](auto& a) {
                return a.sample(
                    ++policy_tick_, *q_, *dq_, {gyro_->x(), gyro_->y(), gyro_->z()},
                    {gravity_->x(), gravity_->y(), gravity_->z()},
                    {command_->x(), command_->y(), command_->z()}, *height_, history, *state_ == 3);
            },
            adapter_);
        if (!frame) {
            *sequence_ = 0;
            return;
        }
        std::copy(frame->begin(), frame->end(), frame_->begin());
        *sequence_ = policy_tick_;
    }

private:
    const PolicyParameters& policy_;
    std::variant<ProneMotion, Wheelbipe> adapter_;
    InputInterface<JointVector> q_, dq_, applied_;
    InputInterface<Eigen::Vector3d> gravity_, gyro_, command_;
    InputInterface<double> height_, rate_;
    InputInterface<bool> enable_, valid_, action_applied_;
    InputInterface<int> state_;
    InputInterface<std::size_t> reset_, tick_;
    OutputInterface<std::vector<double>> frame_;
    OutputInterface<std::size_t> sequence_;
    std::optional<std::size_t> last_tick_;
    std::size_t policy_tick_ = 0, last_reset_ = 0;
    int last_state_ = 0;
    bool was_enabled_ = false, schedule_checked_ = false;
};
} // namespace rmcs_core::controller::chassis
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegObservation, rmcs_executor::Component)
