#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/chassis/wheel_leg_control_state.hpp"

namespace rmcs_core::controller::chassis {

class WheelLegRlConsumer
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    WheelLegRlConsumer()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        const auto rl_base = get_parameter_or<std::string>("rl_base", "/wheel_leg/rl");
        for (std::size_t i = 0; i < action_.size(); ++i)
            register_input(rl_base + "/action/" + kActionNames[i], action_[i], false);
        register_input(rl_base + "/valid", valid_, false);
        register_input(rl_base + "/healthy", healthy_, false);
        register_input("/chassis/control_state", state_);
        register_input("/chassis/reset_count", reset_count_);

        policy_leg_default_position_ = parameter_array_or(
            "policy_leg_default_position", std::array<double, 4>{0.0, 0.0, 0.0, 0.0});
        urdf_zero_leg_position_ = parameter_array_or(
            "urdf_zero_leg_position",
            std::array<double, 4>{1.6, 2.93, -1.6, -2.93});
        safe_leg_position_ = parameter_array_or(
            "safe_leg_position", std::array<double, 4>{-0.5, -0.35, 0.5, 0.35});

        constexpr std::array<const char*, 4> kLegJoints{
            "left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint"};
        constexpr std::array<const char*, 2> kWheelJoints{"left_wheel", "right_wheel"};
        for (std::size_t i = 0; i < kLegJoints.size(); ++i) {
            register_input(
                std::string{"/wheel_leg/"} + kLegJoints[i] + "/angle", leg_position_[i], false);
            register_output(
                std::string{"/wheel_leg/"} + kLegJoints[i] + "/control_angle", leg_target_[i],
                safe_leg_position_[i]);
        }
        for (std::size_t i = 0; i < kWheelJoints.size(); ++i)
            register_output(
                std::string{"/wheel_leg/"} + kWheelJoints[i] + "/control_velocity",
                wheel_target_[i], 0.0);

        action_scale_ = get_parameter_or<double>("action_scale", 0.25);
        wheel_velocity_scale_ = get_parameter_or<double>("wheel_velocity_scale", 10.0);
        invalid_action_ = get_parameter_or<double>("invalid_action", 0.0);
    }

    void update() override {
        const bool actions_ready = std::all_of(
            action_.begin(), action_.end(), [](const auto& input) { return input.ready(); });
        const bool leg_positions_ready = std::all_of(
            leg_position_.begin(), leg_position_.end(),
            [](const auto& input) { return input.ready() && std::isfinite(*input); });
        const auto state = static_cast<WheelLegControlState>(*state_);
        if (*reset_count_ != last_reset_count_) {
            last_reset_count_ = *reset_count_;
            hold_target_valid_ = false;
            publish_safe_targets_();
            return;
        }
        if (state == WheelLegControlState::kInit || state == WheelLegControlState::kDisabled
            || !leg_positions_ready) {
            hold_target_valid_ = false;
            publish_safe_targets_();
            return;
        }
        if (state == WheelLegControlState::kUrdfZero) {
            hold_target_valid_ = false;
            publish_fixed_targets_(urdf_zero_leg_position_);
            return;
        }
        if (state == WheelLegControlState::kNominal) {
            hold_target_valid_ = false;
            publish_fixed_targets_(policy_leg_default_position_);
            return;
        }
        if (state == WheelLegControlState::kCalibratedZero) {
            hold_target_valid_ = false;
            publish_fixed_targets_(std::array<double, 4>{0.0, 0.0, 0.0, 0.0});
            return;
        }
        const bool policy_valid = state == WheelLegControlState::kRl && actions_ready
                               && valid_.ready() && healthy_.ready() && *valid_ > 0.5
                               && *healthy_ > 0.5;
        if (!policy_valid) {
            publish_hold_targets_();
            return;
        }
        hold_target_valid_ = false;
        for (std::size_t i = 0; i < leg_target_.size(); ++i) {
            const double value = *action_[i];
            if (!std::isfinite(value)) {
                *leg_target_[i] = policy_leg_default_position_[i];
                continue;
            }
            const double desired = policy_leg_default_position_[i]
                                 + action_scale_ * std::clamp(value, -3.0, 3.0);
            const double delta = desired - *leg_position_[i];
            *leg_target_[i] =
                *leg_position_[i] + std::atan2(std::sin(delta), std::cos(delta));
        }
        for (std::size_t i = 0; i < wheel_target_.size(); ++i) {
            const double value = *action_[i == 0 ? 4 : 5];
            *wheel_target_[i] = std::isfinite(value)
                                    ? wheel_velocity_scale_ * std::clamp(value, -3.0, 3.0)
                                    : 0.0;
        }
    }

private:
    static constexpr std::array<const char*, 6> kActionNames{
        "left_hip", "left_knee", "right_hip", "right_knee", "left_wheel", "right_wheel"};

    std::array<double, 4> parameter_array_or(
        const char* name, const std::array<double, 4>& fallback) const {
        if (!has_parameter(name))
            return fallback;
        const auto values = get_parameter(name).as_double_array();
        if (values.size() != fallback.size())
            throw std::invalid_argument(
                std::string{"WheelLegRlConsumer: parameter '"} + name
                + "' must contain exactly 4 values");
        std::array<double, 4> result{};
        for (std::size_t i = 0; i < result.size(); ++i) {
            if (!std::isfinite(values[i]))
                throw std::invalid_argument(
                    std::string{"WheelLegRlConsumer: parameter '"} + name
                    + "' contains a non-finite value");
            result[i] = values[i];
        }
        return result;
    }

    void publish_safe_targets_() {
        for (std::size_t i = 0; i < leg_target_.size(); ++i)
            *leg_target_[i] = safe_leg_position_[i];
        publish_wheel_zero_();
    }

    void publish_hold_targets_() {
        if (!hold_target_valid_) {
            for (std::size_t i = 0; i < hold_leg_position_.size(); ++i)
                hold_leg_position_[i] = *leg_position_[i];
            hold_target_valid_ = true;
        }
        publish_fixed_targets_(hold_leg_position_);
    }

    void publish_fixed_targets_(const std::array<double, 4>& desired) {
        for (std::size_t i = 0; i < leg_target_.size(); ++i)
            *leg_target_[i] = desired[i];
        publish_wheel_zero_();
    }

    void publish_wheel_zero_() {
        for (auto& target : wheel_target_)
            *target = 0.0;
    }

    std::array<InputInterface<double>, 6> action_;
    std::array<InputInterface<double>, 4> leg_position_;
    InputInterface<double> valid_;
    InputInterface<double> healthy_;
    InputInterface<int> state_;
    InputInterface<std::size_t> reset_count_;
    std::array<OutputInterface<double>, 4> leg_target_;
    std::array<OutputInterface<double>, 2> wheel_target_;
    std::array<double, 4> policy_leg_default_position_{0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> urdf_zero_leg_position_{1.6, 2.93, -1.6, -2.93};
    std::array<double, 4> safe_leg_position_{-0.5, -0.35, 0.5, 0.35};
    std::array<double, 4> hold_leg_position_{};
    bool hold_target_valid_ = false;

    double action_scale_ = 0.25;
    double wheel_velocity_scale_ = 10.0;
    double invalid_action_ = 0.0;
    std::size_t last_reset_count_ = 0;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegRlConsumer, rmcs_executor::Component)
