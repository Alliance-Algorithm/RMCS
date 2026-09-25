#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

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
        safe_leg_position_ = parameter_array_or(
            "safe_leg_position", std::array<double, 4>{-0.5, -0.35, 0.5, 0.35});

        constexpr std::array<const char*, 4> kLegJoints{
            "left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint"};
        constexpr std::array<const char*, 2> kWheelJoints{"left_wheel", "right_wheel"};
        for (std::size_t i = 0; i < kLegJoints.size(); ++i)
            register_output(
                std::string{"/wheel_leg/"} + kLegJoints[i] + "/control_angle", leg_target_[i],
                safe_leg_position_[i]);
        for (std::size_t i = 0; i < kWheelJoints.size(); ++i)
            register_output(
                std::string{"/wheel_leg/"} + kWheelJoints[i] + "/control_velocity",
                wheel_target_[i], 0.0);

        action_scale_ = get_parameter_or<double>("action_scale", 0.5);
        wheel_velocity_scale_ = get_parameter_or<double>("wheel_velocity_scale", 10.0);
        invalid_action_ = get_parameter_or<double>("invalid_action", 0.0);
    }

    void update() override {
        const bool actions_ready = std::all_of(
            action_.begin(), action_.end(), [](const auto& input) { return input.ready(); });
        const bool policy_valid = actions_ready && valid_.ready() && healthy_.ready()
                               && *valid_ > 0.5 && *healthy_ > 0.5 && *state_ == 3;
        if (*reset_count_ != last_reset_count_) {
            last_reset_count_ = *reset_count_;
            publish_safe_targets_();
            return;
        }
        if (!policy_valid) {
            publish_safe_targets_();
            return;
        }
        for (std::size_t i = 0; i < leg_target_.size(); ++i) {
            const double value = *action_[i];
            *leg_target_[i] = std::isfinite(value)
                                  ? policy_leg_default_position_[i]
                                        + action_scale_ * std::clamp(value, -3.0, 3.0)
                                  : policy_leg_default_position_[i];
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
        for (auto& target : wheel_target_)
            *target = 0.0;
    }

    std::array<InputInterface<double>, 6> action_;
    InputInterface<double> valid_;
    InputInterface<double> healthy_;
    InputInterface<int> state_;
    InputInterface<std::size_t> reset_count_;
    std::array<OutputInterface<double>, 4> leg_target_;
    std::array<OutputInterface<double>, 2> wheel_target_;
    std::array<double, 4> policy_leg_default_position_{0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> safe_leg_position_{-0.5, -0.35, 0.5, 0.35};

    double action_scale_ = 0.5;
    double wheel_velocity_scale_ = 10.0;
    double invalid_action_ = 0.0;
    std::size_t last_reset_count_ = 0;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegRlConsumer, rmcs_executor::Component)
