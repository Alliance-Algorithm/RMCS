#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

namespace rmcs_core::controller::chassis {

class WheelLegReferenceController final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegReferenceController()
        : Node{get_component_name(), node::options()} {
        constexpr std::array names{
            "left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint"};
        for (std::size_t i = 0; i < names.size(); ++i) {
            const std::string prefix = std::string{"/wheel_leg/"} + names[i];
            register_input(prefix + "/angle", angle_[i]);
            register_input(prefix + "/velocity", velocity_[i]);
            register_input(prefix + "/max_torque", max_torque_[i]);
            register_input(prefix + "/fault_code", fault_[i]);
            register_input(prefix + "/status_code", status_[i]);
            register_output(prefix + "/control_torque", torque_[i], 0.0);
        }
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/wheel_leg/dr16_fresh", dr16_fresh_);
        register_input("/wheel_leg/feedback_fresh", feedback_fresh_);
        register_output("/wheel_leg/enable_request", enable_request_, false);
        register_output("/wheel_leg/reference/state", state_, 0);

        const auto values = get_parameter("reference_motor_angles").as_double_array();
        if (values.size() != reference_.size())
            throw std::runtime_error("reference_motor_angles requires four motor angles");
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (!std::isfinite(values[i]))
                throw std::runtime_error("reference_motor_angles must be finite");
            reference_[i] = values[i];
        }
        kp_ = get_parameter_or("hold_kp", 4.0);
        kd_ = get_parameter_or("hold_kd", 0.25);
        torque_limit_ = get_parameter_or("hold_torque_limit", 0.5);
        capture_window_ = get_parameter_or("capture_window_rad", 0.05);
        velocity_limit_ = get_parameter_or("hold_velocity_limit", 0.4);
        max_feedback_step_ = get_parameter_or("max_feedback_step_rad", 0.04);
        if (!std::isfinite(kp_) || !std::isfinite(kd_) || !std::isfinite(torque_limit_)
            || !std::isfinite(capture_window_) || !std::isfinite(velocity_limit_)
            || !std::isfinite(max_feedback_step_) || kp_ <= 0 || kd_ < 0 || torque_limit_ <= 0
            || capture_window_ <= 0 || velocity_limit_ <= 0 || max_feedback_step_ <= 0)
            throw std::runtime_error("Invalid reference hold gains or limits");
    }

    void update() override {
        for (auto& output : torque_)
            *output = 0.0;
        *enable_request_ = false;
        *state_ = 0;

        const auto left = *switch_left_, right = *switch_right_;
        if (left == rmcs_msgs::Switch::DOWN && right == rmcs_msgs::Switch::DOWN) {
            down_seen_ = *dr16_fresh_;
            last_angles_.reset();
            return;
        }
        if (left != rmcs_msgs::Switch::MIDDLE || right != rmcs_msgs::Switch::MIDDLE)
            down_seen_ = false;
        if (!down_seen_ || left != rmcs_msgs::Switch::MIDDLE || right != rmcs_msgs::Switch::MIDDLE
            || !*dr16_fresh_ || !*feedback_fresh_) {
            if (!*dr16_fresh_ || !*feedback_fresh_)
                down_seen_ = false;
            last_angles_.reset();
            return;
        }

        std::array<double, 4> current_angles;
        for (std::size_t i = 0; i < reference_.size(); ++i) {
            const double angle = *angle_[i];
            // Motor angles are multi-turn; wrapping by 2π could select another linkage branch.
            if (!std::isfinite(angle) || !std::isfinite(*velocity_[i])
                || !std::isfinite(*max_torque_[i]) || *max_torque_[i] <= 0 || *fault_[i] != 0
                || std::abs(angle - reference_[i]) > capture_window_
                || (last_angles_ && std::abs(angle - (*last_angles_)[i]) > max_feedback_step_)
                || std::abs(*velocity_[i]) > velocity_limit_) {
                down_seen_ = false;
                last_angles_.reset();
                return;
            }
            current_angles[i] = angle;
        }
        last_angles_ = current_angles;

        *enable_request_ = true;
        *state_ = 1;
        if (!std::ranges::all_of(status_, [](const auto& status) { return *status == 1; }))
            return;
        for (std::size_t i = 0; i < reference_.size(); ++i)
            *torque_[i] = std::clamp(
                kp_ * (reference_[i] - *angle_[i]) - kd_ * *velocity_[i],
                -std::min(torque_limit_, *max_torque_[i]),
                std::min(torque_limit_, *max_torque_[i]));
        *state_ = 2;
    }

private:
    std::array<InputInterface<double>, 4> angle_, velocity_, max_torque_;
    std::array<InputInterface<int>, 4> fault_, status_;
    std::array<OutputInterface<double>, 4> torque_;
    InputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    InputInterface<bool> dr16_fresh_, feedback_fresh_;
    OutputInterface<bool> enable_request_;
    OutputInterface<int> state_;
    std::array<double, 4> reference_{};
    double kp_ = 4.0, kd_ = 0.25, torque_limit_ = 0.5;
    double capture_window_ = 0.05, velocity_limit_ = 0.4, max_feedback_step_ = 0.04;
    bool down_seen_ = false;
    std::optional<std::array<double, 4>> last_angles_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegReferenceController, rmcs_executor::Component)
