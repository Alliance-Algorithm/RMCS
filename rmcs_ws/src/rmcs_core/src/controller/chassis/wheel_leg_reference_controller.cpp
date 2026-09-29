#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <ranges>
#include <stdexcept>
#include <string>

#include <rclcpp/logging.hpp>
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
        for (std::size_t i = 0; i < kJointNames.size(); ++i) {
            const std::string prefix = std::string{"/wheel_leg/"} + kJointNames[i];
            register_input(prefix + "/status_code", status_[i]);
            register_output(prefix + "/control_torque", torque_[i], 0.0);
            register_output(
                prefix + "/control_angle", angle_target_[i],
                std::numeric_limits<double>::quiet_NaN());
        }
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/wheel_leg/dr16_fresh", dr16_fresh_);
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
    }

    void update() override {
        const int previous_state = *state_;
        for (auto& output : torque_)
            *output = 0.0;
        for (auto& output : angle_target_)
            *output = std::numeric_limits<double>::quiet_NaN();
        *enable_request_ = false;
        *state_ = 0;

        const auto left = *switch_left_, right = *switch_right_;
        if (!*dr16_fresh_ || left != rmcs_msgs::Switch::MIDDLE
            || right != rmcs_msgs::Switch::MIDDLE) {
            if (previous_state != 0)
                RCLCPP_INFO(get_logger(), "DM reference hold disabled by remote switch or timeout");
            return;
        }

        for (std::size_t i = 0; i < reference_.size(); ++i)
            *angle_target_[i] = reference_[i];
        *enable_request_ = true;
        *state_ = 1;
        if (previous_state == 0)
            RCLCPP_INFO(get_logger(), "Reference hold requested at the four motor zero positions");
        if (!std::ranges::all_of(status_, [](const auto& status) { return *status == 1; }))
            return;
        *state_ = 2;
        if (previous_state != 2)
            RCLCPP_INFO(get_logger(), "All four DM drives report enabled for reference hold");
    }

private:
    static constexpr std::array kJointNames{
        "left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint"};
    std::array<InputInterface<int>, 4> status_;
    std::array<OutputInterface<double>, 4> torque_;
    std::array<OutputInterface<double>, 4> angle_target_;
    InputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    InputInterface<bool> dr16_fresh_;
    OutputInterface<bool> enable_request_;
    OutputInterface<int> state_;
    std::array<double, 4> reference_{};
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegReferenceController, rmcs_executor::Component)
