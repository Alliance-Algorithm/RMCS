#include <cmath>
#include <limits>
#include <numbers>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::gimbal {

class DualYawController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DualYawController()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/gimbal/top_yaw/angle", top_yaw_angle_);
        register_input("/gimbal/top_yaw/velocity", top_yaw_velocity_);
        register_input("/gimbal/bottom_yaw/angle", bottom_yaw_angle_);
        register_input("/gimbal/bottom_yaw/velocity", bottom_yaw_velocity_);

        register_output("/gimbal/bottom_yaw/control_angle_shift", bottom_yaw_control_angle_shift_);

        register_output("/gimbal/yaw/angle", yaw_angle_, 0.0);
        register_output("/gimbal/yaw/velocity", yaw_velocity_, 0.0);
    }

    void update() override {
        *bottom_yaw_control_angle_shift_ =
            (*switch_left_ == rmcs_msgs::Switch::DOWN && *switch_right_ == rmcs_msgs::Switch::DOWN)
                ? nan_
                : 0.0;

        double yaw_angle = *top_yaw_angle_ + *bottom_yaw_angle_;
        if (yaw_angle < 0)
            yaw_angle += 2 * std::numbers::pi;
        else if (yaw_angle > 2 * std::numbers::pi)
            yaw_angle -= 2 * std::numbers::pi;
        *yaw_angle_ = yaw_angle;

        *yaw_velocity_ = *top_yaw_velocity_ + *bottom_yaw_velocity_;
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<rmcs_msgs::Switch> switch_right_, switch_left_;
    InputInterface<double> top_yaw_angle_, top_yaw_velocity_;
    InputInterface<double> bottom_yaw_angle_, bottom_yaw_velocity_;

    OutputInterface<double> bottom_yaw_control_angle_shift_;

    OutputInterface<double> yaw_angle_, yaw_velocity_;
};
} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::DualYawController, rmcs_executor::Component)
