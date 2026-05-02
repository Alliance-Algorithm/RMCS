#include <algorithm>
#include <cmath>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::gimbal {

class HoldController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    HoldController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , joystick_deadband_(get_parameter("joystick_deadband").as_double())
        , mouse_deadband_(get_parameter("mouse_deadband").as_double()) {
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);

        register_output("/gimbal/hold/desired", hold_desired_, false);
    }

    void update() override {
        using namespace rmcs_msgs; // NOLINT(google-build-using-namespace)
        const auto switch_left = *switch_left_;
        const auto switch_right = *switch_right_;

        const bool gimbal_disabled = switch_left == Switch::UNKNOWN
                                  || switch_right == Switch::UNKNOWN
                                  || (switch_left == Switch::DOWN && switch_right == Switch::DOWN);

        if (gimbal_disabled || !is_idle()) {
            idle_counter_ = 0;
            *hold_desired_ = false;
            return;
        }

        idle_counter_ = std::min(idle_counter_ + 1, kEntryTicks);
        *hold_desired_ = idle_counter_ >= kEntryTicks;
    }

private:
    [[nodiscard]] bool is_idle() const {
        const auto joystick_norm = joystick_left_->norm();
        const auto mouse_norm = mouse_velocity_->norm();
        return std::isfinite(joystick_norm) && std::isfinite(mouse_norm)
            && joystick_norm < joystick_deadband_ && mouse_norm < mouse_deadband_;
    }

    static constexpr int kEntryTicks = 50;

    const double joystick_deadband_;
    const double mouse_deadband_;
    int idle_counter_ = 0;

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;

    OutputInterface<bool> hold_desired_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::HoldController, rmcs_executor::Component)
