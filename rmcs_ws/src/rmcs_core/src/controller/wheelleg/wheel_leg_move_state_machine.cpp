#include <algorithm>
#include <cmath>
#include <string>

#include <eigen3/Eigen/Core>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

namespace rmcs_core::controller::wheelleg {

// MOVE state machine (top layer), layered after STAND.
//
//   mode = "stand": double MIDDLE arms the STAND state machine only; the robot
//                   holds the nominal stance (move command stays 0).
//   mode = "move" : double MIDDLE arms STAND first; once STAND reports
//                   /wheelleg/stand/ready, this machine forwards the DR16
//                   joystick to the traditional command layer
//                   (/wheelleg/move/velocity, /wheelleg/move/yaw_rate).
//
// Both DOWN fully disables (STAND clears `armed`, so `active` drops and the move
// command is zeroed here as well).
class WheelLegMoveStateMachine
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegMoveStateMachine()
        : Node{get_component_name(), node::options()} {
        mode_ = get_parameter_or<std::string>("mode", std::string{"stand"});
        if (mode_ != "stand" && mode_ != "move")
            throw std::invalid_argument("move state machine mode must be 'stand' or 'move'");
        vx_max_ = get_parameter_or("vx_max", 0.5);
        vy_max_ = get_parameter_or("vy_max", 0.0);
        yaw_rate_max_ = get_parameter_or("yaw_rate_max", 1.0);
        deadzone_ = get_parameter_or("deadzone", 0.08);
        yaw_invert_ = get_parameter_or("yaw_invert", false);

        register_input("/remote/joystick/right", joystick_right_, false);
        register_input("/remote/joystick/left", joystick_left_, false);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/wheel_leg/dr16_fresh", dr16_fresh_, false);
        register_input("/wheelleg/stand/armed", stand_armed_, false);
        register_input("/wheelleg/stand/ready", stand_ready_, false);

        register_output("/wheelleg/move/velocity", move_velocity_, 0.0);
        register_output("/wheelleg/move/yaw_rate", move_yaw_rate_, 0.0);
    }

    void update() override {
        using Switch = rmcs_msgs::Switch;
        const bool fresh = dr16_fresh_.ready() && *dr16_fresh_;
        const auto left = fresh ? *switch_left_ : Switch::UNKNOWN;
        const auto right = fresh ? *switch_right_ : Switch::UNKNOWN;

        const bool armed = stand_armed_.ready() && *stand_armed_;
        const bool ready = stand_ready_.ready() && *stand_ready_;
        const bool active =
            mode_ == "move" && armed && ready && fresh && left == Switch::MIDDLE
            && right == Switch::MIDDLE;

        if (!active) {
            *move_velocity_ = 0.0;
            *move_yaw_rate_ = 0.0;
            return;
        }

        const double forward = channel_(joystick_right_.ready() ? joystick_right_->x() : 0.0);
        const double yaw = channel_(joystick_left_.ready() ? joystick_left_->y() : 0.0);
        *move_velocity_ = forward * vx_max_;
        *move_yaw_rate_ = (yaw_invert_ ? -1.0 : 1.0) * yaw * yaw_rate_max_;
    }

private:
    [[nodiscard]] double channel_(double value) const {
        if (!std::isfinite(value) || std::abs(value) < deadzone_)
            return 0.0;
        return std::clamp(value, -1.0, 1.0);
    }

    std::string mode_ = "stand";
    double vx_max_ = 0.5, vy_max_ = 0.0, yaw_rate_max_ = 1.0;
    double deadzone_ = 0.08;
    bool yaw_invert_ = false;

    InputInterface<Eigen::Vector2d> joystick_right_, joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    InputInterface<bool> dr16_fresh_, stand_armed_, stand_ready_;
    OutputInterface<double> move_velocity_, move_yaw_rate_;
};

} // namespace rmcs_core::controller::wheelleg

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::wheelleg::WheelLegMoveStateMachine, rmcs_executor::Component)
