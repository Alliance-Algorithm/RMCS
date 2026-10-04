#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "wheel_leg_arm_sequence.hpp"

namespace rmcs_core::controller::chassis {

// Chassis command source of the wheel-leg. It decodes the remote control into the chassis command
// interfaces. RlController alone owns PREPARE -> RL, so the executor graph stays acyclic.
class WheelLegChassisController final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    explicit WheelLegChassisController()
        : Node{get_component_name(), node::options()} {
        register_interfaces_();
        load_parameters_();
        height_ = default_command_height_;
        stop_controls_(0);
    }

    void update() override {
        using rmcs_msgs::Switch;

        // RemoteControl may retain the last DR16 switch values after a receive
        // gap. Invalidate the arm sequence independently of those cached values.
        const auto switch_right = *remote_fresh_ ? *switch_right_ : Switch::UNKNOWN;
        const auto switch_left = *remote_fresh_ ? *switch_left_ : Switch::UNKNOWN;
        const auto keyboard = *keyboard_;

        const bool both_down = switch_left == Switch::DOWN && switch_right == Switch::DOWN;
        const bool any_unknown = switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN;

        // Power-on hold: stay at kInit(0) until the operator first moves a switch.
        if (!switch_activity_seen_
            && (switch_left != last_switch_left_ || switch_right != last_switch_right_))
            switch_activity_seen_ = true;

        *jump_request_ = false;
        *jump_apex_delta_ = 0.0;

        if (!switch_activity_seen_) {
            stop_controls_(0);
        } else if (any_unknown || both_down) {
            reset_all_controls_(switch_left, switch_right);
        } else {
            reset_active_ = false;
            update_state_command_(switch_left, switch_right);
            if (arm_sequence_.armed()) {
                update_mode_(switch_left, switch_right, keyboard);
                update_remote_control_();
            } else {
                *mode_ = rmcs_msgs::ChassisMode::AUTO;
                stop_controls_(1);
            }
        }

        last_switch_left_ = switch_left;
        last_switch_right_ = switch_right;
        last_keyboard_ = keyboard;
    }

private:
    using Clock = std::chrono::steady_clock;

    void register_interfaces_() {
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/wheel_leg/dr16_fresh", remote_fresh_);
        register_input("/remote/rotary_knob", rotary_knob_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/predefined/timestamp", timestamp_);

        register_output(
            "/chassis/control_velocity", chassis_control_velocity_,
            rmcs_description::BaseLink::DirectionVector{0.0, 0.0, 0.0});
        register_output("/chassis/control_height", chassis_control_height_, 0.0);
        register_output("/chassis/control_state", chassis_control_state_, 0);
        register_output("/chassis/reset_count", reset_count_output_, std::size_t{0});
        register_output("/chassis/control_mode", mode_, rmcs_msgs::ChassisMode::AUTO);
        register_output("/chassis/jump_request", jump_request_, false);
        register_output("/chassis/jump_apex_delta", jump_apex_delta_, 0.0);
    }

    void load_parameters_() {
        vx_max_ = get_parameter_or<double>("vx_max", 0.5);
        vy_max_ = get_parameter_or<double>("vy_max", 0.0);
        yaw_rate_max_ = get_parameter_or<double>("yaw_rate_max", 1.0);
        spin_yaw_rate_ = get_parameter_or<double>("spin_yaw_rate", 1.0);
        deadzone_ = get_parameter_or<double>("deadzone", 0.08);

        command_height_min_ = get_parameter_or<double>("command_height_min", 0.23);
        command_height_max_ = get_parameter_or<double>("command_height_max", 0.43);
        default_command_height_ = get_parameter_or<double>("default_command_height", 0.305);
        height_rate_max_ = get_parameter_or<double>("height_rate_max", 0.02);
        height_step_ = get_parameter_or<double>("height_step", 0.01);

        angular_z_invert_ = get_parameter_or<bool>("angular_z_invert", false);
        height_invert_ = get_parameter_or<bool>("height_invert", true);
        jump_enabled_ = get_parameter_or<bool>("jump_enabled", false);

        const std::array values{
            vx_max_,
            vy_max_,
            yaw_rate_max_,
            spin_yaw_rate_,
            deadzone_,
            command_height_min_,
            command_height_max_,
            default_command_height_,
            height_rate_max_,
            height_step_};
        if (!std::ranges::all_of(values, [](double value) { return std::isfinite(value); })
            || vx_max_ <= 0.0 || vy_max_ < 0.0 || yaw_rate_max_ <= 0.0 || spin_yaw_rate_ <= 0.0
            || spin_yaw_rate_ > yaw_rate_max_ || deadzone_ < 0.0 || deadzone_ >= 1.0
            || command_height_min_ <= 0.0 || command_height_min_ > command_height_max_
            || default_command_height_ < command_height_min_
            || default_command_height_ > command_height_max_ || height_rate_max_ <= 0.0
            || height_step_ < 0.0)
            throw std::invalid_argument("Invalid wheel-leg chassis command limits");
    }

    void update_mode_(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right,
        const rmcs_msgs::Keyboard& keyboard) {
        using rmcs_msgs::Switch;
        const bool spin_switch = switch_left == Switch::MIDDLE && switch_right == Switch::DOWN;
        const bool spin_switch_edge =
            spin_switch
            && (last_switch_left_ != Switch::MIDDLE || last_switch_right_ != Switch::DOWN);
        if (spin_switch_edge || (!last_keyboard_.c && keyboard.c)) {
            if (*mode_ == rmcs_msgs::ChassisMode::SPIN_FAST) {
                *mode_ = rmcs_msgs::ChassisMode::AUTO;
            } else {
                *mode_ = rmcs_msgs::ChassisMode::SPIN_FAST;
                spinning_forward_ = !spinning_forward_;
            }
        }
    }

    void reset_all_controls_(rmcs_msgs::Switch left, rmcs_msgs::Switch right) {
        if (!reset_active_) {
            *reset_count_output_ += 1;
            reset_active_ = true;
        }
        arm_sequence_.update(left, right, true);
        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        stop_controls_(1);
    }

    void stop_controls_(int state) {
        chassis_control_velocity_->vector << 0.0, 0.0, 0.0;
        *chassis_control_height_ = default_command_height_;
        *chassis_control_state_ = state;
        *jump_request_ = false;
        *jump_apex_delta_ = 0.0;
        height_ = default_command_height_;
        previous_height_timestamp_.reset();
    }

    void update_remote_control_() {
        update_velocity_control_();
        update_height_control_();
        // Hold the request while V is held; RL owns the elapsed time and release transition.
        *jump_request_ = jump_enabled_ && keyboard_->v;
        *jump_apex_delta_ = *jump_request_ ? (keyboard_->shift ? 0.10 : 0.06) : 0.0;
    }

    // Require double DOWN before arming; a single MIDDLE cannot start PREPARE.
    void update_state_command_(rmcs_msgs::Switch left, rmcs_msgs::Switch right) {
        arm_sequence_.update(left, right, true);
        *chassis_control_state_ = arm_sequence_.armed() ? 3 : 1;
    }

    void update_velocity_control_() {
        // Pure SPIN has no translation reference, including keyboard and right-stick input.
        if (*mode_ == rmcs_msgs::ChassisMode::SPIN_FAST) {
            const double yaw_rate = spinning_forward_ ? spin_yaw_rate_ : -spin_yaw_rate_;
            chassis_control_velocity_->vector << 0.0, 0.0,
                (angular_z_invert_ ? -yaw_rate : yaw_rate);
            return;
        }
        const Eigen::Vector2d command = read_translational_command_();
        const double yaw_rate = read_channel_(joystick_left_->y()) * yaw_rate_max_;
        chassis_control_velocity_->vector << command.x() * vx_max_, command.y() * vy_max_,
            (angular_z_invert_ ? -yaw_rate : yaw_rate);
    }

    Eigen::Vector2d read_translational_command_() const {
        const auto keyboard = *keyboard_;
        Eigen::Vector2d command{
            read_channel_(joystick_right_->x()) + keyboard.w - keyboard.s,
            read_channel_(joystick_right_->y()) + keyboard.a - keyboard.d};
        const double magnitude = command.norm();
        if (magnitude > 1.0)
            command /= magnitude;
        return command;
    }

    double read_channel_(double channel) const {
        if (!std::isfinite(channel) || std::abs(channel) < deadzone_)
            return 0.0;
        return std::clamp(channel, -1.0, 1.0);
    }

    void update_height_control_() {
        double rotary_knob = read_channel_(*rotary_knob_);
        if (height_invert_)
            rotary_knob = -rotary_knob;
        if (rotary_knob > 0.0)
            rotary_knob = (rotary_knob - deadzone_) / (1.0 - deadzone_);
        else if (rotary_knob < 0.0)
            rotary_knob = (rotary_knob + deadzone_) / (1.0 - deadzone_);
        const auto now = *timestamp_;
        double dt = 0.0;
        if (previous_height_timestamp_) {
            const auto elapsed = now - *previous_height_timestamp_;
            if (elapsed > Clock::duration::zero() && elapsed <= std::chrono::milliseconds{20})
                dt = std::chrono::duration<double>{elapsed}.count();
        }
        // Do not integrate a startup/reset interval or catch up after a gap.
        // Centered input holds the accumulated height in both AUTO and SPIN.
        previous_height_timestamp_ = now;
        height_ += rotary_knob * height_rate_max_ * dt;
        const auto& keyboard = *keyboard_;
        if (!last_keyboard_.r && keyboard.r)
            height_ += height_step_;
        if (!last_keyboard_.f && keyboard.f)
            height_ -= height_step_;
        height_ = std::clamp(height_, command_height_min_, command_height_max_);
        *chassis_control_height_ = height_;
    }

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<bool> remote_fresh_;
    InputInterface<double> rotary_knob_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<Clock::time_point> timestamp_;

    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;
    OutputInterface<double> chassis_control_height_;
    OutputInterface<int> chassis_control_state_;
    OutputInterface<std::size_t> reset_count_output_;

    OutputInterface<rmcs_msgs::ChassisMode> mode_;
    OutputInterface<bool> jump_request_;
    OutputInterface<double> jump_apex_delta_;

    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    double vx_max_ = 0.5;
    double vy_max_ = 0.0;
    double yaw_rate_max_ = 1.0;
    double spin_yaw_rate_ = 1.0;
    double deadzone_ = 0.08;
    double command_height_min_ = 0.23;
    double command_height_max_ = 0.43;
    double default_command_height_ = 0.305;
    double height_rate_max_ = 0.02;
    double height_step_ = 0.01;
    bool angular_z_invert_ = false;
    bool height_invert_ = true;
    bool jump_enabled_ = false;

    bool spinning_forward_ = true;
    bool switch_activity_seen_ = false;
    WheelLegArmSequence arm_sequence_;
    bool reset_active_ = false;
    double height_ = 0.0;
    std::optional<Clock::time_point> previous_height_timestamp_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegChassisController, rmcs_executor::Component)
