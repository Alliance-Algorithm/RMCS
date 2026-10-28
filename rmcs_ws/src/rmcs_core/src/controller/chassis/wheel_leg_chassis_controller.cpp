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

#include "wheel_leg/mode.hpp"

namespace rmcs_core::controller::chassis {

// Chassis command source of the wheel-leg. It decodes the remote control into the chassis command
// interfaces. WheelLegController owns PREPARE -> RL, so the executor graph stays acyclic.
class WheelLegChassisController final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    explicit WheelLegChassisController()
        : Node{get_component_name(), node::options()}
        , vx_max_(get_parameter_or<double>("vx_max", 0.5))
        , vy_max_(get_parameter_or<double>("vy_max", 0.0))
        , yaw_rate_max_(get_parameter_or<double>("yaw_rate_max", 1.0))
        , spin_yaw_rate_(get_parameter_or<double>("spin_yaw_rate", 1.0))
        , mode_mgr_(*this) {
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/wheel_leg/dr16_fresh", remote_fresh_);
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
        load_parameters_();
        stop_controls_(0);
    }

    void update() override {
        using rmcs_msgs::Switch;

        // RemoteControl outputs UNKNOWN when both DR16 and VT13 are disconnected.
        // dr16_fresh tracks DR16 connection; VT13 state is implicitly in switch values.
        const auto switch_right = *switch_right_;
        const auto switch_left = *switch_left_;
        const auto keyboard = *keyboard_;
        const bool dr16_fresh = *remote_fresh_;

        // VT13 is considered connected if switches are valid and dr16 is not fresh
        const bool vt13_valid = !dr16_fresh
                             && switch_left != Switch::UNKNOWN
                             && switch_right != Switch::UNKNOWN;

        // Power-on hold: stay at kInit(0) until the operator first moves a switch.
        if (!switch_activity_seen_
            && (switch_left != last_switch_left_ || switch_right != last_switch_right_))
            switch_activity_seen_ = true;

        *jump_request_ = false;
        *jump_apex_delta_ = 0.0;

        if (!switch_activity_seen_) {
            stop_controls_(0);
            mode_mgr_.reset();
        } else {
            // Update mode manager with remote connection state
            mode_mgr_.update(switch_left, switch_right, keyboard, dr16_fresh, vt13_valid);

            const bool any_unknown = switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN;
            if (any_unknown || !mode_mgr_.armed()) {
                if (!mode_mgr_.disarm_reason().empty() && mode_mgr_.armed() != last_armed_state_)
                    RCLCPP_WARN(get_logger(), "Chassis disarmed: %s", mode_mgr_.disarm_reason().c_str());
                reset_all_controls_();
            } else {
                reset_active_ = false;
                *chassis_control_state_ = 3;  // Armed
                update_remote_control_();
            }
        }

        last_armed_state_ = mode_mgr_.armed();
        last_switch_left_ = switch_left;
        last_switch_right_ = switch_right;
        last_keyboard_ = keyboard;
    }

private:
    using Clock = std::chrono::steady_clock;

    void load_parameters_() {
        deadzone_ = get_parameter_or<double>("deadzone", 0.05);
        height_rate_max_ = get_parameter_or<double>("height_rate_max", 0.02);
        angular_z_invert_ = get_parameter_or<bool>("angular_z_invert", false);
        height_invert_ = get_parameter_or<bool>("height_invert", false);

        const std::array values{vx_max_, vy_max_, yaw_rate_max_, spin_yaw_rate_, deadzone_, height_rate_max_};
        if (!std::ranges::all_of(values, [](double value) { return std::isfinite(value); })
            || vx_max_ <= 0.0 || vy_max_ < 0.0 || yaw_rate_max_ <= 0.0 || spin_yaw_rate_ <= 0.0
            || deadzone_ < 0.0 || deadzone_ >= 1.0 || height_rate_max_ <= 0.0)
            throw std::invalid_argument("Invalid wheel-leg chassis command limits");
    }

    void reset_all_controls_() {
        if (!reset_active_) {
            *reset_count_output_ += 1;
            reset_active_ = true;
        }
        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        stop_controls_(1);
    }

    void stop_controls_(int state) {
        chassis_control_velocity_->vector << 0.0, 0.0, 0.0;
        *chassis_control_height_ = mode_mgr_.default_command_height();
        *chassis_control_state_ = state;
        *jump_request_ = false;
        *jump_apex_delta_ = 0.0;
        previous_height_timestamp_.reset();
    }

    void update_remote_control_() {
        *mode_ = mode_mgr_.mode();
        update_velocity_control_();
        update_height_control_();
        // Hold the request while V is held; RL owns the elapsed time and release transition.
        const bool jump_enabled = mode_mgr_.jump_enabled();
        *jump_request_ = jump_enabled && keyboard_->v;
        *jump_apex_delta_ = *jump_request_ ? (keyboard_->shift ? 0.10 : 0.06) : 0.0;
    }

    void update_velocity_control_() {
        // Pure SPIN has no translation reference, including keyboard and right-stick input.
        const auto mode = mode_mgr_.mode();
        if (mode == rmcs_msgs::ChassisMode::SPIN_FAST) {
            const bool spinning_forward = mode_mgr_.spinning_forward();
            const double yaw_rate = spinning_forward ? spin_yaw_rate_ : -spin_yaw_rate_;
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
        double height_input = read_channel_(joystick_left_->x());
        if (height_invert_)
            height_input = -height_input;
        if (height_input > 0.0)
            height_input = (height_input - deadzone_) / (1.0 - deadzone_);
        else if (height_input < 0.0)
            height_input = (height_input + deadzone_) / (1.0 - deadzone_);
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

        double height = mode_mgr_.height() + height_input * height_rate_max_ * dt;
        const auto& keyboard = *keyboard_;
        if (!last_keyboard_.r && keyboard.r)
            mode_mgr_.add_height_step();
        else if (!last_keyboard_.f && keyboard.f)
            mode_mgr_.sub_height_step();
        else
            mode_mgr_.set_height(height);

        *chassis_control_height_ = mode_mgr_.height();
    }

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<bool> remote_fresh_;
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
    double deadzone_ = 0.05;
    double height_rate_max_ = 0.02;
    bool angular_z_invert_ = false;
    bool height_invert_ = false;

    WheelLegModeManager mode_mgr_;
    bool switch_activity_seen_ = false;
    bool reset_active_ = false;
    bool last_armed_state_ = false;
    std::optional<Clock::time_point> previous_height_timestamp_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegChassisController, rmcs_executor::Component)
