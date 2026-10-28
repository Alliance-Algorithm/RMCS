#pragma once

#include <algorithm>
#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::chassis {

// Manages wheel-leg chassis mode state, arming sequence, spin mode, height/speed/jump parameters,
// and safety interlock (disarm when all remotes disconnected).
class WheelLegModeManager {
public:
    explicit WheelLegModeManager(rclcpp::Node& node)
        : command_height_min_(node.get_parameter_or("command_height_min", 0.23))
        , command_height_max_(node.get_parameter_or("command_height_max", 0.43))
        , default_command_height_(
              node.get_parameter_or("default_command_height", command_height_min_))
        , height_step_(node.get_parameter_or("height_step", 0.01))
        , jump_enabled_(node.get_parameter_or("jump_enabled", false)) {
        height_ = default_command_height_;
    }

    // Update mode state from remote inputs. dr16_fresh and vt13_valid track remote connection.
    // When both are false, safety interlock disarms the chassis.
    void update(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right,
        const rmcs_msgs::Keyboard& keyboard, bool dr16_fresh, bool vt13_valid) {

        using rmcs_msgs::Switch;

        // Safety: If both remotes are disconnected, treat as UNKNOWN.
        const bool any_remote_connected = dr16_fresh || vt13_valid;
        const auto effective_left = any_remote_connected ? switch_left : Switch::UNKNOWN;
        const auto effective_right = any_remote_connected ? switch_right : Switch::UNKNOWN;

        const bool both_down = effective_left == Switch::DOWN && effective_right == Switch::DOWN;
        const bool any_unknown =
            effective_left == Switch::UNKNOWN || effective_right == Switch::UNKNOWN;

        // Disarm if remote lost or both down
        if (any_unknown || both_down) {
            if (armed_)
                disarm_reason_ = any_unknown ? "All remotes disconnected" : "Both switches DOWN";
            armed_ = false;
            pending_ = both_down;
            reset_to_defaults_();
            last_switch_left_ = effective_left;
            last_switch_right_ = effective_right;
            last_keyboard_ = keyboard;
            return;
        }

        // Arming sequence: hold DOWN/DOWN -> double MIDDLE
        if (both_down) {
            pending_ = true;
            armed_ = false;
        } else if (effective_left == Switch::MIDDLE && effective_right == Switch::MIDDLE) {
            if (pending_) {
                armed_ = true;
                pending_ = false;
                disarm_reason_.clear();
            }
        } else if (armed_ && effective_left == Switch::MIDDLE && effective_right == Switch::DOWN) {
            // Spin switch combination: can continue armed session, but cannot start one
        } else {
            armed_ = false;
            if ((effective_left != Switch::DOWN && effective_left != Switch::MIDDLE)
                || (effective_right != Switch::DOWN && effective_right != Switch::MIDDLE))
                pending_ = false;
        }

        // Mode switching (only when armed)
        if (armed_)
            update_mode_(effective_left, effective_right, keyboard);

        last_switch_left_ = effective_left;
        last_switch_right_ = effective_right;
        last_keyboard_ = keyboard;
    }

    // Query methods
    bool armed() const { return armed_; }
    bool safe_to_control() const { return armed_; }
    const std::string& disarm_reason() const { return disarm_reason_; }

    rmcs_msgs::ChassisMode mode() const { return mode_; }
    bool spinning_forward() const { return spinning_forward_; }

    double height() const { return height_; }
    double command_height_min() const { return command_height_min_; }
    double command_height_max() const { return command_height_max_; }
    double default_command_height() const { return default_command_height_; }
    double height_step() const { return height_step_; }

    bool jump_enabled() const { return jump_enabled_; }

    // Height edits remain bounded by the configured command range.
    void set_height(double height) {
        height_ = std::clamp(height, command_height_min_, command_height_max_);
    }
    void add_height_step() { set_height(height_ + height_step_); }
    void sub_height_step() { set_height(height_ - height_step_); }

    void reset() {
        armed_ = false;
        pending_ = false;
        reset_to_defaults_();
        last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
        last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
        last_keyboard_ = rmcs_msgs::Keyboard::zero();
        disarm_reason_.clear();
    }

private:
    void update_mode_(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right,
        const rmcs_msgs::Keyboard& keyboard) {
        using rmcs_msgs::Switch;

        // Match the common chassis input: the right switch enters DOWN once
        // while the left switch stays MIDDLE. Holding DOWN does not retrigger.
        const bool spin_switch_edge = switch_left == Switch::MIDDLE
                                   && last_switch_right_ == Switch::MIDDLE
                                   && switch_right == Switch::DOWN;

        if (spin_switch_edge || (!last_keyboard_.c && keyboard.c)) {
            if (mode_ == rmcs_msgs::ChassisMode::SPIN_FAST) {
                mode_ = rmcs_msgs::ChassisMode::AUTO;
            } else {
                mode_ = rmcs_msgs::ChassisMode::SPIN_FAST;
                spinning_forward_ = !spinning_forward_;
            }
        }
    }

    void reset_to_defaults_() {
        mode_ = rmcs_msgs::ChassisMode::AUTO;
        height_ = default_command_height_;
        spinning_forward_ = true;
    }

    // Parameters
    const double command_height_min_;
    const double command_height_max_;
    const double default_command_height_;
    const double height_step_;
    const bool jump_enabled_;

    // State
    bool armed_ = false;
    bool pending_ = false;
    rmcs_msgs::ChassisMode mode_ = rmcs_msgs::ChassisMode::AUTO;
    bool spinning_forward_ = true;
    double height_;

    // Last inputs
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    std::string disarm_reason_;
};

} // namespace rmcs_core::controller::chassis
