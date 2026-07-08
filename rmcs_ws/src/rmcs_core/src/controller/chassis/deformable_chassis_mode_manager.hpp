#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

#include <rclcpp/node.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::chassis {

class DeformableChassisModeManager {
public:
    struct JointPostureState {
        rmcs_msgs::ChassisMode mode = rmcs_msgs::ChassisMode::AUTO;
        bool ctrl_low_prone_active = false;
        bool pitch_lock_active = false;
        bool suspension_active = false;
        bool symmetric_posture_target = true;
        bool spinning_forward = true;
        std::array<double, 4> joint_posture_target_deg = {58.0, 58.0, 58.0, 58.0};
        double suspension_reference_angle_deg = 58.0;
    };

    explicit DeformableChassisModeManager(rclcpp::Node& node)
        : min_angle_(node.get_parameter_or("min_angle", 7.0))
        , max_angle_(node.get_parameter_or("max_angle", 58.0))
        , suspension_enable_(node.get_parameter_or("active_suspension_enable", false)) {
        current_target_angle_ = max_angle_;
        joint_current_target_angle_.fill(max_angle_);
        update_joint_posture_state_(false);
    }

    void reset() {
        joint_posture_state_.mode = rmcs_msgs::ChassisMode::AUTO;
        joint_posture_state_.ctrl_low_prone_active = false;
        joint_posture_state_.pitch_lock_active = false;
        joint_posture_state_.suspension_active = false;
        joint_posture_state_.symmetric_posture_target = true;
        joint_posture_state_.spinning_forward = true;
        joint_posture_state_.joint_posture_target_deg.fill(max_angle_);
        joint_posture_state_.suspension_reference_angle_deg = max_angle_;

        current_target_angle_ = max_angle_;
        joint_current_target_angle_.fill(max_angle_);
        apply_symmetric_target_ = true;
        complex_spin_active_ = false;
        complex_spin_elapsed_ = 0.0;
        suspension_enabled_by_toggle_ = false;
        suspension_toggle_left_down_pending_ = false;

        last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
        last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
        last_keyboard_ = rmcs_msgs::Keyboard::zero();
        last_rotary_knob_ = 0.0;

        update_joint_posture_state_(false);
    }

    void update(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right,
        const rmcs_msgs::Keyboard& keyboard, double rotary_knob, double dt) {

        update_mode_from_inputs_(switch_left, switch_right, keyboard);
        update_suspension_toggle_from_inputs_(switch_left, switch_right, keyboard);
        update_lift_target_toggle_(keyboard, rotary_knob, dt);

        joint_posture_state_.ctrl_low_prone_active = keyboard.ctrl;
        joint_posture_state_.pitch_lock_active = keyboard.ctrl;
        joint_posture_state_.suspension_active =
            suspension_enable_ && (keyboard.ctrl || suspension_enabled_by_toggle_);
        update_joint_posture_state_(keyboard.ctrl);

        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;
        last_keyboard_ = keyboard;
    }

    const JointPostureState& joint_posture_state() const { return joint_posture_state_; }

    double min_angle() const { return min_angle_; }
    double max_angle() const { return max_angle_; }
    double max_angle_rad() const { return deg_to_rad_(max_angle_); }

    double active_suspension_min_angle_rad() const { return deg_to_rad_(min_angle_ - 5.0); }

    bool correction_inverted() const {
        double midpoint = (min_angle_ - 5.0 + max_angle_) / 2.0;
        return joint_posture_state_.suspension_reference_angle_deg > midpoint;
    }

private:
    static double deg_to_rad_(double deg) { return deg * std::numbers::pi / 180.0; }

    void update_mode_from_inputs_(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right,
        const rmcs_msgs::Keyboard& keyboard) {

        auto mode = joint_posture_state_.mode;
        if (switch_left == rmcs_msgs::Switch::DOWN) {
            deactivate_complex_spin_();
            joint_posture_state_.mode = mode;
            return;
        }

        if (last_switch_right_ == rmcs_msgs::Switch::MIDDLE
            && switch_right == rmcs_msgs::Switch::DOWN) {
            deactivate_complex_spin_();
            if (mode == rmcs_msgs::ChassisMode::SPIN) {
                mode = rmcs_msgs::ChassisMode::STEP_DOWN;
            } else {
                mode = rmcs_msgs::ChassisMode::SPIN;
                joint_posture_state_.spinning_forward = !joint_posture_state_.spinning_forward;
            }
        } else if (!last_keyboard_.c && keyboard.c) {
            deactivate_complex_spin_();
            if (mode == rmcs_msgs::ChassisMode::SPIN) {
                mode = rmcs_msgs::ChassisMode::AUTO;
            } else {
                mode = rmcs_msgs::ChassisMode::SPIN;
                joint_posture_state_.spinning_forward = !joint_posture_state_.spinning_forward;
            }
        } else if (!last_keyboard_.z && keyboard.z) {
            deactivate_complex_spin_();
            mode = mode == rmcs_msgs::ChassisMode::STEP_DOWN ? rmcs_msgs::ChassisMode::AUTO
                                                             : rmcs_msgs::ChassisMode::STEP_DOWN;
        }

        if (complex_spin_active_)
            mode = rmcs_msgs::ChassisMode::SPIN;

        joint_posture_state_.mode = mode;
    }

    void activate_complex_spin_(rmcs_msgs::ChassisMode& mode) {
        complex_spin_active_ = true;
        complex_spin_elapsed_ = 0.0;
        apply_symmetric_target_ = true;
        if (mode != rmcs_msgs::ChassisMode::SPIN) {
            mode = rmcs_msgs::ChassisMode::SPIN;
            joint_posture_state_.spinning_forward = !joint_posture_state_.spinning_forward;
        }
    }

    void deactivate_complex_spin_() {
        complex_spin_active_ = false;
        complex_spin_elapsed_ = 0.0;
    }

    void apply_front_high_rear_low_target_() {
        joint_current_target_angle_[kLeftFront] = max_angle_;
        joint_current_target_angle_[kRightFront] = max_angle_;
        joint_current_target_angle_[kLeftBack] = min_angle_;
        joint_current_target_angle_[kRightBack] = min_angle_;
        apply_symmetric_target_ = false;
    }

    void apply_front_low_rear_high_target_() {
        joint_current_target_angle_[kLeftFront] = min_angle_;
        joint_current_target_angle_[kRightFront] = min_angle_;
        joint_current_target_angle_[kLeftBack] = max_angle_;
        joint_current_target_angle_[kRightBack] = max_angle_;
        apply_symmetric_target_ = false;
    }

    void toggle_bg_target_() {
        if (joint_current_target_angle_[kLeftFront] > joint_current_target_angle_[kLeftBack])
            apply_front_low_rear_high_target_();
        else
            apply_front_high_rear_low_target_();
    }

    void update_suspension_toggle_from_inputs_(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right,
        const rmcs_msgs::Keyboard& keyboard) {
        const bool keyboard_toggle_requested = !last_keyboard_.e && keyboard.e;
        if (keyboard_toggle_requested
            || suspension_toggle_requested_by_switch_(switch_left, switch_right))
            suspension_enabled_by_toggle_ = !suspension_enabled_by_toggle_;
    }

    bool suspension_toggle_requested_by_switch_(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right) {
        if (last_switch_left_ != rmcs_msgs::Switch::DOWN && switch_left == rmcs_msgs::Switch::DOWN)
            suspension_toggle_left_down_pending_ = true;

        const bool right_entered_up =
            last_switch_right_ != rmcs_msgs::Switch::UP && switch_right == rmcs_msgs::Switch::UP;
        if (suspension_toggle_left_down_pending_ && switch_left == rmcs_msgs::Switch::DOWN
            && right_entered_up) {
            suspension_toggle_left_down_pending_ = false;
            return true;
        }

        if (switch_left != rmcs_msgs::Switch::DOWN)
            suspension_toggle_left_down_pending_ = false;

        return false;
    }

    void update_lift_target_toggle_(
        const rmcs_msgs::Keyboard& keyboard, double rotary_knob, double dt) {

        constexpr double rotary_knob_symmetric_edge_threshold = 0.7;
        constexpr double rotary_knob_bg_edge_threshold = -0.9;
        constexpr double complex_spin_toggle_period = 0.5;

        const bool keyboard_toggle_condition = !last_keyboard_.q && keyboard.q;

        const bool rotary_knob_toggle_condition =
            last_rotary_knob_ < rotary_knob_symmetric_edge_threshold
            && rotary_knob >= rotary_knob_symmetric_edge_threshold;

        const bool rotary_knob_bg_toggle_condition =
            last_rotary_knob_ > rotary_knob_bg_edge_threshold
            && rotary_knob <= rotary_knob_bg_edge_threshold;

        const bool front_high_rear_low = !last_keyboard_.b && keyboard.b;
        const bool front_low_rear_high = !last_keyboard_.g && keyboard.g;

        bool complex_spin_toggle_condition = false;
        if (complex_spin_active_) {
            complex_spin_elapsed_ += dt;
            size_t complex_spin_toggle_count = 0;
            while (complex_spin_elapsed_ >= complex_spin_toggle_period) {
                complex_spin_elapsed_ -= complex_spin_toggle_period;
                ++complex_spin_toggle_count;
            }
            complex_spin_toggle_condition = (complex_spin_toggle_count % 2) == 1;
        }

        if (apply_symmetric_target_)
            joint_current_target_angle_.fill(current_target_angle_);

        if (rotary_knob_toggle_condition || keyboard_toggle_condition
            || complex_spin_toggle_condition) {
            current_target_angle_ =
                (std::abs(current_target_angle_ - max_angle_) < 1e-6) ? min_angle_ : max_angle_;
            apply_symmetric_target_ = true;
            joint_current_target_angle_.fill(current_target_angle_);
        } else if (rotary_knob_bg_toggle_condition) {
            toggle_bg_target_();
        } else if (front_high_rear_low) {
            apply_front_high_rear_low_target_();
        } else if (front_low_rear_high) {
            apply_front_low_rear_high_target_();
        }

        last_rotary_knob_ = rotary_knob;
    }

    static constexpr size_t kLeftFront = 0;
    static constexpr size_t kLeftBack = 1;
    static constexpr size_t kRightBack = 2;
    static constexpr size_t kRightFront = 3;
    static constexpr size_t kJointCount = 4;

    void update_joint_posture_state_(bool ctrl_low_prone_active) {
        std::array<double, kJointCount> effective_joint_posture_target_deg =
            joint_current_target_angle_;
        if (ctrl_low_prone_active)
            effective_joint_posture_target_deg.fill(min_angle_ - 5.0);

        joint_posture_state_.joint_posture_target_deg = effective_joint_posture_target_deg;
        joint_posture_state_.symmetric_posture_target =
            symmetric_joint_target_requested_(effective_joint_posture_target_deg);

        if (joint_posture_state_.symmetric_posture_target) {
            joint_posture_state_.suspension_reference_angle_deg =
                effective_joint_posture_target_deg.front();
            return;
        }

        double posture_angle_sum = 0.0;
        for (double angle_deg : effective_joint_posture_target_deg)
            posture_angle_sum += angle_deg;
        joint_posture_state_.suspension_reference_angle_deg =
            posture_angle_sum / static_cast<double>(kJointCount);
    }

    static bool
        symmetric_joint_target_requested_(const std::array<double, kJointCount>& joint_target_deg) {
        constexpr double epsilon = 1e-6;
        return std::all_of(joint_target_deg.begin() + 1, joint_target_deg.end(), [&](double v) {
            return std::abs(v - joint_target_deg.front()) <= epsilon;
        });
    }

    JointPostureState joint_posture_state_;

    double min_angle_;
    double max_angle_;
    bool suspension_enable_;

    double current_target_angle_;
    std::array<double, kJointCount> joint_current_target_angle_;
    bool apply_symmetric_target_ = true;
    bool complex_spin_active_ = false;
    double complex_spin_elapsed_ = 0.0;
    bool suspension_enabled_by_toggle_ = false;
    bool suspension_toggle_left_down_pending_ = false;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();
    double last_rotary_knob_ = 0.0;
};

} // namespace rmcs_core::controller::chassis
