#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>

#include <rclcpp/node.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::chassis {

class DeformableChassisModeManager {
public:
    enum class SuspensionMode : uint8_t {
        OFF = 0,
        ACTIVE = 1,
    };

    struct JointPostureState {
        rmcs_msgs::ChassisMode mode = rmcs_msgs::ChassisMode::AUTO;
        bool ctrl_low_prone_active = false;
        bool low_prone_active = false;
        bool pitch_lock_active = false;
        bool suspension_active = false;
        SuspensionMode suspension_mode = SuspensionMode::OFF;
        bool symmetric_posture_target = true;
        bool spinning_forward = true;
        std::array<double, 4> joint_posture_target_deg = {58.0, 58.0, 58.0, 58.0};
        double suspension_reference_angle_deg = 58.0;
    };

    explicit DeformableChassisModeManager(rclcpp::Node& node)
        : min_angle_(node.get_parameter_or("min_angle", 5.0))
        , max_angle_(node.get_parameter_or("max_angle", 59.0))
        , active_suspension_base_angle_(
              std::clamp(
                  node.get_parameter_or("active_suspension_base_angle", max_angle_),
                  min_angle_ - 5.0, max_angle_))
        , suspension_enable_(node.get_parameter_or("active_suspension_enable", false)) {
        current_target_angle_ = max_angle_;
        joint_current_target_angle_.fill(max_angle_);
        update_joint_posture_state_(false);
    }

    void reset() {
        joint_posture_state_.mode = rmcs_msgs::ChassisMode::AUTO;
        joint_posture_state_.ctrl_low_prone_active = false;
        joint_posture_state_.low_prone_active = false;
        joint_posture_state_.pitch_lock_active = false;
        joint_posture_state_.suspension_active = false;
        joint_posture_state_.suspension_mode = SuspensionMode::OFF;
        joint_posture_state_.symmetric_posture_target = true;
        joint_posture_state_.spinning_forward = true;
        joint_posture_state_.joint_posture_target_deg.fill(max_angle_);
        joint_posture_state_.suspension_reference_angle_deg = max_angle_;

        current_target_angle_ = max_angle_;
        active_suspension_base_angle_ = max_angle_;
        joint_current_target_angle_.fill(max_angle_);
        apply_symmetric_target_ = true;
        suspension_enabled_by_toggle_ = false;
        low_prone_enabled_by_toggle_ = false;

        last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
        last_keyboard_ = rmcs_msgs::Keyboard::zero();
        last_rotary_knob_ = 0.0;

        update_joint_posture_state_(false);
    }

    void update(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right,
        const rmcs_msgs::Keyboard& keyboard, double rotary_knob, double dt) {

        update_mode_from_inputs_(switch_left, switch_right, keyboard);
        update_low_prone_toggle_from_inputs_(switch_left, switch_right);

        joint_posture_state_.ctrl_low_prone_active = keyboard.ctrl;
        joint_posture_state_.low_prone_active =
            joint_posture_state_.ctrl_low_prone_active || low_prone_enabled_by_toggle_;
        joint_posture_state_.pitch_lock_active = joint_posture_state_.ctrl_low_prone_active;

        update_suspension_mode_from_inputs_(switch_left, switch_right, keyboard, rotary_knob);
        update_posture_target_from_inputs_(switch_left, switch_right, keyboard, rotary_knob, dt);
        update_joint_posture_state_(joint_posture_state_.low_prone_active);

        last_switch_right_ = switch_right;
        last_keyboard_ = keyboard;
    }

    rmcs_msgs::ChassisMode mode() const { return joint_posture_state_.mode; }
    bool pitch_lock_active() const { return joint_posture_state_.pitch_lock_active; }
    bool suspension_active() const { return joint_posture_state_.suspension_active; }
    bool low_prone_active() const { return joint_posture_state_.low_prone_active; }
    bool symmetric_posture_target() const { return joint_posture_state_.symmetric_posture_target; }
    bool spinning_forward() const { return joint_posture_state_.spinning_forward; }
    double suspension_reference_angle_deg() const {
        return joint_posture_state_.suspension_reference_angle_deg;
    }
    void copy_joint_posture_target_deg(std::array<double, 4>& out) const {
        out = joint_posture_state_.joint_posture_target_deg;
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
    static constexpr size_t kLeftFront = 0;
    static constexpr size_t kLeftBack = 1;
    static constexpr size_t kRightBack = 2;
    static constexpr size_t kRightFront = 3;
    static constexpr size_t kJointCount = 4;

    static double deg_to_rad_(double deg) { return deg * std::numbers::pi / 180.0; }

    static bool
        symmetric_joint_target_requested_(const std::array<double, kJointCount>& joint_target_deg) {
        constexpr double epsilon = 1e-6;
        return std::all_of(joint_target_deg.begin() + 1, joint_target_deg.end(), [&](double v) {
            return std::abs(v - joint_target_deg.front()) <= epsilon;
        });
    }

    void update_mode_from_inputs_(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right,
        const rmcs_msgs::Keyboard& keyboard) {

        auto next_mode = joint_posture_state_.mode;
        if (switch_left == rmcs_msgs::Switch::DOWN) {
            joint_posture_state_.mode = next_mode;
            return;
        }

        if (last_switch_right_ == rmcs_msgs::Switch::MIDDLE
            && switch_right == rmcs_msgs::Switch::DOWN) {
            if (next_mode == rmcs_msgs::ChassisMode::SPIN_FAST) {
                next_mode = rmcs_msgs::ChassisMode::STEP_DOWN;
            } else {
                next_mode = rmcs_msgs::ChassisMode::SPIN_FAST;
                joint_posture_state_.spinning_forward = !joint_posture_state_.spinning_forward;
            }
        } else if (!last_keyboard_.c && keyboard.c) {
            if (next_mode == rmcs_msgs::ChassisMode::SPIN_FAST) {
                next_mode = rmcs_msgs::ChassisMode::AUTO;
            } else {
                next_mode = rmcs_msgs::ChassisMode::SPIN_FAST;
                joint_posture_state_.spinning_forward = !joint_posture_state_.spinning_forward;
            }
        } else if (!last_keyboard_.z && keyboard.z) {
            next_mode = next_mode == rmcs_msgs::ChassisMode::STEP_DOWN
                          ? rmcs_msgs::ChassisMode::AUTO
                          : rmcs_msgs::ChassisMode::STEP_DOWN;
        }

        joint_posture_state_.mode = next_mode;
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

    void toggle_front_back_posture_target_() {
        if (joint_current_target_angle_[kLeftFront] > joint_current_target_angle_[kLeftBack])
            apply_front_low_rear_high_target_();
        else
            apply_front_high_rear_low_target_();
    }

    void update_suspension_mode_from_inputs_(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right,
        const rmcs_msgs::Keyboard& keyboard, double rotary_knob) {
        const bool remote_suspension_rotary_mode =
            switch_left == rmcs_msgs::Switch::DOWN && switch_right == rmcs_msgs::Switch::MIDDLE;
        const bool remote_active_toggle_requested =
            remote_suspension_rotary_mode && rotary_knob_down_edge_(rotary_knob);

        const bool keyboard_active_suspension_toggle_requested = !last_keyboard_.e && keyboard.e;
        if (keyboard_active_suspension_toggle_requested || remote_active_toggle_requested)
            suspension_enabled_by_toggle_ = !suspension_enabled_by_toggle_;

        const bool active_requested =
            suspension_enable_
            && (joint_posture_state_.low_prone_active || suspension_enabled_by_toggle_);

        joint_posture_state_.suspension_mode = SuspensionMode::OFF;
        if (active_requested)
            joint_posture_state_.suspension_mode = SuspensionMode::ACTIVE;

        joint_posture_state_.suspension_active =
            joint_posture_state_.suspension_mode == SuspensionMode::ACTIVE;
    }

    void update_low_prone_toggle_from_inputs_(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right) {
        if (switch_left == rmcs_msgs::Switch::DOWN && switch_right == rmcs_msgs::Switch::UP
            && last_switch_right_ == rmcs_msgs::Switch::MIDDLE) {
            low_prone_enabled_by_toggle_ = !low_prone_enabled_by_toggle_;
        }
    }

    void update_posture_target_from_inputs_(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right,
        const rmcs_msgs::Keyboard& keyboard, double rotary_knob, double /*dt*/) {
        const bool remote_joint_posture_rotary_mode =
            switch_left == rmcs_msgs::Switch::MIDDLE && switch_right == rmcs_msgs::Switch::MIDDLE;

        const bool keyboard_posture_toggle_condition = !last_keyboard_.q && keyboard.q;
        const bool remote_posture_toggle_condition =
            remote_joint_posture_rotary_mode && rotary_knob_down_edge_(rotary_knob);
        const bool remote_front_back_posture_toggle_condition =
            remote_joint_posture_rotary_mode && rotary_knob_up_edge_(rotary_knob);
        const bool front_high_rear_low = !last_keyboard_.b && keyboard.b;
        const bool front_low_rear_high = !last_keyboard_.g && keyboard.g;

        if (apply_symmetric_target_)
            joint_current_target_angle_.fill(current_target_angle_);

        const bool posture_toggle_requested =
            remote_posture_toggle_condition || keyboard_posture_toggle_condition;

        if (posture_toggle_requested) {
            if (joint_posture_state_.suspension_active) {
                active_suspension_base_angle_ =
                    (std::abs(active_suspension_base_angle_ - max_angle_) < 1e-6) ? min_angle_
                                                                                  : max_angle_;
                current_target_angle_ = active_suspension_base_angle_;
                apply_symmetric_target_ = true;
                joint_current_target_angle_.fill(current_target_angle_);
            } else {
                current_target_angle_ =
                    (std::abs(current_target_angle_ - max_angle_) < 1e-6) ? min_angle_ : max_angle_;
                apply_symmetric_target_ = true;
                joint_current_target_angle_.fill(current_target_angle_);
            }
        } else if (remote_front_back_posture_toggle_condition) {
            toggle_front_back_posture_target_();
        } else if (front_high_rear_low) {
            apply_front_high_rear_low_target_();
        } else if (front_low_rear_high) {
            apply_front_low_rear_high_target_();
        }

        last_rotary_knob_ = rotary_knob;
    }

    bool rotary_knob_down_edge_(double rotary_knob) const {
        constexpr double rotary_knob_edge_threshold = 0.7;
        return last_rotary_knob_ < rotary_knob_edge_threshold
            && rotary_knob >= rotary_knob_edge_threshold;
    }

    bool rotary_knob_up_edge_(double rotary_knob) const {
        constexpr double rotary_knob_edge_threshold = 0.7;
        return last_rotary_knob_ > -rotary_knob_edge_threshold
            && rotary_knob <= -rotary_knob_edge_threshold;
    }

    void update_joint_posture_state_(bool low_prone_active) {
        std::array<double, kJointCount> effective_joint_posture_target_deg =
            joint_current_target_angle_;
        if (low_prone_active)
            effective_joint_posture_target_deg.fill(min_angle_ - 5.0);

        joint_posture_state_.joint_posture_target_deg = effective_joint_posture_target_deg;
        joint_posture_state_.symmetric_posture_target =
            symmetric_joint_target_requested_(effective_joint_posture_target_deg);

        if (joint_posture_state_.suspension_active) {
            joint_posture_state_.suspension_reference_angle_deg =
                low_prone_active ? min_angle_ : active_suspension_base_angle_;
            return;
        }

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

    JointPostureState joint_posture_state_;

    double min_angle_;
    double max_angle_;
    double active_suspension_base_angle_;
    bool suspension_enable_;

    double current_target_angle_;
    std::array<double, kJointCount> joint_current_target_angle_;
    bool apply_symmetric_target_ = true;
    bool suspension_enabled_by_toggle_ = false;
    bool low_prone_enabled_by_toggle_ = false;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();
    double last_rotary_knob_ = 0.0;
};

} // namespace rmcs_core::controller::chassis
