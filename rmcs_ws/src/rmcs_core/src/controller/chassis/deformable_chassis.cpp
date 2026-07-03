#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <numbers>
#include <stdexcept>

#include <eigen3/Eigen/Dense>
#include <fmt/format.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::chassis {

class DeformableChassis
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    enum JointIndex : size_t {
        kLeftFront = 0,
        kLeftBack = 1,
        kRightBack = 2,
        kRightFront = 3,
        kJointCount = 4,
    };

    DeformableChassis()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , following_velocity_controller_(10.0, 0.0, 0.0)
        , spin_ratio_(std::clamp(get_parameter_or("spin_ratio", 0.6), 0.0, 1.0))
        , launch_ramp_shortcut_enabled_(get_parameter_or("launch_ramp_shortcut_enabled", true))

        , min_angle_(get_parameter_or("min_angle", 7.0))
        , max_angle_(get_parameter_or("max_angle", 58.0))
        , target_physical_velocity_limit_(
              std::max(
                  deg_to_rad(std::abs(get_parameter_or("target_physical_velocity_limit", 180.0))),
                  1e-6))
        , target_physical_acceleration_limit_(
              std::max(
                  deg_to_rad(
                      std::abs(get_parameter_or("target_physical_acceleration_limit", 720.0))),
                  1e-6))
        , active_suspension_enable_(get_parameter_or("active_suspension_enable", false)) {

        following_velocity_controller_.output_max = angular_velocity_max_;
        following_velocity_controller_.output_min = -angular_velocity_max_;

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/rotary_knob", rotary_knob_);
        register_input("/predefined/update_rate", update_rate_);

        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_, false);
        register_input("/gimbal/yaw/control_angle_error", gimbal_yaw_angle_error_, false);
        register_input("/chassis/manual_yaw_velocity_override", manual_yaw_velocity_override_, false);

        for (size_t i = 0; i < kJointCount; ++i)
            register_input(
                fmt::format("/chassis/{}_joint/physical_angle", kJointNames[i]),
                joint_physical_angle_[i], false);
        register_output("/chassis/angle", chassis_angle_, nan_);
        register_output("/chassis/control_angle", chassis_control_angle_, nan_);

        register_output("/chassis/control_mode", mode_);
        register_output("/chassis/control_velocity", chassis_control_velocity_);
        register_output("/chassis/ctrl_hold_active", ctrl_hold_active_, false);

        for (size_t i = 0; i < kJointCount; ++i) {
            register_output(
                fmt::format("/chassis/{}_joint/base_control_angle_error", kJointNames[i]),
                joint_angle_error_[i], nan_);
            register_output(
                fmt::format("/chassis/{}_joint/base_target_physical_angle", kJointNames[i]),
                joint_target_physical_angle_[i], nan_);
            register_output(
                fmt::format("/chassis/{}_joint/base_target_physical_velocity", kJointNames[i]),
                joint_target_physical_velocity_[i], nan_);
            register_output(
                fmt::format("/chassis/{}_joint/base_target_physical_acceleration", kJointNames[i]),
                joint_target_physical_acceleration_[i], nan_);
        }

        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        *ctrl_hold_active_ = false;
        chassis_control_velocity_->vector << nan_, nan_, nan_;

        current_target_angle_ = max_angle_;
        lf_current_target_angle_ = max_angle_;
        lb_current_target_angle_ = max_angle_;
        rf_current_target_angle_ = max_angle_;
        rb_current_target_angle_ = max_angle_;
    }

    void before_updating() override {
        if (!gimbal_yaw_angle_.ready()) {
            gimbal_yaw_angle_.make_and_bind_directly(0.0);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/gimbal/yaw/angle\". Set to 0.0.");
        }
        if (!gimbal_yaw_angle_error_.ready()) {
            gimbal_yaw_angle_error_.make_and_bind_directly(0.0);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/gimbal/yaw/control_angle_error\". Set to 0.0.");
        }
        if (!manual_yaw_velocity_override_.ready())
            manual_yaw_velocity_override_.make_and_bind_directly(nan_);
        validate_joint_feedback_inputs();
    }

    void update() override {
        using rmcs_msgs::Switch;

        const auto switch_right = *switch_right_;
        const auto switch_left = *switch_left_;
        const auto keyboard = *keyboard_;

        do {
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                reset_all_controls();
                break;
            }

            update_mode_from_inputs_(switch_left, switch_right, keyboard);
            update_suspension_toggle_from_inputs_(switch_left, switch_right);
            *ctrl_hold_active_ = ctrl_hold_requested_by_input_();
            update_velocity_control();
            update_lift_target_toggle(keyboard);
            run_joint_intent_pipeline_();
        } while (false);

        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;
        last_keyboard_ = keyboard;
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double translational_velocity_max_ = 10.0;
    static constexpr double angular_velocity_max_ = 30.0;
    void validate_joint_feedback_inputs() const {
        for (size_t i = 0; i < kJointCount; ++i)
            if (!joint_physical_angle_[i].ready())
                throw std::runtime_error(
                    "missing deformable chassis feedback interfaces: expected "
                    "/chassis/*_joint/physical_angle");
    }

    void update_mode_from_inputs_(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right,
        const rmcs_msgs::Keyboard& keyboard) {
        auto mode = *mode_;
        const bool q_pressed = keyboard.q;
        const bool e_pressed = keyboard.e;
        const bool last_q_pressed = last_keyboard_.q;
        const bool last_e_pressed = last_keyboard_.e;
        const bool last_c_pressed = last_keyboard_.c;
        const bool qe_combo_pressed = q_pressed && e_pressed;
        const bool last_qe_combo_pressed = last_q_pressed && last_e_pressed;
        const bool e_rising = !last_e_pressed && e_pressed;
        const bool c_rising = !last_c_pressed && keyboard.c;
        const bool qe_combo_rising = !last_qe_combo_pressed && qe_combo_pressed;
        if (switch_left == rmcs_msgs::Switch::DOWN) {
            deactivate_complex_spin_();
            deactivate_qe_complex_spin_();
            return;
        }

        if (qe_complex_spin_active_) {
            if (c_rising) {
                deactivate_qe_complex_spin_();
                apply_symmetric_target = true;
                lf_current_target_angle_ = current_target_angle_;
                lb_current_target_angle_ = current_target_angle_;
                rb_current_target_angle_ = current_target_angle_;
                rf_current_target_angle_ = current_target_angle_;
                mode = rmcs_msgs::ChassisMode::SPIN;
            }
        } else if (qe_combo_rising) {
            deactivate_complex_spin_();
            activate_qe_complex_spin_(mode);
        } else if (e_rising && !q_pressed) {
            if (complex_spin_active_) {
                deactivate_complex_spin_();
                if (mode == rmcs_msgs::ChassisMode::SPIN)
                    mode = rmcs_msgs::ChassisMode::AUTO;
            } else {
                activate_complex_spin_(mode);
            }
        } else if (last_switch_right_ == rmcs_msgs::Switch::MIDDLE
            && switch_right == rmcs_msgs::Switch::DOWN) {
            deactivate_complex_spin_();
            deactivate_qe_complex_spin_();
            if (mode == rmcs_msgs::ChassisMode::SPIN) {
                mode = rmcs_msgs::ChassisMode::STEP_DOWN;
            } else {
                mode = rmcs_msgs::ChassisMode::SPIN;
                spinning_forward_ = !spinning_forward_;
            }
        } else if (!last_keyboard_.c && keyboard.c) {
            deactivate_complex_spin_();
            deactivate_qe_complex_spin_();
            if (mode == rmcs_msgs::ChassisMode::SPIN) {
                mode = rmcs_msgs::ChassisMode::AUTO;
            } else {
                mode = rmcs_msgs::ChassisMode::SPIN;
                spinning_forward_ = !spinning_forward_;
            }
        } else if (launch_ramp_shortcut_enabled_ && !last_keyboard_.x && keyboard.x) {
            deactivate_complex_spin_();
            deactivate_qe_complex_spin_();
            mode = mode == rmcs_msgs::ChassisMode::LAUNCH_RAMP
                     ? rmcs_msgs::ChassisMode::AUTO
                     : rmcs_msgs::ChassisMode::LAUNCH_RAMP;
        } else if (!last_keyboard_.z && keyboard.z) {
            deactivate_complex_spin_();
            deactivate_qe_complex_spin_();
            mode = mode == rmcs_msgs::ChassisMode::STEP_DOWN ? rmcs_msgs::ChassisMode::AUTO
                                                             : rmcs_msgs::ChassisMode::STEP_DOWN;
        }

        if (complex_spin_active_ || qe_complex_spin_active_)
            mode = rmcs_msgs::ChassisMode::SPIN;

        *mode_ = mode;
    }

    void activate_complex_spin_(rmcs_msgs::ChassisMode& mode) {
        complex_spin_active_ = true;
        complex_spin_elapsed_ = 0.0;
        apply_symmetric_target = true;
        if (mode != rmcs_msgs::ChassisMode::SPIN) {
            mode = rmcs_msgs::ChassisMode::SPIN;
            spinning_forward_ = !spinning_forward_;
        }
    }

    void deactivate_complex_spin_() {
        complex_spin_active_ = false;
        complex_spin_elapsed_ = 0.0;
    }

    void activate_qe_complex_spin_(rmcs_msgs::ChassisMode& mode) {
        qe_complex_spin_active_ = true;
        qe_last_toggle_elapsed_ = 0.0;
        qe_front_high_rear_low_ = true;
        apply_front_high_rear_low_target_();
        if (mode != rmcs_msgs::ChassisMode::SPIN) {
            mode = rmcs_msgs::ChassisMode::SPIN;
            spinning_forward_ = !spinning_forward_;
        }
    }

    void deactivate_qe_complex_spin_() {
        qe_complex_spin_active_ = false;
        qe_last_toggle_elapsed_ = 0.0;
    }

    void apply_front_high_rear_low_target_() {
        lf_current_target_angle_ = max_angle_;
        rf_current_target_angle_ = max_angle_;
        lb_current_target_angle_ = min_angle_;
        rb_current_target_angle_ = min_angle_;
        apply_symmetric_target = false;
        qe_front_high_rear_low_ = true;
    }

    void apply_front_low_rear_high_target_() {
        lf_current_target_angle_ = min_angle_;
        rf_current_target_angle_ = min_angle_;
        lb_current_target_angle_ = max_angle_;
        rb_current_target_angle_ = max_angle_;
        apply_symmetric_target = false;
        qe_front_high_rear_low_ = false;
    }

    void toggle_bg_target_() {
        if (qe_front_high_rear_low_) {
            apply_front_low_rear_high_target_();
        } else {
            apply_front_high_rear_low_target_();
        }
    }

    void toggle_qe_complex_spin_target_() {
        toggle_bg_target_();
    }

    void update_qe_complex_spin_toggle_() {
        constexpr double qe_complex_spin_toggle_period = 1.0;

        qe_last_toggle_elapsed_ += update_dt();
        size_t qe_complex_spin_toggle_count = 0;
        while (qe_last_toggle_elapsed_ >= qe_complex_spin_toggle_period) {
            qe_last_toggle_elapsed_ -= qe_complex_spin_toggle_period;
            ++qe_complex_spin_toggle_count;
        }

        if ((qe_complex_spin_toggle_count % 2) == 1)
            toggle_qe_complex_spin_target_();
    }

    std::array<double, kJointCount> read_current_joint_physical_angles_() const {
        std::array<double, kJointCount> current_physical_angles{};
        current_physical_angles.fill(nan_);
        for (size_t i = 0; i < kJointCount; ++i) {
            if (joint_physical_angle_[i].ready() && std::isfinite(*joint_physical_angle_[i]))
                current_physical_angles[i] = *joint_physical_angle_[i];
        }
        return current_physical_angles;
    }

    bool prone_override_requested_by_keyboard() const { return keyboard_.ready() && keyboard_->ctrl; }

    bool suspension_toggle_requested_by_switch_(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right) const {
        return switch_left == rmcs_msgs::Switch::DOWN && switch_right == rmcs_msgs::Switch::UP
            && last_switch_right_ == rmcs_msgs::Switch::MIDDLE;
    }

    void update_suspension_toggle_from_inputs_(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right) {
        if (suspension_toggle_requested_by_switch_(switch_left, switch_right)) {
            suspension_on_by_switch = !suspension_on_by_switch;
        }
    }

    bool ctrl_hold_requested_by_input_() const {
        return prone_override_requested_by_keyboard() || suspension_on_by_switch;
    }

    bool suspension_requested_by_input_() const {
        return active_suspension_enable_ && ctrl_hold_requested_by_input_();
    }

    bool ensure_joint_target_states_from_feedback(
        const std::array<double, kJointCount>& current_physical_angles) {
        bool any_active = false;
        for (size_t i = 0; i < kJointCount; ++i) {
            if (std::isfinite(current_physical_angles[i]) && !joint_target_active_[i]) {
                joint_target_physical_angle_state_rad_[i] = current_physical_angles[i];
                joint_target_physical_velocity_state_rad_[i] = 0.0;
                joint_target_physical_acceleration_state_rad_[i] = 0.0;
                current_target_physical_angles_rad_[i] = current_physical_angles[i];
                joint_target_active_[i] = true;
            }

            any_active = any_active || joint_target_active_[i];
        }

        return any_active;
    }

    bool any_joint_target_active_() const {
        return std::any_of(
            joint_target_active_.begin(), joint_target_active_.end(), [](bool active) {
                return active;
            });
    }

    void reset_all_controls() {
        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        *ctrl_hold_active_ = false;

        chassis_control_velocity_->vector << nan_, nan_, nan_;
        *chassis_angle_ = nan_;
        *chassis_control_angle_ = nan_;

        current_target_angle_ = max_angle_;
        lf_current_target_angle_ = current_target_angle_;
        lb_current_target_angle_ = current_target_angle_;
        rb_current_target_angle_ = current_target_angle_;
        rf_current_target_angle_ = current_target_angle_;
        joint_target_active_.fill(false);
        current_target_physical_angles_rad_.fill(nan_);
        joint_target_physical_angle_state_rad_.fill(nan_);
        joint_target_physical_velocity_state_rad_.fill(0.0);
        joint_target_physical_acceleration_state_rad_.fill(0.0);
        suspension_on_by_switch = false;
        deactivate_complex_spin_();
        deactivate_qe_complex_spin_();

        for (size_t i = 0; i < kJointCount; ++i) {
            *joint_target_physical_angle_[i] = nan_;
            *joint_target_physical_velocity_[i] = nan_;
            *joint_target_physical_acceleration_[i] = nan_;
            *joint_angle_error_[i] = nan_;
        }

    }

    void update_velocity_control() {
        const Eigen::Vector2d translational_velocity = update_translational_velocity_control();
        const double angular_velocity = update_angular_velocity_control();
        chassis_control_velocity_->vector << translational_velocity, angular_velocity;
    }

    double update_dt() const {
        if (update_rate_.ready() && std::isfinite(*update_rate_) && *update_rate_ > 1e-6)
            return 1.0 / *update_rate_;
        return default_dt_;
    }

    Eigen::Vector2d update_translational_velocity_control() {
        const auto keyboard = *keyboard_;
        const Eigen::Vector2d keyboard_move{keyboard.w - keyboard.s, keyboard.a - keyboard.d};

        Eigen::Vector2d translational_velocity =
            Eigen::Rotation2Dd{*gimbal_yaw_angle_} * ((*joystick_right_) + keyboard_move);

        if (translational_velocity.norm() > 1.0)
            translational_velocity.normalize();

        translational_velocity *= translational_velocity_max_;
        return translational_velocity;
    }

    double update_angular_velocity_control() {
        if (*ctrl_hold_active_ && std::isfinite(*manual_yaw_velocity_override_)) {
            *chassis_angle_ = 2 * std::numbers::pi - *gimbal_yaw_angle_;
            *chassis_control_angle_ = nan_;
            return std::clamp(
                *manual_yaw_velocity_override_, -angular_velocity_max_, angular_velocity_max_);
        }

        double angular_velocity = 0.0;
        double chassis_control_angle = nan_;

        switch (*mode_) {
        case rmcs_msgs::ChassisMode::AUTO: break;

        case rmcs_msgs::ChassisMode::SPIN: {
            angular_velocity =
                spin_ratio_ * (spinning_forward_ ? angular_velocity_max_ : -angular_velocity_max_);
            angular_velocity =
                std::clamp(angular_velocity, -angular_velocity_max_, angular_velocity_max_);
        } break;

        case rmcs_msgs::ChassisMode::STEP_DOWN: {
            double err = calculate_unsigned_chassis_angle_error(chassis_control_angle);

            // In step-down mode, front/back can both be used for alignment.
            constexpr double alignment = std::numbers::pi;
            while (err > alignment / 2) {
                chassis_control_angle -= alignment;
                if (chassis_control_angle < 0)
                    chassis_control_angle += 2 * std::numbers::pi;
                err -= alignment;
            }

            angular_velocity = following_velocity_controller_.update(err);
        } break;

        case rmcs_msgs::ChassisMode::LAUNCH_RAMP: {
            double err = calculate_unsigned_chassis_angle_error(chassis_control_angle);

            constexpr double alignment = 2 * std::numbers::pi;
            if (err > alignment / 2)
                err -= alignment;

            angular_velocity = following_velocity_controller_.update(err);
        } break;

        default: break;
        }

        *chassis_angle_ = 2 * std::numbers::pi - *gimbal_yaw_angle_;
        *chassis_control_angle_ = chassis_control_angle;

        return angular_velocity;
    }

    double calculate_unsigned_chassis_angle_error(double& chassis_control_angle) {
        chassis_control_angle = *gimbal_yaw_angle_error_;
        if (chassis_control_angle < 0)
            chassis_control_angle += 2 * std::numbers::pi;

        double err = chassis_control_angle + *gimbal_yaw_angle_;
        if (err >= 2 * std::numbers::pi)
            err -= 2 * std::numbers::pi;

        return err;
    }

    void update_lift_target_toggle(rmcs_msgs::Keyboard keyboard) {
        constexpr double rotary_knob_symmetric_edge_threshold = 0.7;
        constexpr double rotary_knob_bg_edge_threshold = -0.9;
        constexpr double complex_spin_toggle_period = 0.5;

        const bool keyboard_toggle_condition =
            !qe_complex_spin_active_ && !last_keyboard_.q && keyboard.q && !keyboard.e;
        const bool rotary_knob_toggle_condition =
            last_rotary_knob_ < rotary_knob_symmetric_edge_threshold
            && *rotary_knob_ >= rotary_knob_symmetric_edge_threshold;
        const bool rotary_knob_bg_toggle_condition =
            !qe_complex_spin_active_ && last_rotary_knob_ > rotary_knob_bg_edge_threshold
            && *rotary_knob_ <= rotary_knob_bg_edge_threshold;
        const bool front_high_rear_low = !qe_complex_spin_active_ && !last_keyboard_.b && keyboard.b;
        const bool front_low_rear_high = !qe_complex_spin_active_ && !last_keyboard_.g && keyboard.g;
        bool complex_spin_toggle_condition = false;

        if (complex_spin_active_) {
            complex_spin_elapsed_ += update_dt();
            size_t complex_spin_toggle_count = 0;
            while (complex_spin_elapsed_ >= complex_spin_toggle_period) {
                complex_spin_elapsed_ -= complex_spin_toggle_period;
                ++complex_spin_toggle_count;
            }
            complex_spin_toggle_condition = (complex_spin_toggle_count % 2) == 1;
        }

        if (qe_complex_spin_active_)
            update_qe_complex_spin_toggle_();

        if (apply_symmetric_target) {
            lf_current_target_angle_ = current_target_angle_;
            lb_current_target_angle_ = current_target_angle_;
            rb_current_target_angle_ = current_target_angle_;
            rf_current_target_angle_ = current_target_angle_;
        }

        if (rotary_knob_toggle_condition || keyboard_toggle_condition || complex_spin_toggle_condition) {
            current_target_angle_ =
                (std::abs(current_target_angle_ - max_angle_) < 1e-6) ? min_angle_ : max_angle_;
            apply_symmetric_target = true;
        } else if (rotary_knob_bg_toggle_condition) {
            toggle_bg_target_();
        } else if (front_high_rear_low) {
            apply_front_high_rear_low_target_();
        } else if (front_low_rear_high) {
            apply_front_low_rear_high_target_();
        }

        last_rotary_knob_ = *rotary_knob_;
    }

    // Chassis owns the high-level joint intent pipeline: read feedback, generate deploy targets,
    // coordinate suspension overrides, then publish the resulting joint intent for the servo layer.
    void run_joint_intent_pipeline_() {
        const auto current_physical_angles = read_current_joint_physical_angles_();

        if (!ensure_joint_target_states_from_feedback(current_physical_angles)) {
            publish_nan_joint_targets();
            return;
        }

        current_target_physical_angles_rad_[kLeftFront] = deg_to_rad(lf_current_target_angle_);
        current_target_physical_angles_rad_[kLeftBack] = deg_to_rad(lb_current_target_angle_);
        current_target_physical_angles_rad_[kRightBack] = deg_to_rad(rb_current_target_angle_);
        current_target_physical_angles_rad_[kRightFront] = deg_to_rad(rf_current_target_angle_);

        update_joint_target_trajectory();
        publish_joint_target_angles(current_physical_angles);
    }

    static double deg_to_rad(double deg) { return deg * std::numbers::pi / 180.0; }

    void update_joint_target_trajectory() {
        const double dt = update_dt();
        for (size_t i = 0; i < kJointCount; ++i) {
            if (!joint_target_active_[i])
                continue;

            double& angle_state = joint_target_physical_angle_state_rad_[i];
            double& velocity_state = joint_target_physical_velocity_state_rad_[i];
            double& acceleration_state = joint_target_physical_acceleration_state_rad_[i];
            const double target_angle = current_target_physical_angles_rad_[i];

            if (!std::isfinite(target_angle) || !std::isfinite(angle_state)) {
                continue;
            }

            const double position_error = target_angle - angle_state;
            const double stopping_distance =
                velocity_state * velocity_state / (2.0 * target_physical_acceleration_limit_);

            double desired_velocity = 0.0;
            if (std::abs(position_error) > 1e-6 && std::abs(position_error) > stopping_distance) {
                desired_velocity =
                    std::copysign(target_physical_velocity_limit_, position_error);
            }

            const double velocity_error = desired_velocity - velocity_state;
            acceleration_state =
                std::clamp(
                    velocity_error / dt, -target_physical_acceleration_limit_,
                    target_physical_acceleration_limit_);

            velocity_state += acceleration_state * dt;
            velocity_state =
                std::clamp(
                    velocity_state, -target_physical_velocity_limit_,
                    target_physical_velocity_limit_);
            angle_state += velocity_state * dt;

            const double next_error = target_angle - angle_state;
            if ((position_error > 0.0 && next_error < 0.0)
                || (position_error < 0.0 && next_error > 0.0)
                || (std::abs(next_error) < 1e-5 && std::abs(velocity_state) < 1e-3)) {
                angle_state = target_angle;
                velocity_state = 0.0;
                acceleration_state = 0.0;
            }
        }
    }

    void publish_joint_target_angles(
        const std::array<double, kJointCount>& current_physical_angles) {
        if (!any_joint_target_active_()) {
            publish_nan_joint_targets();
            return;
        }

        for (size_t i = 0; i < kJointCount; ++i) {
            if (!joint_target_active_[i]) {
                *joint_target_physical_angle_[i] = nan_;
                *joint_target_physical_velocity_[i] = nan_;
                *joint_target_physical_acceleration_[i] = nan_;
                *joint_angle_error_[i] = nan_;
                continue;
            }

            *joint_target_physical_angle_[i] = joint_target_physical_angle_state_rad_[i];
            *joint_target_physical_velocity_[i] = joint_target_physical_velocity_state_rad_[i];
            *joint_target_physical_acceleration_[i] = joint_target_physical_acceleration_state_rad_[i];
            *joint_angle_error_[i] = std::isfinite(current_physical_angles[i])
                                        ? current_physical_angles[i]
                                              - joint_target_physical_angle_state_rad_[i]
                                        : nan_;
        }
    }

    void publish_nan_joint_targets() {
        joint_target_active_.fill(false);

        for (size_t i = 0; i < kJointCount; ++i) {
            *joint_target_physical_angle_[i] = nan_;
            *joint_target_physical_velocity_[i] = nan_;
            *joint_target_physical_acceleration_[i] = nan_;
            *joint_angle_error_[i] = nan_;
        }
    }

private:
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<double> rotary_knob_;
    InputInterface<double> update_rate_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();
    double last_rotary_knob_ = 0.0;

    InputInterface<double> gimbal_yaw_angle_, gimbal_yaw_angle_error_;
    InputInterface<double> manual_yaw_velocity_override_;
    OutputInterface<double> chassis_angle_, chassis_control_angle_;

    OutputInterface<rmcs_msgs::ChassisMode> mode_;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;
    OutputInterface<bool> ctrl_hold_active_;

    bool spinning_forward_ = true;
    bool apply_symmetric_target = true;
    bool complex_spin_active_ = false;
    double complex_spin_elapsed_ = 0.0;
    bool qe_complex_spin_active_ = false;
    bool qe_front_high_rear_low_ = true;
    double qe_last_toggle_elapsed_ = 0.0;
    pid::PidCalculator following_velocity_controller_;
    const double spin_ratio_;
    const bool launch_ramp_shortcut_enabled_;

    static constexpr std::array<const char*, kJointCount> kJointNames = {
        "left_front", "left_back", "right_back", "right_front"};
    std::array<InputInterface<double>, kJointCount> joint_physical_angle_;

    std::array<OutputInterface<double>, kJointCount> joint_angle_error_;

    std::array<OutputInterface<double>, kJointCount> joint_target_physical_angle_;
    std::array<OutputInterface<double>, kJointCount> joint_target_physical_velocity_;
    std::array<OutputInterface<double>, kJointCount> joint_target_physical_acceleration_;

    double min_angle_;
    double max_angle_;

    double current_target_angle_;
    double lf_current_target_angle_, lb_current_target_angle_, rb_current_target_angle_,
        rf_current_target_angle_;

    std::array<double, kJointCount> current_target_physical_angles_rad_   = {0.0, 0.0, 0.0, 0.0};

    std::array<bool, kJointCount> joint_target_active_ = {false, false, false, false};
    std::array<double, kJointCount> joint_target_physical_angle_state_rad_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> joint_target_physical_velocity_state_rad_ = {
        0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> joint_target_physical_acceleration_state_rad_ = {
        0.0, 0.0, 0.0, 0.0};

    double target_physical_velocity_limit_;
    double target_physical_acceleration_limit_;
    bool active_suspension_enable_;
    bool suspension_on_by_switch = false;
    static constexpr double default_dt_ = 1e-3;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::DeformableChassis, rmcs_executor::Component)
