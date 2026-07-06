#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

#include <eigen3/Eigen/Dense>
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

        , min_angle_(get_parameter_or("min_angle", 7.0))
        , max_angle_(get_parameter_or("max_angle", 58.0))
        , active_suspension_base_angle_deg_(std::clamp(
              get_parameter_or("active_suspension_base_angle", max_angle_), min_angle_ - 5.0,
              max_angle_))
        , active_suspension_base_angle_scroll_step_deg_(std::max(
              std::abs(get_parameter_or("active_suspension_base_angle_scroll_step_deg", 5.0)),
              0.0))
        , target_physical_velocity_limit_(
              std::max(
                  deg_to_rad(std::abs(get_parameter_or("target_physical_velocity_limit", 180.0))),
                  1e-6))
        , target_physical_acceleration_limit_(
              std::max(
                  deg_to_rad(
                      std::abs(get_parameter_or("target_physical_acceleration_limit", 720.0))),
                  1e-6))
        , active_suspension_enable_(get_parameter_or("active_suspension_enable", false))
        , suspension_velocity_limit_(
              std::max(
                  deg_to_rad(
                      std::abs(get_parameter_or(
                          "active_suspension_target_velocity_limit_deg",
                          get_parameter_or("target_physical_velocity_limit", 180.0)))),
                  1e-6))
        , suspension_acceleration_limit_(
              std::max(
                  deg_to_rad(
                      std::abs(get_parameter_or(
                          "active_suspension_target_acceleration_limit_deg",
                          get_parameter_or("target_physical_acceleration_limit", 720.0)))),
                  1e-6))
        , suspension_correction_velocity_limit_(
              std::max(
                  deg_to_rad(std::abs(get_parameter_or(
                      "active_suspension_correction_velocity_limit_deg", 720.0))),
                  1e-6))
        , suspension_correction_acceleration_limit_(
              std::max(
                  deg_to_rad(std::abs(get_parameter_or(
                      "active_suspension_correction_acceleration_limit_deg", 3600.0))),
                  1e-6))
        , chassis_imu_calibration_wait_time_(
              std::max(get_parameter_or("chassis_imu_calibration_wait_s", 2.0), 0.0))
        , chassis_imu_calibration_sample_time_(
              std::max(get_parameter_or("chassis_imu_calibration_sample_s", 3.0), 1e-6)) {

        following_velocity_controller_.output_max = angular_velocity_max_;
        following_velocity_controller_.output_min = -angular_velocity_max_;
        load_active_suspension_pid_(
            "active_suspension_pitch_outer_", pitch_outer_pid_, 8.0, 0.35, 0.28,
            -2.0, 2.0, -3.0, 3.0);
        load_active_suspension_pid_(
            "active_suspension_pitch_inner_", pitch_inner_pid_, 2.0, 0.0, 0.0,
            -1.0, 1.0, -0.785, 0.785);
        load_active_suspension_pid_(
            "active_suspension_roll_outer_", roll_outer_pid_, 8.0, 0.35, 0.28,
            -2.0, 2.0, -3.0, 3.0);
        load_active_suspension_pid_(
            "active_suspension_roll_inner_", roll_inner_pid_, 2.0, 0.0, 0.0,
            -1.0, 1.0, -0.785, 0.785);

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/mouse/mouse_wheel", mouse_wheel_);
        register_input("/remote/rotary_knob", rotary_knob_);
        register_input("/predefined/update_rate", update_rate_);

        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_, false);
        register_input("/gimbal/yaw/control_angle_error", gimbal_yaw_angle_error_, false);
        register_input("/chassis/manual_yaw_velocity_override", manual_yaw_velocity_override_, false);

        register_input(
            "/chassis/left_front_joint/physical_angle", left_front_joint_physical_angle_, false);
        register_input(
            "/chassis/left_back_joint/physical_angle", left_back_joint_physical_angle_, false);
        register_input(
            "/chassis/right_front_joint/physical_angle", right_front_joint_physical_angle_, false);
        register_input(
            "/chassis/right_back_joint/physical_angle", right_back_joint_physical_angle_, false);
        register_input("/chassis/imu/pitch", chassis_imu_pitch_, false);
        register_input("/chassis/imu/roll", chassis_imu_roll_, false);
        register_input("/chassis/imu/pitch_rate", chassis_imu_pitch_rate_, false);
        register_input("/chassis/imu/roll_rate", chassis_imu_roll_rate_, false);

        register_output("/gimbal/scope/control_torque", scope_motor_control_torque, nan_);

        register_output("/chassis/angle", chassis_angle_, nan_);
        register_output("/chassis/control_angle", chassis_control_angle_, nan_);

        register_output("/chassis/control_mode", mode_);
        register_output("/chassis/control_velocity", chassis_control_velocity_);
        register_output("/chassis/ctrl_hold_active", ctrl_hold_active_, false);
        register_output("/chassis/active_suspension/active", active_suspension_active_, false);

        register_output("/chassis/left_front_joint/control_angle_error", lf_angle_error_, nan_);
        register_output("/chassis/left_back_joint/control_angle_error", lb_angle_error_, nan_);
        register_output("/chassis/right_front_joint/control_angle_error", rf_angle_error_, nan_);
        register_output("/chassis/right_back_joint/control_angle_error", rb_angle_error_, nan_);

        register_output(
            "/chassis/left_front_joint/target_physical_angle",
            left_front_joint_target_physical_angle_, nan_);
        register_output(
            "/chassis/left_back_joint/target_physical_angle",
            left_back_joint_target_physical_angle_, nan_);
        register_output(
            "/chassis/right_back_joint/target_physical_angle",
            right_back_joint_target_physical_angle_, nan_);
        register_output(
            "/chassis/right_front_joint/target_physical_angle",
            right_front_joint_target_physical_angle_, nan_);
        register_output(
            "/chassis/left_front_joint/target_physical_velocity",
            left_front_joint_target_physical_velocity_, nan_);
        register_output(
            "/chassis/left_back_joint/target_physical_velocity",
            left_back_joint_target_physical_velocity_, nan_);
        register_output(
            "/chassis/right_back_joint/target_physical_velocity",
            right_back_joint_target_physical_velocity_, nan_);
        register_output(
            "/chassis/right_front_joint/target_physical_velocity",
            right_front_joint_target_physical_velocity_, nan_);
        register_output(
            "/chassis/left_front_joint/target_physical_acceleration",
            left_front_joint_target_physical_acceleration_, nan_);
        register_output(
            "/chassis/left_back_joint/target_physical_acceleration",
            left_back_joint_target_physical_acceleration_, nan_);
        register_output(
            "/chassis/right_back_joint/target_physical_acceleration",
            right_back_joint_target_physical_acceleration_, nan_);
        register_output(
            "/chassis/right_front_joint/target_physical_acceleration",
            right_front_joint_target_physical_acceleration_, nan_);

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
        if (!chassis_imu_pitch_.ready())
            chassis_imu_pitch_.make_and_bind_directly(0.0);
        if (!chassis_imu_roll_.ready())
            chassis_imu_roll_.make_and_bind_directly(0.0);
        if (!chassis_imu_pitch_rate_.ready())
            chassis_imu_pitch_rate_.make_and_bind_directly(0.0);
        if (!chassis_imu_roll_rate_.ready())
            chassis_imu_roll_rate_.make_and_bind_directly(0.0);
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
            update_suspension_toggle_from_inputs_(switch_left, switch_right, keyboard);
            *ctrl_hold_active_ = ctrl_hold_requested_by_input_();
            *active_suspension_active_ = suspension_requested_by_input_();
            update_velocity_control();
            update_lift_target_toggle(keyboard);
            update_active_suspension_base_angle_from_mouse_wheel_();
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
    static constexpr double rad_to_deg_ = 180.0 / std::numbers::pi;
    static constexpr double imu_calibration_offset_limit_rad_ =
        1.0 * std::numbers::pi / 180.0;

    void load_active_suspension_pid_(
        const std::string& prefix, pid::PidCalculator& pid, double kp_default, double ki_default,
        double kd_default, double integral_min_default, double integral_max_default,
        double output_min_default, double output_max_default) {
        pid.kp = get_parameter_or(prefix + "kp", kp_default);
        pid.ki = get_parameter_or(prefix + "ki", ki_default);
        pid.kd = get_parameter_or(prefix + "kd", kd_default);
        pid.integral_min = get_parameter_or(prefix + "integral_min", integral_min_default);
        pid.integral_max = get_parameter_or(prefix + "integral_max", integral_max_default);
        pid.output_min = get_parameter_or(prefix + "output_min", output_min_default);
        pid.output_max = get_parameter_or(prefix + "output_max", output_max_default);
    }

    void validate_joint_feedback_inputs() const {
        if (left_front_joint_physical_angle_.ready() && left_back_joint_physical_angle_.ready()
            && right_front_joint_physical_angle_.ready() && right_back_joint_physical_angle_.ready())
            return;

        throw std::runtime_error(
            "missing deformable chassis feedback interfaces: expected /chassis/*_joint/physical_angle");
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
        const std::array<const InputInterface<double>*, kJointCount> physical_angle_inputs{
            &left_front_joint_physical_angle_, &left_back_joint_physical_angle_,
            &right_back_joint_physical_angle_, &right_front_joint_physical_angle_};

        std::array<double, kJointCount> current_physical_angles{};
        current_physical_angles.fill(nan_);
        for (size_t i = 0; i < kJointCount; ++i) {
            if (physical_angle_inputs[i]->ready() && std::isfinite(*(*physical_angle_inputs[i]))) {
                current_physical_angles[i] = *(*physical_angle_inputs[i]);
            }
        }

        return current_physical_angles;
    }

    bool suspension_toggle_requested_by_keyboard_(const rmcs_msgs::Keyboard& keyboard) const {
        return !last_keyboard_.e && keyboard.e && !keyboard.q;
    }

    bool suspension_toggle_requested_by_switch_(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right) const {
        return switch_left == rmcs_msgs::Switch::DOWN && switch_right == rmcs_msgs::Switch::UP
            && last_switch_right_ == rmcs_msgs::Switch::MIDDLE;
    }

    void update_suspension_toggle_from_inputs_(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right,
        const rmcs_msgs::Keyboard& keyboard) {
        if (suspension_toggle_requested_by_switch_(switch_left, switch_right)) {
            active_suspension_on_by_remote_ = !active_suspension_on_by_remote_;
        }
        if (suspension_toggle_requested_by_keyboard_(keyboard)) {
            active_suspension_on_by_keyboard_ = !active_suspension_on_by_keyboard_;
        }
    }

    bool prone_override_requested_by_keyboard() const {
        return keyboard_.ready() && keyboard_->ctrl;
    }

    bool ctrl_hold_requested_by_input_() const {
        return prone_override_requested_by_keyboard() || active_suspension_on_by_remote_
            || active_suspension_on_by_keyboard_;
    }

    bool low_prone_override_active_() const {
        return prone_override_requested_by_keyboard();
    }

    bool suspension_requested_by_input_() const {
        return active_suspension_enable_ && ctrl_hold_requested_by_input_();
    }

    bool symmetric_joint_target_requested_() const {
        constexpr double epsilon = 1e-6;
        return std::abs(lf_current_target_angle_ - lb_current_target_angle_) <= epsilon
            && std::abs(lf_current_target_angle_ - rb_current_target_angle_) <= epsilon
            && std::abs(lf_current_target_angle_ - rf_current_target_angle_) <= epsilon;
    }

    void reset_attitude_correction_state_() {
        pitch_outer_pid_.reset();
        pitch_inner_pid_.reset();
        roll_outer_pid_.reset();
        roll_inner_pid_.reset();
        joint_suspension_active_.fill(false);
        suspension_correction_target_rad_.fill(0.0);
    }

    void reset_active_suspension_correction_state_() {
        suspension_correction_target_rad_.fill(0.0);
        suspension_correction_state_rad_.fill(0.0);
        suspension_correction_velocity_state_rad_.fill(0.0);
        suspension_correction_acceleration_state_rad_.fill(0.0);
    }

    void reset_chassis_imu_calibration_window_() {
        chassis_imu_calibration_hold_elapsed_ = 0.0;
        chassis_imu_calibration_sample_count_ = 0;
        chassis_imu_pitch_sum_ = 0.0;
        chassis_imu_roll_sum_ = 0.0;
        chassis_imu_calibration_completed_for_window_ = false;
    }

    void update_chassis_imu_calibration_() {
        if (chassis_imu_calibrated_once_)
            return;

        if (!symmetric_joint_target_requested_()) {
            reset_chassis_imu_calibration_window_();
            return;
        }

        const double raw_pitch = *chassis_imu_pitch_;
        const double raw_roll = *chassis_imu_roll_;
        if (!std::isfinite(raw_pitch) || !std::isfinite(raw_roll))
            return;

        chassis_imu_calibration_hold_elapsed_ += update_dt();
        if (chassis_imu_calibration_hold_elapsed_ < chassis_imu_calibration_wait_time_)
            return;

        const double calibration_end_time =
            chassis_imu_calibration_wait_time_ + chassis_imu_calibration_sample_time_;
        if (chassis_imu_calibration_hold_elapsed_ < calibration_end_time) {
            chassis_imu_pitch_sum_ += raw_pitch;
            chassis_imu_roll_sum_ += raw_roll;
            ++chassis_imu_calibration_sample_count_;
            return;
        }

        if (chassis_imu_calibration_completed_for_window_)
            return;

        chassis_imu_calibration_completed_for_window_ = true;
        if (chassis_imu_calibration_sample_count_ == 0) {
            RCLCPP_WARN(
                get_logger(),
                "[chassis imu calibration] skipped because no valid samples were collected");
            return;
        }

        chassis_imu_pitch_offset_ = std::clamp(
            chassis_imu_pitch_sum_ / static_cast<double>(chassis_imu_calibration_sample_count_),
            -imu_calibration_offset_limit_rad_, imu_calibration_offset_limit_rad_);
        chassis_imu_roll_offset_ = std::clamp(
            chassis_imu_roll_sum_ / static_cast<double>(chassis_imu_calibration_sample_count_),
            -imu_calibration_offset_limit_rad_, imu_calibration_offset_limit_rad_);
        chassis_imu_calibrated_once_ = true;
        RCLCPP_INFO(
            get_logger(),
            "[chassis imu calibration] pitch_offset=% .3f deg roll_offset=% .3f deg "
            "(samples=%zu)",
            chassis_imu_pitch_offset_ * rad_to_deg_, chassis_imu_roll_offset_ * rad_to_deg_,
            chassis_imu_calibration_sample_count_);
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

    double active_suspension_min_angle_deg_() const { return min_angle_ - 5.0; }

    double active_suspension_min_angle_rad_() const {
        return deg_to_rad(active_suspension_min_angle_deg_());
    }

    double active_suspension_base_angle_rad_() const {
        return deg_to_rad(active_suspension_base_angle_deg_);
    }

    bool active_suspension_correction_inverted_() const {
        const double midpoint = (active_suspension_min_angle_deg_() + max_angle_) / 2.0;
        return active_suspension_base_angle_deg_ > midpoint;
    }

    void update_active_suspension_base_angle_from_mouse_wheel_() {
        if (!suspension_requested_by_input_() || low_prone_override_active_() || !mouse_wheel_.ready())
            return;

        const double mouse_wheel = *mouse_wheel_;
        if (!std::isfinite(mouse_wheel) || std::abs(mouse_wheel) <= 1e-9)
            return;

        active_suspension_base_angle_deg_ = std::clamp(
            active_suspension_base_angle_deg_
                - mouse_wheel * active_suspension_base_angle_scroll_step_deg_,
            active_suspension_min_angle_deg_(), max_angle_);
    }

    void update_active_suspension_() {
        if (!suspension_requested_by_input_()) {
            reset_attitude_correction_state_();
            return;
        }

        constexpr double max_attitude = 30.0 * std::numbers::pi / 180.0;
        const double corrected_pitch =
            std::clamp(*chassis_imu_pitch_ - chassis_imu_pitch_offset_, -max_attitude, max_attitude);
        const double corrected_roll =
            std::clamp(*chassis_imu_roll_ - chassis_imu_roll_offset_, -max_attitude, max_attitude);
        const double corrected_pitch_rate = *chassis_imu_pitch_rate_;
        const double corrected_roll_rate = *chassis_imu_roll_rate_;

        const double pitch_outer = pitch_outer_pid_.update(-corrected_pitch);
        const double roll_outer = roll_outer_pid_.update(corrected_roll);
        const double pitch_angle_diff = pitch_inner_pid_.update(pitch_outer - corrected_pitch_rate);
        const double roll_angle_diff = roll_inner_pid_.update(roll_outer + corrected_roll_rate);
        if (!std::isfinite(pitch_angle_diff) || !std::isfinite(roll_angle_diff)) {
            reset_attitude_correction_state_();
            return;
        }

        if (active_suspension_correction_inverted_()) {
            const double front_pitch_drop = std::max(pitch_angle_diff, 0.0);
            const double back_pitch_drop = std::max(-pitch_angle_diff, 0.0);
            const double left_roll_drop = std::max(-roll_angle_diff, 0.0);
            const double right_roll_drop = std::max(roll_angle_diff, 0.0);

            suspension_correction_target_rad_[kLeftFront] =
                -(front_pitch_drop + left_roll_drop);
            suspension_correction_target_rad_[kLeftBack] = -(back_pitch_drop + left_roll_drop);
            suspension_correction_target_rad_[kRightBack] = -(back_pitch_drop + right_roll_drop);
            suspension_correction_target_rad_[kRightFront] =
                -(front_pitch_drop + right_roll_drop);
        } else {
            const double front_pitch_add = std::max(-pitch_angle_diff, 0.0);
            const double back_pitch_add = std::max(pitch_angle_diff, 0.0);
            const double left_roll_add = std::max(roll_angle_diff, 0.0);
            const double right_roll_add = std::max(-roll_angle_diff, 0.0);

            suspension_correction_target_rad_[kLeftFront] = front_pitch_add + left_roll_add;
            suspension_correction_target_rad_[kLeftBack] = back_pitch_add + left_roll_add;
            suspension_correction_target_rad_[kRightBack] = back_pitch_add + right_roll_add;
            suspension_correction_target_rad_[kRightFront] = front_pitch_add + right_roll_add;
        }

        joint_suspension_active_.fill(true);
    }

    void update_active_suspension_correction_trajectory() {
        const double dt = update_dt();
        const double max_target_angle = deg_to_rad(max_angle_);
        for (size_t i = 0; i < kJointCount; ++i) {
            if (!joint_suspension_active_[i]) {
                suspension_correction_target_rad_[i] = 0.0;
            }

        const double base_angle = std::isfinite(joint_target_physical_angle_state_rad_[i])
                                      ? joint_target_physical_angle_state_rad_[i]
                                      : (low_prone_override_active_() ? active_suspension_min_angle_rad_()
                                                                      : active_suspension_base_angle_rad_());
            const double correction_min = active_suspension_min_angle_rad_() - base_angle;
            const double correction_max = max_target_angle - base_angle;
            const double target = std::clamp(
                suspension_correction_target_rad_[i], correction_min, correction_max);

            double& angle_state = suspension_correction_state_rad_[i];
            double& velocity_state = suspension_correction_velocity_state_rad_[i];
            double& acceleration_state = suspension_correction_acceleration_state_rad_[i];
            const double position_error = target - angle_state;
            const double stopping_distance =
                velocity_state * velocity_state / (2.0 * suspension_correction_acceleration_limit_);

            double desired_velocity = 0.0;
            if (std::abs(position_error) > 1e-6 && std::abs(position_error) > stopping_distance) {
                desired_velocity = std::copysign(suspension_correction_velocity_limit_, position_error);
            }

            const double velocity_error = desired_velocity - velocity_state;
            acceleration_state = std::clamp(
                velocity_error / dt, -suspension_correction_acceleration_limit_,
                suspension_correction_acceleration_limit_);

            velocity_state += acceleration_state * dt;
            velocity_state = std::clamp(
                velocity_state, -suspension_correction_velocity_limit_,
                suspension_correction_velocity_limit_);
            angle_state += velocity_state * dt;

            const double next_error = target - angle_state;
            if ((position_error > 0.0 && next_error < 0.0)
                || (position_error < 0.0 && next_error > 0.0)
                || (std::abs(next_error) < 1e-5 && std::abs(velocity_state) < 1e-3)) {
                angle_state = target;
                velocity_state = 0.0;
                acceleration_state = 0.0;
            }
        }
    }

    void reset_all_controls() {
        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        *ctrl_hold_active_ = false;
        reset_attitude_correction_state_();
        reset_chassis_imu_calibration_window_();

        chassis_control_velocity_->vector << nan_, nan_, nan_;
        *chassis_angle_ = nan_;
        *chassis_control_angle_ = nan_;

        current_target_angle_ = max_angle_;
        active_suspension_base_angle_deg_ = max_angle_;
        lf_current_target_angle_ = current_target_angle_;
        lb_current_target_angle_ = current_target_angle_;
        rb_current_target_angle_ = current_target_angle_;
        rf_current_target_angle_ = current_target_angle_;
        joint_target_active_.fill(false);
        current_target_physical_angles_rad_.fill(nan_);
        joint_target_physical_angle_state_rad_.fill(nan_);
        joint_target_physical_velocity_state_rad_.fill(0.0);
        joint_target_physical_acceleration_state_rad_.fill(0.0);
        reset_active_suspension_correction_state_();
        active_suspension_on_by_keyboard_ = false;
        active_suspension_on_by_remote_ = false;
        deactivate_complex_spin_();
        deactivate_qe_complex_spin_();

        *scope_motor_control_torque = nan_;

        *lf_angle_error_ = nan_;
        *lb_angle_error_ = nan_;
        *rf_angle_error_ = nan_;
        *rb_angle_error_ = nan_;

        *left_front_joint_target_physical_angle_ = nan_;
        *left_back_joint_target_physical_angle_ = nan_;
        *right_back_joint_target_physical_angle_ = nan_;
        *right_front_joint_target_physical_angle_ = nan_;
        *left_front_joint_target_physical_velocity_ = nan_;
        *left_back_joint_target_physical_velocity_ = nan_;
        *right_back_joint_target_physical_velocity_ = nan_;
        *right_front_joint_target_physical_velocity_ = nan_;
        *left_front_joint_target_physical_acceleration_ = nan_;
        *left_back_joint_target_physical_acceleration_ = nan_;
        *right_back_joint_target_physical_acceleration_ = nan_;
        *right_front_joint_target_physical_acceleration_ = nan_;

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
        const bool suspension_requested = suspension_requested_by_input_();

        if (!ensure_joint_target_states_from_feedback(current_physical_angles)) {
            publish_nan_joint_targets();
            return;
        }

        current_target_physical_angles_rad_[kLeftFront] = deg_to_rad(lf_current_target_angle_);
        current_target_physical_angles_rad_[kLeftBack] = deg_to_rad(lb_current_target_angle_);
        current_target_physical_angles_rad_[kRightBack] = deg_to_rad(rb_current_target_angle_);
        current_target_physical_angles_rad_[kRightFront] = deg_to_rad(rf_current_target_angle_);
        if (suspension_requested) {
            current_target_physical_angles_rad_.fill(
                low_prone_override_active_() ? active_suspension_min_angle_rad_()
                                             : active_suspension_base_angle_rad_());
        }

        update_chassis_imu_calibration_();
        scope_motor_control(suspension_requested);
        update_active_suspension_();

        update_joint_target_trajectory();
        update_active_suspension_correction_trajectory();
        publish_joint_target_angles(current_physical_angles);
    }

    static double deg_to_rad(double deg) { return deg * std::numbers::pi / 180.0; }

    void scope_motor_control(bool suspension_requested = false) {
        const bool prone_target_active = suspension_requested;
        if (prone_target_active && *mode_ != rmcs_msgs::ChassisMode::SPIN) {
            *scope_motor_control_torque = -0.3;
        } else {
            *scope_motor_control_torque = 0.3;
        }
    }

    void update_joint_target_trajectory() {
        const double dt = update_dt();
        for (size_t i = 0; i < kJointCount; ++i) {
            if (!joint_target_active_[i])
                continue;

            double& angle_state = joint_target_physical_angle_state_rad_[i];
            double& velocity_state = joint_target_physical_velocity_state_rad_[i];
            double& acceleration_state = joint_target_physical_acceleration_state_rad_[i];
            const double target_angle = current_target_physical_angles_rad_[i];
            const double velocity_limit = joint_suspension_active_[i]
                                            ? suspension_velocity_limit_
                                            : target_physical_velocity_limit_;
            const double acceleration_limit =
                joint_suspension_active_[i] ? suspension_acceleration_limit_
                                            : target_physical_acceleration_limit_;

            if (!std::isfinite(target_angle) || !std::isfinite(angle_state)) {
                continue;
            }

            const double position_error = target_angle - angle_state;
            const double stopping_distance =
                velocity_state * velocity_state / (2.0 * acceleration_limit);

            double desired_velocity = 0.0;
            if (std::abs(position_error) > 1e-6 && std::abs(position_error) > stopping_distance) {
                desired_velocity = std::copysign(velocity_limit, position_error);
            }

            const double velocity_error = desired_velocity - velocity_state;
            acceleration_state =
                std::clamp(velocity_error / dt, -acceleration_limit, acceleration_limit);

            velocity_state += acceleration_state * dt;
            velocity_state = std::clamp(velocity_state, -velocity_limit, velocity_limit);
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

        const auto publish_joint = [this, &current_physical_angles](
                                       size_t index, OutputInterface<double>& angle_output,
                                       OutputInterface<double>& velocity_output,
                                       OutputInterface<double>& acceleration_output,
                                       OutputInterface<double>& angle_error_output) {
            if (!joint_target_active_[index]) {
                *angle_output = nan_;
                *velocity_output = nan_;
                *acceleration_output = nan_;
                *angle_error_output = nan_;
                return;
            }

            const double target_angle = joint_target_physical_angle_state_rad_[index]
                                      + suspension_correction_state_rad_[index];
            *angle_output = std::clamp(
                target_angle, active_suspension_min_angle_rad_(), deg_to_rad(max_angle_));
            *velocity_output = joint_target_physical_velocity_state_rad_[index]
                             + suspension_correction_velocity_state_rad_[index];
            *acceleration_output = joint_target_physical_acceleration_state_rad_[index]
                                 + suspension_correction_acceleration_state_rad_[index];
            *angle_error_output = std::isfinite(current_physical_angles[index])
                                    ? current_physical_angles[index]
                                          - *angle_output
                                    : nan_;
        };

        publish_joint(
            kLeftFront, left_front_joint_target_physical_angle_,
            left_front_joint_target_physical_velocity_,
            left_front_joint_target_physical_acceleration_, lf_angle_error_);
        publish_joint(
            kLeftBack, left_back_joint_target_physical_angle_,
            left_back_joint_target_physical_velocity_,
            left_back_joint_target_physical_acceleration_, lb_angle_error_);
        publish_joint(
            kRightBack, right_back_joint_target_physical_angle_,
            right_back_joint_target_physical_velocity_,
            right_back_joint_target_physical_acceleration_, rb_angle_error_);
        publish_joint(
            kRightFront, right_front_joint_target_physical_angle_,
            right_front_joint_target_physical_velocity_,
            right_front_joint_target_physical_acceleration_, rf_angle_error_);
    }

    void publish_nan_joint_targets() {
        reset_attitude_correction_state_();
        reset_active_suspension_correction_state_();
        joint_target_active_.fill(false);

        *left_front_joint_target_physical_angle_ = nan_;
        *left_back_joint_target_physical_angle_ = nan_;
        *right_back_joint_target_physical_angle_ = nan_;
        *right_front_joint_target_physical_angle_ = nan_;

        *left_front_joint_target_physical_velocity_ = nan_;
        *left_back_joint_target_physical_velocity_ = nan_;
        *right_back_joint_target_physical_velocity_ = nan_;
        *right_front_joint_target_physical_velocity_ = nan_;

        *left_front_joint_target_physical_acceleration_ = nan_;
        *left_back_joint_target_physical_acceleration_ = nan_;
        *right_back_joint_target_physical_acceleration_ = nan_;
        *right_front_joint_target_physical_acceleration_ = nan_;

        *lf_angle_error_ = nan_;
        *lb_angle_error_ = nan_;
        *rb_angle_error_ = nan_;
        *rf_angle_error_ = nan_;

    }

private:
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<double> mouse_wheel_;
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
    OutputInterface<bool> active_suspension_active_;

    bool spinning_forward_ = true;
    bool apply_symmetric_target = true;
    bool complex_spin_active_ = false;
    double complex_spin_elapsed_ = 0.0;
    bool qe_complex_spin_active_ = false;
    bool qe_front_high_rear_low_ = true;
    double qe_last_toggle_elapsed_ = 0.0;
    pid::PidCalculator following_velocity_controller_;
    const double spin_ratio_;

    InputInterface<double> left_front_joint_physical_angle_;
    InputInterface<double> left_back_joint_physical_angle_;
    InputInterface<double> right_front_joint_physical_angle_;
    InputInterface<double> right_back_joint_physical_angle_;
    InputInterface<double> chassis_imu_pitch_;
    InputInterface<double> chassis_imu_roll_;
    InputInterface<double> chassis_imu_pitch_rate_;
    InputInterface<double> chassis_imu_roll_rate_;

    OutputInterface<double> scope_motor_control_torque;

    OutputInterface<double> lf_angle_error_;
    OutputInterface<double> lb_angle_error_;
    OutputInterface<double> rf_angle_error_;
    OutputInterface<double> rb_angle_error_;

    OutputInterface<double> left_front_joint_target_physical_angle_;
    OutputInterface<double> left_back_joint_target_physical_angle_;
    OutputInterface<double> right_back_joint_target_physical_angle_;
    OutputInterface<double> right_front_joint_target_physical_angle_;
    OutputInterface<double> left_front_joint_target_physical_velocity_;
    OutputInterface<double> left_back_joint_target_physical_velocity_;
    OutputInterface<double> right_back_joint_target_physical_velocity_;
    OutputInterface<double> right_front_joint_target_physical_velocity_;
    OutputInterface<double> left_front_joint_target_physical_acceleration_;
    OutputInterface<double> left_back_joint_target_physical_acceleration_;
    OutputInterface<double> right_back_joint_target_physical_acceleration_;
    OutputInterface<double> right_front_joint_target_physical_acceleration_;

    double min_angle_;
    double max_angle_;
    double active_suspension_base_angle_deg_;
    double active_suspension_base_angle_scroll_step_deg_;

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
    bool active_suspension_on_by_keyboard_ = false;
    bool active_suspension_on_by_remote_ = false;
    double suspension_velocity_limit_;
    double suspension_acceleration_limit_;
    double suspension_correction_velocity_limit_;
    double suspension_correction_acceleration_limit_;
    std::array<bool, kJointCount> joint_suspension_active_ = {false, false, false, false};
    std::array<double, kJointCount> suspension_correction_target_rad_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> suspension_correction_state_rad_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> suspension_correction_velocity_state_rad_ = {
        0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> suspension_correction_acceleration_state_rad_ = {
        0.0, 0.0, 0.0, 0.0};
    pid::PidCalculator pitch_outer_pid_{};
    pid::PidCalculator pitch_inner_pid_{};
    pid::PidCalculator roll_outer_pid_{};
    pid::PidCalculator roll_inner_pid_{};
    double chassis_imu_pitch_offset_ = 0.0;
    double chassis_imu_roll_offset_ = 0.0;
    double chassis_imu_calibration_wait_time_;
    double chassis_imu_calibration_sample_time_;
    double chassis_imu_calibration_hold_elapsed_ = 0.0;
    size_t chassis_imu_calibration_sample_count_ = 0;
    double chassis_imu_pitch_sum_ = 0.0;
    double chassis_imu_roll_sum_ = 0.0;
    bool chassis_imu_calibration_completed_for_window_ = false;
    bool chassis_imu_calibrated_once_ = false;
    static constexpr double default_dt_ = 1e-3;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::DeformableChassis, rmcs_executor::Component)
