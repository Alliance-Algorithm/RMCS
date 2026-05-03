#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <numbers>
#include <stdexcept>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::chassis {

class DeformableChassis
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    enum class JointFeedbackSource : uint8_t { kLegacyEncoderAngle, kMotorAngle };
    enum class SuspensionPhase : uint8_t { kInactive, kArming, kActive, kReleasing };
    enum JointIndex : size_t {
        kLeftFront = 0,
        kLeftBack = 1,
        kRightBack = 2,
        kRightFront = 3,
        kJointCount = 4,
    };

    struct JointFeedbackFrame {
        std::array<double, kJointCount> motor_angles{};
        std::array<double, kJointCount> physical_angles{};
        std::array<double, kJointCount> physical_velocities{};
        std::array<double, kJointCount> joint_torques{};
        std::array<double, kJointCount> eso_z2{};
        std::array<double, kJointCount> eso_z3{};
    };

    struct LegFeedback {
        double motor_angle = std::numeric_limits<double>::quiet_NaN();
        double physical_angle = std::numeric_limits<double>::quiet_NaN();
        double physical_velocity = std::numeric_limits<double>::quiet_NaN();
        double joint_torque = std::numeric_limits<double>::quiet_NaN();
        double eso_z2 = std::numeric_limits<double>::quiet_NaN();
        double eso_z3 = std::numeric_limits<double>::quiet_NaN();
    };

    struct AttitudeBias {
        double pitch_force = 0.0;
        double roll_force = 0.0;
    };

    struct LegControlState {
        SuspensionPhase phase = SuspensionPhase::kInactive;
        double support_force = 0.0;
        double contact_confidence = 1.0;
        double filtered_contact_confidence = 1.0;
        double phase_elapsed = 0.0;
        bool requested_deploy = false;
        bool output_active = false;
        bool contact_latched = false;
    };

    struct LegCommand {
        double requested_target_angle = std::numeric_limits<double>::quiet_NaN();
        double final_target_angle = std::numeric_limits<double>::quiet_NaN();
        double target_velocity = 0.0;
        double target_acceleration = 0.0;
        bool suspension_mode = false;
        double suspension_torque = std::numeric_limits<double>::quiet_NaN();
    };

    struct AttitudePidAxis {
        double kp = 20.0;
        double ki = 0.0;
        double kd = 0.0;
        double integral = 0.0;
        double integral_limit = std::numeric_limits<double>::infinity();
        double output_limit = std::numeric_limits<double>::infinity();

        void reset() { integral = 0.0; }

        double update(double error, double rate, double dt) {
            if (!std::isfinite(error) || !std::isfinite(rate) || !std::isfinite(dt) || dt <= 0.0) {
                reset();
                return std::numeric_limits<double>::quiet_NaN();
            }

            integral = std::clamp(integral + error * dt, -integral_limit, integral_limit);
            const double output = kp * error + ki * integral - kd * rate;
            return std::clamp(output, -output_limit, output_limit);
        }
    };

    DeformableChassis()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , following_velocity_controller_(10.0, 0.0, 0.0)
        , spin_ratio_(std::clamp(get_parameter_or("spin_ratio", 0.6), 0.0, 1.0))

        , min_angle_(get_parameter_or("min_angle", 15.0))
        , max_angle_(get_parameter_or("max_angle", 55.0))
        , left_front_joint_offset_(get_parameter_or("left_front_joint_offset", 0.0))
        , left_back_joint_offset_(get_parameter_or("left_back_joint_offset", 0.0))
        , right_front_joint_offset_(get_parameter_or("right_front_joint_offset", 0.0))
        , right_back_joint_offset_(get_parameter_or("right_back_joint_offset", 0.0))
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
        , active_suspension_mass_(get_parameter_or("active_suspension_mass", 22.5))
        , active_suspension_rod_length_(get_parameter_or("active_suspension_rod_length", 0.150))
        , active_suspension_Kz_(get_parameter_or("active_suspension_Kz", 150.0))
        , active_suspension_Kp_(get_parameter_or("active_suspension_Kp", 200.0))
        , active_suspension_pitch_ki_(get_parameter_or("active_suspension_pitch_ki", 0.0))
        , active_suspension_Dp_(get_parameter_or("active_suspension_Dp", 20.0))
        , active_suspension_Kr_(get_parameter_or("active_suspension_Kr", 200.0))
        , active_suspension_roll_ki_(get_parameter_or("active_suspension_roll_ki", 0.0))
        , active_suspension_Dr_(get_parameter_or("active_suspension_Dr", 20.0))
        , active_suspension_D_leg_(get_parameter_or("active_suspension_D_leg", 10.0))
        , active_suspension_com_height_(get_parameter_or("active_suspension_com_height", 0.15))
        , active_suspension_wheel_base_half_x_(get_parameter_or(
              "active_suspension_wheel_base_half_x", 0.2341741 / std::numbers::sqrt2))
        , active_suspension_wheel_base_half_y_(get_parameter_or(
              "active_suspension_wheel_base_half_y", 0.2341741 / std::numbers::sqrt2))
        , active_suspension_gravity_comp_gain_(
              get_parameter_or("active_suspension_gravity_comp_gain", 1.0))
        , active_suspension_control_acceleration_limit_(
              std::abs(get_parameter_or("active_suspension_control_acceleration_limit", 6.0)))
        , active_suspension_preload_angle_(
              std::abs(get_parameter_or("active_suspension_preload_angle_deg", 8.0))
              * std::numbers::pi / 180.0)
        , active_suspension_entry_offset_(
              std::abs(get_parameter_or(
                  "active_suspension_entry_offset_deg",
                  get_parameter_or("active_suspension_enter_deploy_tolerance_deg", 1.5)))
              * std::numbers::pi / 180.0)
        , active_suspension_ride_height_offset_(
              std::abs(get_parameter_or("active_suspension_ride_height_offset_deg", 0.0))
              * std::numbers::pi / 180.0)
        , active_suspension_hold_travel_(
              std::abs(get_parameter_or(
                  "active_suspension_hold_travel_deg",
                  get_parameter_or("active_suspension_exit_deploy_tolerance_deg", 3.0)))
              * std::numbers::pi / 180.0)
        , active_suspension_activation_velocity_threshold_(
              get_parameter_or("active_suspension_activation_velocity_threshold_deg", 15.0)
              * std::numbers::pi / 180.0)
        , active_suspension_target_physical_velocity_limit_(
              std::max(
                  deg_to_rad(
                      std::abs(get_parameter_or(
                          "active_suspension_target_velocity_limit_deg",
                          get_parameter_or("target_physical_velocity_limit", 180.0)))),
                  1e-6))
        , active_suspension_target_physical_acceleration_limit_(
              std::max(
                  deg_to_rad(
                      std::abs(get_parameter_or(
                          "active_suspension_target_acceleration_limit_deg",
                          get_parameter_or("target_physical_acceleration_limit", 720.0)))),
                  1e-6))
        , active_suspension_torque_limit_(
              std::abs(get_parameter_or("active_suspension_torque_limit", 80.0)))
        , active_suspension_pitch_angle_diff_limit_(
              std::abs(get_parameter_or(
                  "active_suspension_pitch_angle_diff_limit_deg", max_angle_ - min_angle_))
              * std::numbers::pi / 180.0)
        , active_suspension_roll_angle_diff_limit_(
              std::abs(get_parameter_or(
                  "active_suspension_roll_angle_diff_limit_deg", max_angle_ - min_angle_))
              * std::numbers::pi / 180.0)
        , active_suspension_pid_integral_limit_(
              std::abs(get_parameter_or(
                  "active_suspension_pid_integral_limit_deg", max_angle_ - min_angle_))
              * std::numbers::pi / 180.0)
        , chassis_imu_calibration_wait_time_(
              std::max(get_parameter_or("chassis_imu_calibration_wait_s", 2.0), 0.0))
        , chassis_imu_calibration_sample_time_(
              std::max(get_parameter_or("chassis_imu_calibration_sample_s", 3.0), 1e-6)) {

        following_velocity_controller_.output_max = angular_velocity_max_;
        following_velocity_controller_.output_min = -angular_velocity_max_;
        pitch_attitude_pid_.kp = active_suspension_Kp_;
        pitch_attitude_pid_.ki = active_suspension_pitch_ki_;
        pitch_attitude_pid_.kd = active_suspension_Dp_;
        pitch_attitude_pid_.integral_limit = active_suspension_pid_integral_limit_;
        pitch_attitude_pid_.output_limit = active_suspension_pitch_angle_diff_limit_;
        roll_attitude_pid_.kp = active_suspension_Kr_;
        roll_attitude_pid_.ki = active_suspension_roll_ki_;
        roll_attitude_pid_.kd = active_suspension_Dr_;
        roll_attitude_pid_.integral_limit = active_suspension_pid_integral_limit_;
        roll_attitude_pid_.output_limit = active_suspension_roll_angle_diff_limit_;

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/rotary_knob", rotary_knob_);
        register_input("/predefined/update_rate", update_rate_);

        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_, false);
        register_input("/gimbal/yaw/control_angle_error", gimbal_yaw_angle_error_, false);

        register_input("/chassis/left_front_joint/angle", left_front_joint_angle_, false);
        register_input("/chassis/left_back_joint/angle", left_back_joint_angle_, false);
        register_input("/chassis/right_front_joint/angle", right_front_joint_angle_, false);
        register_input("/chassis/right_back_joint/angle", right_back_joint_angle_, false);

        register_input(
            "/chassis/left_front_joint/physical_angle", left_front_joint_physical_angle_, false);
        register_input(
            "/chassis/left_back_joint/physical_angle", left_back_joint_physical_angle_, false);
        register_input(
            "/chassis/right_front_joint/physical_angle", right_front_joint_physical_angle_, false);
        register_input(
            "/chassis/right_back_joint/physical_angle", right_back_joint_physical_angle_, false);
        register_input(
            "/chassis/left_front_joint/physical_velocity", left_front_joint_physical_velocity_,
            false);
        register_input(
            "/chassis/left_back_joint/physical_velocity", left_back_joint_physical_velocity_,
            false);
        register_input(
            "/chassis/right_back_joint/physical_velocity", right_back_joint_physical_velocity_,
            false);
        register_input(
            "/chassis/right_front_joint/physical_velocity", right_front_joint_physical_velocity_,
            false);
        register_input("/chassis/left_front_joint/torque", left_front_joint_torque_, false);
        register_input("/chassis/left_back_joint/torque", left_back_joint_torque_, false);
        register_input("/chassis/right_back_joint/torque", right_back_joint_torque_, false);
        register_input("/chassis/right_front_joint/torque", right_front_joint_torque_, false);

        register_input(
            "/chassis/left_front_joint/encoder_angle", left_front_joint_encoder_angle_, false);
        register_input(
            "/chassis/left_back_joint/encoder_angle", left_back_joint_encoder_angle_, false);
        register_input(
            "/chassis/right_front_joint/encoder_angle", right_front_joint_encoder_angle_, false);
        register_input(
            "/chassis/right_back_joint/encoder_angle", right_back_joint_encoder_angle_, false);
        register_input("/chassis/imu/pitch", chassis_imu_pitch_, false);
        register_input("/chassis/imu/roll", chassis_imu_roll_, false);
        register_input("/chassis/imu/pitch_rate", chassis_imu_pitch_rate_, false);
        register_input("/chassis/imu/roll_rate", chassis_imu_roll_rate_, false);
        register_input("/chassis/left_front_joint/eso_z2", left_front_joint_eso_z2_, false);
        register_input("/chassis/left_front_joint/eso_z3", left_front_joint_eso_z3_, false);
        register_input("/chassis/left_back_joint/eso_z2", left_back_joint_eso_z2_, false);
        register_input("/chassis/left_back_joint/eso_z3", left_back_joint_eso_z3_, false);
        register_input("/chassis/right_back_joint/eso_z2", right_back_joint_eso_z2_, false);
        register_input("/chassis/right_back_joint/eso_z3", right_back_joint_eso_z3_, false);
        register_input("/chassis/right_front_joint/eso_z2", right_front_joint_eso_z2_, false);
        register_input("/chassis/right_front_joint/eso_z3", right_front_joint_eso_z3_, false);

        register_output("/gimbal/scope/control_torque", scope_motor_control_torque, nan_);

        register_output("/chassis/angle", chassis_angle_, nan_);
        register_output("/chassis/control_angle", chassis_control_angle_, nan_);

        register_output("/chassis/control_mode", mode_);
        register_output("/chassis/control_velocity", chassis_control_velocity_);

        register_output("/chassis/left_front_joint/control_angle_error", lf_angle_error_, nan_);
        register_output("/chassis/left_back_joint/control_angle_error", lb_angle_error_, nan_);
        register_output("/chassis/right_front_joint/control_angle_error", rf_angle_error_, nan_);
        register_output("/chassis/right_back_joint/control_angle_error", rb_angle_error_, nan_);

        register_output(
            "/chassis/left_front_joint/target_angle", left_front_joint_target_angle_, nan_);
        register_output(
            "/chassis/left_back_joint/target_angle", left_back_joint_target_angle_, nan_);
        register_output(
            "/chassis/right_back_joint/target_angle", right_back_joint_target_angle_, nan_);
        register_output(
            "/chassis/right_front_joint/target_angle", right_front_joint_target_angle_, nan_);

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
        register_output(
            "/chassis/left_front_joint/suspension_mode", left_front_joint_suspension_mode_, false);
        register_output(
            "/chassis/left_back_joint/suspension_mode", left_back_joint_suspension_mode_, false);
        register_output(
            "/chassis/right_back_joint/suspension_mode", right_back_joint_suspension_mode_, false);
        register_output(
            "/chassis/right_front_joint/suspension_mode", right_front_joint_suspension_mode_,
            false);
        register_output(
            "/chassis/left_front_joint/suspension_torque", left_front_joint_suspension_torque_,
            nan_);
        register_output(
            "/chassis/left_back_joint/suspension_torque", left_back_joint_suspension_torque_, nan_);
        register_output(
            "/chassis/right_back_joint/suspension_torque", right_back_joint_suspension_torque_,
            nan_);
        register_output(
            "/chassis/right_front_joint/suspension_torque", right_front_joint_suspension_torque_,
            nan_);

        register_output("/chassis/processed_encoder/angle", processed_encoder_angle_, nan_);

        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        chassis_control_velocity_->vector << nan_, nan_, nan_;

        current_target_angle_ = max_angle_;
        lf_current_target_angle_ = max_angle_;
        lb_current_target_angle_ = max_angle_;
        rf_current_target_angle_ = max_angle_;
        rb_current_target_angle_ = max_angle_;

        const bool left_front_joint_offset = has_parameter("left_front_joint_offset");
        const bool left_back_joint_offset = has_parameter("left_back_joint_offset");
        const bool right_front_joint_offset = has_parameter("right_front_joint_offset");
        const bool right_back_joint_offset = has_parameter("right_back_joint_offset");

        const bool has_any_joint_offset = left_front_joint_offset || left_back_joint_offset
                                       || right_front_joint_offset || right_back_joint_offset;
        const bool has_all_joint_offsets = left_front_joint_offset && left_back_joint_offset
                                        && right_front_joint_offset && right_back_joint_offset;
        if (has_any_joint_offset && !has_all_joint_offsets)
            throw std::runtime_error(
                "deformable chassis joint offsets must be configured for all four joints or "
                "removed entirely");

        joint_feedback_source_ = has_all_joint_offsets ? JointFeedbackSource::kLegacyEncoderAngle
                                                       : JointFeedbackSource::kMotorAngle;
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
        if (!left_front_joint_torque_.ready())
            left_front_joint_torque_.make_and_bind_directly(0.0);
        if (!left_back_joint_torque_.ready())
            left_back_joint_torque_.make_and_bind_directly(0.0);
        if (!right_back_joint_torque_.ready())
            right_back_joint_torque_.make_and_bind_directly(0.0);
        if (!right_front_joint_torque_.ready())
            right_front_joint_torque_.make_and_bind_directly(0.0);
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
            update_velocity_control();
            update_lift_target_toggle(keyboard);
            run_joint_intent_pipeline_();
        } while (false);

        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;
        last_keyboard_ = keyboard;
    }

private:
    static constexpr double inf_ = std::numeric_limits<double>::infinity();
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kGravity_ = 9.81;
    static constexpr double kMaxAttitudeRad_ = 30.0 * std::numbers::pi / 180.0;
    static constexpr double kMinForceArmSin_ = 0.1;
    static constexpr double kContactConfidenceEnterThreshold_ = 0.55;
    static constexpr double kContactConfidenceExitThreshold_ = 0.35;
    static constexpr double kContactConfidenceFilterAlpha_ = 0.25;
    static constexpr double kMinimumArmingTime_ = 0.02;

    static constexpr double translational_velocity_max_ = 10.0;
    static constexpr double angular_velocity_max_ = 30.0;
    static constexpr double rad_to_deg_ = 180.0 / std::numbers::pi;
    static constexpr std::array<double, kJointCount> kPitchSigns_ = {-1.0, 1.0, 1.0, -1.0};
    static constexpr std::array<double, kJointCount> kRollSigns_ = {1.0, 1.0, -1.0, -1.0};

    static double wrap_deg(double deg) {
        deg = std::fmod(deg, 360.0);
        if (deg >= 180.0)
            deg -= 360.0;
        if (deg < -180.0)
            deg += 360.0;
        return deg;
    }

    void validate_joint_feedback_inputs() const {
        const bool ready =
            joint_feedback_source_ == JointFeedbackSource::kMotorAngle
                ? left_front_joint_angle_.ready() && left_back_joint_angle_.ready()
                      && right_front_joint_angle_.ready() && right_back_joint_angle_.ready()
                : left_front_joint_encoder_angle_.ready() && left_back_joint_encoder_angle_.ready()
                      && right_front_joint_encoder_angle_.ready()
                      && right_back_joint_encoder_angle_.ready();

        if (ready)
            return;

        throw std::runtime_error(
            joint_feedback_source_ == JointFeedbackSource::kMotorAngle
                ? "missing V2 joint feedback interfaces: expected /chassis/*_joint/angle"
                : "missing legacy joint feedback interfaces: expected /chassis/*_joint/"
                  "encoder_angle");
    }

    double joint_angle_deg(
        const InputInterface<double>& joint_angle,
        const InputInterface<double>& joint_encoder_angle, double joint_offset,
        double legacy_fixed_compensation) const {
        if (joint_feedback_source_ == JointFeedbackSource::kMotorAngle)
            return wrap_deg(*joint_angle * rad_to_deg_);

        return wrap_deg(joint_offset) - wrap_deg(*joint_encoder_angle) + legacy_fixed_compensation;
    }

    void clear_suspension_output_interfaces_() {
        *left_front_joint_suspension_mode_ = false;
        *left_back_joint_suspension_mode_ = false;
        *right_back_joint_suspension_mode_ = false;
        *right_front_joint_suspension_mode_ = false;

        *left_front_joint_suspension_torque_ = nan_;
        *left_back_joint_suspension_torque_ = nan_;
        *right_back_joint_suspension_torque_ = nan_;
        *right_front_joint_suspension_torque_ = nan_;
    }

    void reset_suspension_internal_state_() {
        joint_suspension_active_.fill(false);
        for (size_t index = 0; index < kJointCount; ++index) {
            auto& leg_state = leg_control_states_[index];
            leg_state.phase = SuspensionPhase::kInactive;
            leg_state.support_force = 0.0;
            leg_state.contact_confidence = 1.0;
            leg_state.filtered_contact_confidence = 1.0;
            leg_state.phase_elapsed = 0.0;
            leg_state.requested_deploy = false;
            leg_state.output_active = false;
            leg_state.contact_latched = false;
            leg_commands_[index] = LegCommand{};
        }
    }

    void publish_suspension_outputs_() {
        *left_front_joint_suspension_mode_ = leg_commands_[kLeftFront].suspension_mode;
        *left_back_joint_suspension_mode_ = leg_commands_[kLeftBack].suspension_mode;
        *right_back_joint_suspension_mode_ = leg_commands_[kRightBack].suspension_mode;
        *right_front_joint_suspension_mode_ = leg_commands_[kRightFront].suspension_mode;

        *left_front_joint_suspension_torque_ = leg_commands_[kLeftFront].suspension_torque;
        *left_back_joint_suspension_torque_ = leg_commands_[kLeftBack].suspension_torque;
        *right_back_joint_suspension_torque_ = leg_commands_[kRightBack].suspension_torque;
        *right_front_joint_suspension_torque_ = leg_commands_[kRightFront].suspension_torque;
    }

    void update_mode_from_inputs_(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right,
        const rmcs_msgs::Keyboard& keyboard) {
        auto mode = *mode_;
        if (switch_left == rmcs_msgs::Switch::DOWN)
            return;

        if (last_switch_right_ == rmcs_msgs::Switch::MIDDLE
            && switch_right == rmcs_msgs::Switch::DOWN) {
            if (mode == rmcs_msgs::ChassisMode::SPIN) {
                mode = rmcs_msgs::ChassisMode::STEP_DOWN;
            } else {
                mode = rmcs_msgs::ChassisMode::SPIN;
                spinning_forward_ = !spinning_forward_;
            }
        } else if (!last_keyboard_.c && keyboard.c) {
            if (mode == rmcs_msgs::ChassisMode::SPIN) {
                mode = rmcs_msgs::ChassisMode::AUTO;
            } else {
                mode = rmcs_msgs::ChassisMode::SPIN;
                spinning_forward_ = !spinning_forward_;
            }
        } else if (!last_keyboard_.x && keyboard.x) {
            mode = mode == rmcs_msgs::ChassisMode::LAUNCH_RAMP
                     ? rmcs_msgs::ChassisMode::AUTO
                     : rmcs_msgs::ChassisMode::LAUNCH_RAMP;
        } else if (!last_keyboard_.z && keyboard.z) {
            mode = mode == rmcs_msgs::ChassisMode::STEP_DOWN ? rmcs_msgs::ChassisMode::AUTO
                                                             : rmcs_msgs::ChassisMode::STEP_DOWN;
        }

        *mode_ = mode;
    }

    // JointFeedbackAdapter: normalize motor / physical / legacy encoder feedback into one frame.
    JointFeedbackFrame read_joint_feedback_frame_() const {
        JointFeedbackFrame joint_feedback;
        update_current_joint_feedback(
            joint_feedback.motor_angles, joint_feedback.physical_angles,
            joint_feedback.physical_velocities, joint_feedback.joint_torques, joint_feedback.eso_z2,
            joint_feedback.eso_z3);
        return joint_feedback;
    }

    bool prone_override_requested_() const { return keyboard_.ready() && keyboard_->ctrl; }

    bool suspension_requested_by_switch_() const {
        return switch_left_.ready() && switch_right_.ready()
            && *switch_left_ == rmcs_msgs::Switch::DOWN && *switch_right_ == rmcs_msgs::Switch::UP;
    }

    bool suspension_requested_by_input_() const {
        return active_suspension_enable_
            && (prone_override_requested_() || suspension_requested_by_switch_());
    }

    bool suspension_deploy_requested_() const { return suspension_requested_by_input_(); }

    bool symmetric_joint_target_requested_() const {
        constexpr double epsilon = 1e-6;
        return std::abs(lf_current_target_angle_ - lb_current_target_angle_) <= epsilon
            && std::abs(lf_current_target_angle_ - rb_current_target_angle_) <= epsilon
            && std::abs(lf_current_target_angle_ - rf_current_target_angle_) <= epsilon;
    }

    bool refresh_requested_joint_targets_from_deploy_state_() {
        requested_target_physical_angles_rad_[kLeftFront] = deg_to_rad(lf_current_target_angle_);
        requested_target_physical_angles_rad_[kLeftBack] = deg_to_rad(lb_current_target_angle_);
        requested_target_physical_angles_rad_[kRightBack] = deg_to_rad(rb_current_target_angle_);
        requested_target_physical_angles_rad_[kRightFront] = deg_to_rad(rf_current_target_angle_);

        const bool prone_override = prone_override_requested_();
        if (suspension_deploy_requested_()) {
            requested_target_physical_angles_rad_.fill(deg_to_rad(min_angle_));
        }

        current_target_physical_angles_rad_ = requested_target_physical_angles_rad_;
        return prone_override;
    }

    void reset_attitude_correction_state_() {
        pitch_attitude_pid_.reset();
        roll_attitude_pid_.reset();
        reset_suspension_internal_state_();
    }

    void reset_chassis_imu_calibration_window_() {
        chassis_imu_calibration_hold_elapsed_ = 0.0;
        chassis_imu_calibration_sample_count_ = 0;
        chassis_imu_pitch_sum_ = 0.0;
        chassis_imu_roll_sum_ = 0.0;
        chassis_imu_calibration_completed_for_window_ = false;
    }

    void update_chassis_imu_calibration_() {
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

        chassis_imu_pitch_offset_ =
            chassis_imu_pitch_sum_ / static_cast<double>(chassis_imu_calibration_sample_count_);
        chassis_imu_roll_offset_ =
            chassis_imu_roll_sum_ / static_cast<double>(chassis_imu_calibration_sample_count_);
        RCLCPP_INFO(
            get_logger(),
            "[chassis imu calibration] pitch_offset=% .3f deg roll_offset=% .3f deg "
            "(samples=%zu)",
            chassis_imu_pitch_offset_ * rad_to_deg_, chassis_imu_roll_offset_ * rad_to_deg_,
            chassis_imu_calibration_sample_count_);
    }

    void update_current_joint_feedback(
        std::array<double, kJointCount>& current_motor_angles,
        std::array<double, kJointCount>& current_physical_angles,
        std::array<double, kJointCount>& current_physical_velocities,
        std::array<double, kJointCount>& current_joint_torques,
        std::array<double, kJointCount>& current_eso_z2,
        std::array<double, kJointCount>& current_eso_z3) const {
        const std::array<const InputInterface<double>*, kJointCount> motor_angle_inputs{
            &left_front_joint_angle_, &left_back_joint_angle_, &right_back_joint_angle_,
            &right_front_joint_angle_};
        const std::array<const InputInterface<double>*, kJointCount> physical_angle_inputs{
            &left_front_joint_physical_angle_, &left_back_joint_physical_angle_,
            &right_back_joint_physical_angle_, &right_front_joint_physical_angle_};
        const std::array<const InputInterface<double>*, kJointCount> physical_velocity_inputs{
            &left_front_joint_physical_velocity_, &left_back_joint_physical_velocity_,
            &right_back_joint_physical_velocity_, &right_front_joint_physical_velocity_};
        const std::array<const InputInterface<double>*, kJointCount> torque_inputs{
            &left_front_joint_torque_, &left_back_joint_torque_, &right_back_joint_torque_,
            &right_front_joint_torque_};
        const std::array<const InputInterface<double>*, kJointCount> eso_z2_inputs{
            &left_front_joint_eso_z2_, &left_back_joint_eso_z2_, &right_back_joint_eso_z2_,
            &right_front_joint_eso_z2_};
        const std::array<const InputInterface<double>*, kJointCount> eso_z3_inputs{
            &left_front_joint_eso_z3_, &left_back_joint_eso_z3_, &right_back_joint_eso_z3_,
            &right_front_joint_eso_z3_};

        current_motor_angles.fill(nan_);
        current_physical_angles.fill(nan_);
        current_physical_velocities.fill(nan_);
        current_joint_torques.fill(nan_);
        current_eso_z2.fill(nan_);
        current_eso_z3.fill(nan_);

        for (size_t i = 0; i < kJointCount; ++i) {
            if (motor_angle_inputs[i]->ready() && std::isfinite(*(*motor_angle_inputs[i]))) {
                current_motor_angles[i] = *(*motor_angle_inputs[i]);
                current_physical_angles[i] = motor_to_physical_angle(current_motor_angles[i]);
            }

            if (physical_angle_inputs[i]->ready() && std::isfinite(*(*physical_angle_inputs[i]))) {
                current_physical_angles[i] = *(*physical_angle_inputs[i]);
            }
            if (physical_velocity_inputs[i]->ready()
                && std::isfinite(*(*physical_velocity_inputs[i]))) {
                current_physical_velocities[i] = *(*physical_velocity_inputs[i]);
            }
            if (torque_inputs[i]->ready() && std::isfinite(*(*torque_inputs[i]))) {
                current_joint_torques[i] = *(*torque_inputs[i]);
            }
            if (eso_z2_inputs[i]->ready() && std::isfinite(*(*eso_z2_inputs[i]))) {
                current_eso_z2[i] = *(*eso_z2_inputs[i]);
            }
            if (eso_z3_inputs[i]->ready() && std::isfinite(*(*eso_z3_inputs[i]))) {
                current_eso_z3[i] = *(*eso_z3_inputs[i]);
            }
        }
    }

    bool initialize_joint_target_states_from_feedback(
        const std::array<double, kJointCount>& current_motor_angles,
        const std::array<double, kJointCount>& current_physical_angles) {
        for (size_t i = 0; i < kJointCount; ++i) {
            if (!std::isfinite(current_motor_angles[i])
                || !std::isfinite(current_physical_angles[i]))
                return false;
        }

        joint_target_angle_state_rad_ = current_motor_angles;
        joint_target_physical_angle_state_rad_ = current_physical_angles;
        joint_target_physical_velocity_state_rad_ = {0.0, 0.0, 0.0, 0.0};
        joint_target_physical_acceleration_state_rad_ = {0.0, 0.0, 0.0, 0.0};
        requested_target_physical_angles_rad_ = current_physical_angles;
        current_target_physical_angles_rad_ = current_physical_angles;
        joint_target_active_ = true;
        return true;
    }

    void sync_joint_target_state_from_feedback(
        JointIndex index, double current_motor_angle, double current_physical_angle) {
        joint_target_angle_state_rad_[index] = current_motor_angle;
        joint_target_physical_angle_state_rad_[index] = current_physical_angle;
        joint_target_physical_velocity_state_rad_[index] = 0.0;
        joint_target_physical_acceleration_state_rad_[index] = 0.0;
    }

    static LegFeedback leg_feedback_at_(const JointFeedbackFrame& joint_feedback, size_t index) {
        return LegFeedback{
            .motor_angle = joint_feedback.motor_angles[index],
            .physical_angle = joint_feedback.physical_angles[index],
            .physical_velocity = joint_feedback.physical_velocities[index],
            .joint_torque = joint_feedback.joint_torques[index],
            .eso_z2 = joint_feedback.eso_z2[index],
            .eso_z3 = joint_feedback.eso_z3[index],
        };
    }

    double estimate_contact_confidence_(const LegFeedback& leg_feedback) const {
        double confidence = 1.0;
        if (std::isfinite(leg_feedback.eso_z3)) {
            confidence -= std::clamp(std::abs(leg_feedback.eso_z3) / 80.0, 0.0, 0.5);
        }
        if (std::isfinite(leg_feedback.joint_torque)) {
            confidence += std::clamp(std::abs(leg_feedback.joint_torque) / 20.0, 0.0, 0.3);
        }
        if (std::isfinite(leg_feedback.physical_velocity)) {
            confidence -= std::clamp(std::abs(leg_feedback.physical_velocity) / 10.0, 0.0, 0.2);
        }
        return std::clamp(confidence, 0.0, 1.0);
    }

    bool contact_ready_(const LegControlState& leg_state) const {
        return leg_state.contact_latched
            || leg_state.filtered_contact_confidence >= kContactConfidenceEnterThreshold_;
    }

    void prepare_leg_commands_for_cycle_() {
        for (size_t index = 0; index < kJointCount; ++index) {
            leg_commands_[index] = LegCommand{
                .requested_target_angle = requested_target_physical_angles_rad_[index],
                .final_target_angle = current_target_physical_angles_rad_[index],
                .target_velocity = joint_target_physical_velocity_state_rad_[index],
                .target_acceleration = joint_target_physical_acceleration_state_rad_[index],
                .suspension_mode = false,
                .suspension_torque = nan_,
            };
            leg_control_states_[index].output_active = false;
            leg_control_states_[index].support_force = 0.0;
        }
    }

    void update_leg_contact_estimates_(const JointFeedbackFrame& joint_feedback) {
        for (size_t index = 0; index < kJointCount; ++index) {
            const LegFeedback leg_feedback = leg_feedback_at_(joint_feedback, index);
            auto& leg_state = leg_control_states_[index];
            leg_state.contact_confidence = estimate_contact_confidence_(leg_feedback);
            leg_state.filtered_contact_confidence = std::clamp(
                (1.0 - kContactConfidenceFilterAlpha_) * leg_state.filtered_contact_confidence
                    + kContactConfidenceFilterAlpha_ * leg_state.contact_confidence,
                0.0, 1.0);
            if (leg_state.filtered_contact_confidence >= kContactConfidenceEnterThreshold_) {
                leg_state.contact_latched = true;
            } else if (leg_state.filtered_contact_confidence <= kContactConfidenceExitThreshold_) {
                leg_state.contact_latched = false;
            }
        }
    }

    void update_leg_states_(
        const JointFeedbackFrame& joint_feedback, double entry_angle, double release_angle,
        double dt) {
        for (size_t index = 0; index < kJointCount; ++index) {
            const LegFeedback leg_feedback = leg_feedback_at_(joint_feedback, index);
            const bool requested_deploy =
                std::isfinite(requested_target_physical_angles_rad_[index])
                && requested_target_physical_angles_rad_[index] <= entry_angle;
            update_leg_suspension_state_(
                index, leg_feedback, requested_deploy, entry_angle, release_angle, dt);
        }
    }

    void compute_leg_support_intents_(
        const JointFeedbackFrame& joint_feedback, const AttitudeBias& attitude_bias,
        double support_zero_angle, double ride_height_angle) {
        for (size_t index = 0; index < kJointCount; ++index) {
            const LegFeedback leg_feedback = leg_feedback_at_(joint_feedback, index);
            auto& leg_state = leg_control_states_[index];
            if (leg_state.phase != SuspensionPhase::kActive) {
                continue;
            }

            leg_state.support_force =
                compute_leg_support_force_(index, leg_feedback, attitude_bias, support_zero_angle);
            leg_commands_[index].final_target_angle = ride_height_angle;
            leg_commands_[index].suspension_mode = true;
            leg_commands_[index].suspension_torque =
                leg_force_to_joint_torque_(leg_state.support_force, leg_feedback.physical_angle);
        }
    }

    void apply_leg_commands_to_targets_() {
        for (size_t index = 0; index < kJointCount; ++index) {
            current_target_physical_angles_rad_[index] = leg_commands_[index].final_target_angle;
        }
    }

    AttitudeBias compute_attitude_force_bias_() {
        const double corrected_pitch = std::clamp(
            *chassis_imu_pitch_ - chassis_imu_pitch_offset_, -kMaxAttitudeRad_, kMaxAttitudeRad_);
        const double corrected_roll = std::clamp(
            *chassis_imu_roll_ - chassis_imu_roll_offset_, -kMaxAttitudeRad_, kMaxAttitudeRad_);
        const double corrected_pitch_rate = *chassis_imu_pitch_rate_;
        const double corrected_roll_rate = *chassis_imu_roll_rate_;
        const double dt = update_dt();

        const double pitch_force =
            pitch_attitude_pid_.update(-corrected_pitch, corrected_pitch_rate, dt);
        const double roll_force =
            roll_attitude_pid_.update(corrected_roll, -corrected_roll_rate, dt);
        if (!std::isfinite(pitch_force) || !std::isfinite(roll_force)) {
            return {.pitch_force = nan_, .roll_force = nan_};
        }
        return {.pitch_force = pitch_force, .roll_force = roll_force};
    }

    double compute_leg_support_force_(
        size_t index, const LegFeedback& leg_feedback, const AttitudeBias& attitude_bias,
        double support_zero_angle) const {
        const double gravity_force_per_wheel = active_suspension_gravity_comp_gain_
                                             * active_suspension_mass_ * kGravity_
                                             / static_cast<double>(kJointCount);
        double support_force =
            gravity_force_per_wheel
            + active_suspension_Kz_ * (leg_feedback.physical_angle - support_zero_angle)
            + active_suspension_D_leg_ * leg_feedback.physical_velocity;

        support_force += kPitchSigns_[index] * attitude_bias.pitch_force;
        support_force += kRollSigns_[index] * attitude_bias.roll_force;
        if (active_suspension_com_height_ > 0.0 && active_suspension_wheel_base_half_x_ > 1e-6
            && active_suspension_wheel_base_half_y_ > 1e-6) {
            support_force += kPitchSigns_[index] * active_suspension_mass_
                           * control_acceleration_estimate_.x() * active_suspension_com_height_
                           / (4.0 * active_suspension_wheel_base_half_x_);
            support_force += kRollSigns_[index] * active_suspension_mass_
                           * control_acceleration_estimate_.y() * active_suspension_com_height_
                           / (4.0 * active_suspension_wheel_base_half_y_);
        }
        return std::max(support_force, 0.0);
    }

    double leg_force_to_joint_torque_(double leg_force, double physical_angle) const {
        const double sin_alpha = std::max(std::sin(physical_angle), kMinForceArmSin_);
        return std::clamp(
            leg_force * active_suspension_rod_length_ * sin_alpha, -active_suspension_torque_limit_,
            active_suspension_torque_limit_);
    }

    void update_leg_suspension_state_(
        size_t index, const LegFeedback& leg_feedback, bool requested_deploy, double entry_angle,
        double release_angle, double dt) {
        auto& leg_state = leg_control_states_[index];
        if (leg_state.requested_deploy != requested_deploy) {
            leg_state.phase_elapsed = 0.0;
        } else {
            leg_state.phase_elapsed += dt;
        }
        leg_state.requested_deploy = requested_deploy;

        if (!requested_deploy || !std::isfinite(leg_feedback.physical_angle)
            || !std::isfinite(leg_feedback.physical_velocity)) {
            leg_state.phase = SuspensionPhase::kInactive;
            joint_suspension_active_[index] = false;
            leg_state.output_active = false;
            leg_state.phase_elapsed = 0.0;
            leg_state.contact_latched = false;
            return;
        }

        const bool entry_ready = leg_feedback.physical_angle <= entry_angle;
        const bool velocity_ready = std::abs(leg_feedback.physical_velocity)
                                 <= active_suspension_activation_velocity_threshold_;
        const bool contact_ready = contact_ready_(leg_state);

        switch (leg_state.phase) {
        case SuspensionPhase::kInactive:
            joint_suspension_active_[index] = false;
            leg_state.output_active = false;
            if (entry_ready) {
                leg_state.phase = SuspensionPhase::kArming;
                leg_state.phase_elapsed = 0.0;
            }
            break;

        case SuspensionPhase::kArming:
            joint_suspension_active_[index] = false;
            leg_state.output_active = false;
            if (!entry_ready) {
                leg_state.phase = SuspensionPhase::kInactive;
                leg_state.phase_elapsed = 0.0;
                break;
            }
            if (velocity_ready && std::isfinite(leg_feedback.motor_angle)
                && (contact_ready || leg_state.phase_elapsed >= kMinimumArmingTime_)) {
                sync_joint_target_state_from_feedback(
                    static_cast<JointIndex>(index), leg_feedback.motor_angle,
                    leg_feedback.physical_angle);
                joint_suspension_active_[index] = true;
                leg_state.phase = SuspensionPhase::kActive;
                leg_state.output_active = true;
                leg_state.phase_elapsed = 0.0;
            }
            break;

        case SuspensionPhase::kActive:
            if (leg_feedback.physical_angle > release_angle
                || (!contact_ready && leg_state.phase_elapsed >= kMinimumArmingTime_)) {
                joint_suspension_active_[index] = false;
                leg_state.phase = SuspensionPhase::kReleasing;
                leg_state.output_active = false;
                leg_state.phase_elapsed = 0.0;
                break;
            }
            joint_suspension_active_[index] = true;
            leg_state.output_active = true;
            break;

        case SuspensionPhase::kReleasing:
            joint_suspension_active_[index] = false;
            leg_state.output_active = false;
            if (!entry_ready || leg_state.phase_elapsed >= kMinimumArmingTime_) {
                leg_state.phase = SuspensionPhase::kInactive;
                leg_state.phase_elapsed = 0.0;
            }
            break;
        }
    }

    void update_active_suspension_(const JointFeedbackFrame& joint_feedback) {
        clear_suspension_output_interfaces_();
        prepare_leg_commands_for_cycle_();
        if (!suspension_requested_by_input_()) {
            reset_attitude_correction_state_();
            current_target_physical_angles_rad_ = requested_target_physical_angles_rad_;
            publish_suspension_outputs_();
            return;
        }

        const double deploy_angle = deg_to_rad(min_angle_);
        const double entry_angle = deploy_angle + active_suspension_entry_offset_;
        const double ride_height_angle = std::clamp(
            deploy_angle + active_suspension_ride_height_offset_, deploy_angle,
            deg_to_rad(max_angle_));
        const double support_zero_angle = deploy_angle - active_suspension_preload_angle_;
        const double release_angle = ride_height_angle + active_suspension_hold_travel_;
        const AttitudeBias attitude_bias = compute_attitude_force_bias_();
        if (!std::isfinite(attitude_bias.pitch_force) || !std::isfinite(attitude_bias.roll_force)) {
            reset_attitude_correction_state_();
            current_target_physical_angles_rad_.fill(deploy_angle);
            publish_suspension_outputs_();
            return;
        }

        const double dt = update_dt();
        update_leg_contact_estimates_(joint_feedback);
        update_leg_states_(joint_feedback, entry_angle, release_angle, dt);
        compute_leg_support_intents_(
            joint_feedback, attitude_bias, support_zero_angle, ride_height_angle);
        apply_leg_commands_to_targets_();
        publish_suspension_outputs_();
    }

    void reset_all_controls() {
        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        reset_control_acceleration_estimate();
        reset_attitude_correction_state_();
        reset_chassis_imu_calibration_window_();

        chassis_control_velocity_->vector << nan_, nan_, nan_;
        *chassis_angle_ = nan_;
        *chassis_control_angle_ = nan_;

        current_target_angle_ = max_angle_;
        lf_current_target_angle_ = current_target_angle_;
        lb_current_target_angle_ = current_target_angle_;
        rb_current_target_angle_ = current_target_angle_;
        rf_current_target_angle_ = current_target_angle_;
        joint_target_active_ = false;

        *scope_motor_control_torque = nan_;

        *lf_angle_error_ = nan_;
        *lb_angle_error_ = nan_;
        *rf_angle_error_ = nan_;
        *rb_angle_error_ = nan_;

        *left_front_joint_target_angle_ = nan_;
        *left_back_joint_target_angle_ = nan_;
        *right_back_joint_target_angle_ = nan_;
        *right_front_joint_target_angle_ = nan_;

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

        clear_suspension_output_interfaces_();
        *processed_encoder_angle_ = nan_;
    }

    void update_velocity_control() {
        const Eigen::Vector2d translational_velocity = update_translational_velocity_control();
        const double angular_velocity = update_angular_velocity_control();
        update_control_acceleration_estimate(translational_velocity);
        chassis_control_velocity_->vector << translational_velocity, angular_velocity;
    }

    double update_dt() const {
        if (update_rate_.ready() && std::isfinite(*update_rate_) && *update_rate_ > 1e-6)
            return 1.0 / *update_rate_;
        return default_dt_;
    }

    void reset_control_acceleration_estimate() {
        control_acceleration_estimate_.setZero();
        last_control_translational_velocity_.setZero();
        last_control_translational_velocity_valid_ = false;
    }

    void update_control_acceleration_estimate(const Eigen::Vector2d& translational_velocity) {
        if (!translational_velocity.array().isFinite().all()) {
            reset_control_acceleration_estimate();
            return;
        }

        if (!last_control_translational_velocity_valid_) {
            last_control_translational_velocity_ = translational_velocity;
            control_acceleration_estimate_.setZero();
            last_control_translational_velocity_valid_ = true;
            return;
        }

        const double dt = update_dt();
        const Eigen::Vector2d max_acceleration =
            Eigen::Vector2d::Constant(active_suspension_control_acceleration_limit_);
        control_acceleration_estimate_ =
            ((translational_velocity - last_control_translational_velocity_) / dt)
                .cwiseMax(-max_acceleration)
                .cwiseMin(max_acceleration);
        last_control_translational_velocity_ = translational_velocity;
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
        constexpr double rotary_knob_edge_threshold = 0.7;

        const bool keyboard_toggle_condition = !last_keyboard_.q && keyboard.q;
        const bool rotary_knob_toggle_condition =
            last_rotary_knob_ < rotary_knob_edge_threshold
            && *rotary_knob_ >= rotary_knob_edge_threshold;
        const bool front_high_rear_low = !last_keyboard_.b && keyboard.b;
        const bool front_low_rear_high = !last_keyboard_.g && keyboard.g;

        if (apply_symmetric_target) {
            lf_current_target_angle_ = current_target_angle_;
            lb_current_target_angle_ = current_target_angle_;
            rb_current_target_angle_ = current_target_angle_;
            rf_current_target_angle_ = current_target_angle_;
        }

        if (rotary_knob_toggle_condition || keyboard_toggle_condition) {
            current_target_angle_ =
                (std::abs(current_target_angle_ - max_angle_) < 1e-6) ? min_angle_ : max_angle_;
            apply_symmetric_target = true;
        } else if (front_high_rear_low) {
            lf_current_target_angle_ = max_angle_;
            rf_current_target_angle_ = max_angle_;
            lb_current_target_angle_ = min_angle_;
            rb_current_target_angle_ = min_angle_;
            apply_symmetric_target = false;
        } else if (front_low_rear_high) {
            lf_current_target_angle_ = min_angle_;
            rf_current_target_angle_ = min_angle_;
            lb_current_target_angle_ = max_angle_;
            rb_current_target_angle_ = max_angle_;
            apply_symmetric_target = false;
        }

        last_rotary_knob_ = *rotary_knob_;
    }

    // Chassis owns the high-level joint intent pipeline: read feedback, generate deploy targets,
    // coordinate suspension overrides, then publish the resulting joint intent for the servo layer.
    void run_joint_intent_pipeline_() {
        const auto joint_feedback = read_joint_feedback_frame_();

        if (!joint_target_active_
            && !initialize_joint_target_states_from_feedback(
                joint_feedback.motor_angles, joint_feedback.physical_angles)) {
            publish_nan_joint_targets();
            return;
        }

        update_chassis_imu_calibration_();
        const bool prone_override = refresh_requested_joint_targets_from_deploy_state_();
        scope_motor_control(prone_override);
        update_active_suspension_(joint_feedback);
        update_joint_target_trajectory();
        publish_joint_target_angles(joint_feedback.physical_angles);
    }

    static double deg_to_rad(double deg) { return deg * std::numbers::pi / 180.0; }

    static double physical_to_motor_angle(double physical_angle_rad) {
        return joint_zero_physical_angle_rad_ - physical_angle_rad;
    }

    static double motor_to_physical_angle(double motor_angle_rad) {
        return joint_zero_physical_angle_rad_ - motor_angle_rad;
    }

    void scope_motor_control(bool prone_override = false) {
        const bool prone_target_active = prone_override;
        if (prone_target_active && *mode_ != rmcs_msgs::ChassisMode::SPIN) {
            *scope_motor_control_torque = -0.3;
            // if (*scope_motor_velocity <= std::abs(0.1)){
            //     *scope_motor_control_torque = 0.18 * 1.0 / 36.0;
            // }
        } else {
            *scope_motor_control_torque = 0.3;
            // if (*scope_motor_velocity <= std::abs(0.1)){
            //     *scope_motor_control_torque = -0.18 * 1.0 / 36.0;
            // }
        }
    }

    bool publish_current_joint_target_angles() {
        const std::array<InputInterface<double>*, kJointCount> motor_angle_inputs{
            &left_front_joint_angle_, &left_back_joint_angle_, &right_back_joint_angle_,
            &right_front_joint_angle_};

        std::array<double, kJointCount> current_motor_angles{};
        std::array<double, kJointCount> current_physical_angles{};
        for (size_t i = 0; i < kJointCount; ++i) {
            if (!motor_angle_inputs[i]->ready() || !std::isfinite(*(*motor_angle_inputs[i]))) {
                return false;
            }
            current_motor_angles[i] = *(*motor_angle_inputs[i]);
            current_physical_angles[i] = motor_to_physical_angle(current_motor_angles[i]);
        }

        joint_target_angle_state_rad_ = current_motor_angles;
        joint_target_physical_angle_state_rad_ = current_physical_angles;
        joint_target_physical_velocity_state_rad_ = {0.0, 0.0, 0.0, 0.0};
        joint_target_physical_acceleration_state_rad_ = {0.0, 0.0, 0.0, 0.0};
        requested_target_physical_angles_rad_ = current_physical_angles;
        current_target_physical_angles_rad_ = current_physical_angles;
        joint_target_active_ = true;
        return true;
    }

    void update_joint_target_trajectory() {
        const double dt = update_dt();
        for (size_t i = 0; i < kJointCount; ++i) {
            double& angle_state = joint_target_physical_angle_state_rad_[i];
            double& velocity_state = joint_target_physical_velocity_state_rad_[i];
            double& acceleration_state = joint_target_physical_acceleration_state_rad_[i];
            const double target_angle = current_target_physical_angles_rad_[i];
            const double velocity_limit = joint_suspension_active_[i]
                                            ? active_suspension_target_physical_velocity_limit_
                                            : target_physical_velocity_limit_;
            const double acceleration_limit =
                joint_suspension_active_[i] ? active_suspension_target_physical_acceleration_limit_
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

            joint_target_angle_state_rad_[i] = physical_to_motor_angle(angle_state);
        }
    }

    void publish_joint_target_angles(
        const std::array<double, kJointCount>& current_physical_angles) {
        if (!joint_target_active_) {
            publish_nan_joint_targets();
            return;
        }

        *left_front_joint_target_angle_ = joint_target_angle_state_rad_[kLeftFront];
        *left_back_joint_target_angle_ = joint_target_angle_state_rad_[kLeftBack];
        *right_back_joint_target_angle_ = joint_target_angle_state_rad_[kRightBack];
        *right_front_joint_target_angle_ = joint_target_angle_state_rad_[kRightFront];

        *left_front_joint_target_physical_angle_ =
            joint_target_physical_angle_state_rad_[kLeftFront];
        *left_back_joint_target_physical_angle_ = joint_target_physical_angle_state_rad_[kLeftBack];
        *right_back_joint_target_physical_angle_ =
            joint_target_physical_angle_state_rad_[kRightBack];
        *right_front_joint_target_physical_angle_ =
            joint_target_physical_angle_state_rad_[kRightFront];

        *left_front_joint_target_physical_velocity_ =
            joint_target_physical_velocity_state_rad_[kLeftFront];
        *left_back_joint_target_physical_velocity_ =
            joint_target_physical_velocity_state_rad_[kLeftBack];
        *right_back_joint_target_physical_velocity_ =
            joint_target_physical_velocity_state_rad_[kRightBack];
        *right_front_joint_target_physical_velocity_ =
            joint_target_physical_velocity_state_rad_[kRightFront];

        *left_front_joint_target_physical_acceleration_ =
            joint_target_physical_acceleration_state_rad_[kLeftFront];
        *left_back_joint_target_physical_acceleration_ =
            joint_target_physical_acceleration_state_rad_[kLeftBack];
        *right_back_joint_target_physical_acceleration_ =
            joint_target_physical_acceleration_state_rad_[kRightBack];
        *right_front_joint_target_physical_acceleration_ =
            joint_target_physical_acceleration_state_rad_[kRightFront];

        *lf_angle_error_ = std::isfinite(current_physical_angles[kLeftFront])
                             ? current_physical_angles[kLeftFront]
                                   - joint_target_physical_angle_state_rad_[kLeftFront]
                             : nan_;
        *lb_angle_error_ = std::isfinite(current_physical_angles[kLeftBack])
                             ? current_physical_angles[kLeftBack]
                                   - joint_target_physical_angle_state_rad_[kLeftBack]
                             : nan_;
        *rb_angle_error_ = std::isfinite(current_physical_angles[kRightBack])
                             ? current_physical_angles[kRightBack]
                                   - joint_target_physical_angle_state_rad_[kRightBack]
                             : nan_;
        *rf_angle_error_ = std::isfinite(current_physical_angles[kRightFront])
                             ? current_physical_angles[kRightFront]
                                   - joint_target_physical_angle_state_rad_[kRightFront]
                             : nan_;

        bool all_joint_angles_finite = true;
        double physical_angle_sum = 0.0;
        for (double current_physical_angle : current_physical_angles) {
            if (!std::isfinite(current_physical_angle)) {
                all_joint_angles_finite = false;
                break;
            }
            physical_angle_sum += current_physical_angle;
        }

        *processed_encoder_angle_ = all_joint_angles_finite ? rad_to_deg_ * physical_angle_sum
                                                                  / static_cast<double>(kJointCount)
                                                            : nan_;
    }

    void publish_nan_joint_targets() {
        reset_attitude_correction_state_();

        *left_front_joint_target_angle_ = nan_;
        *left_back_joint_target_angle_ = nan_;
        *right_back_joint_target_angle_ = nan_;
        *right_front_joint_target_angle_ = nan_;

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

        clear_suspension_output_interfaces_();
    }

private:
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<double> rotary_knob_;
    InputInterface<double> update_rate_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();
    double last_rotary_knob_ = 0.0;

    InputInterface<double> gimbal_yaw_angle_, gimbal_yaw_angle_error_;
    OutputInterface<double> chassis_angle_, chassis_control_angle_;

    OutputInterface<rmcs_msgs::ChassisMode> mode_;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;

    bool spinning_forward_ = true;
    bool apply_symmetric_target = true;
    pid::PidCalculator following_velocity_controller_;
    const double spin_ratio_;

    InputInterface<double> left_front_joint_angle_;
    InputInterface<double> left_back_joint_angle_;
    InputInterface<double> right_front_joint_angle_;
    InputInterface<double> right_back_joint_angle_;

    InputInterface<double> left_front_joint_physical_angle_;
    InputInterface<double> left_back_joint_physical_angle_;
    InputInterface<double> right_front_joint_physical_angle_;
    InputInterface<double> right_back_joint_physical_angle_;
    InputInterface<double> left_front_joint_physical_velocity_;
    InputInterface<double> left_back_joint_physical_velocity_;
    InputInterface<double> right_front_joint_physical_velocity_;
    InputInterface<double> right_back_joint_physical_velocity_;
    InputInterface<double> left_front_joint_torque_;
    InputInterface<double> left_back_joint_torque_;
    InputInterface<double> right_front_joint_torque_;
    InputInterface<double> right_back_joint_torque_;

    InputInterface<double> left_front_joint_encoder_angle_;
    InputInterface<double> left_back_joint_encoder_angle_;
    InputInterface<double> right_front_joint_encoder_angle_;
    InputInterface<double> right_back_joint_encoder_angle_;
    InputInterface<double> chassis_imu_pitch_;
    InputInterface<double> chassis_imu_roll_;
    InputInterface<double> chassis_imu_pitch_rate_;
    InputInterface<double> chassis_imu_roll_rate_;
    InputInterface<double> left_front_joint_eso_z2_;
    InputInterface<double> left_front_joint_eso_z3_;
    InputInterface<double> left_back_joint_eso_z2_;
    InputInterface<double> left_back_joint_eso_z3_;
    InputInterface<double> right_back_joint_eso_z2_;
    InputInterface<double> right_back_joint_eso_z3_;
    InputInterface<double> right_front_joint_eso_z2_;
    InputInterface<double> right_front_joint_eso_z3_;

    OutputInterface<double> scope_motor_control_torque;

    OutputInterface<double> lf_angle_error_;
    OutputInterface<double> lb_angle_error_;
    OutputInterface<double> rf_angle_error_;
    OutputInterface<double> rb_angle_error_;

    OutputInterface<double> left_front_joint_target_angle_;
    OutputInterface<double> left_back_joint_target_angle_;
    OutputInterface<double> right_back_joint_target_angle_;
    OutputInterface<double> right_front_joint_target_angle_;

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
    OutputInterface<bool> left_front_joint_suspension_mode_;
    OutputInterface<bool> left_back_joint_suspension_mode_;
    OutputInterface<bool> right_back_joint_suspension_mode_;
    OutputInterface<bool> right_front_joint_suspension_mode_;
    OutputInterface<double> left_front_joint_suspension_torque_;
    OutputInterface<double> left_back_joint_suspension_torque_;
    OutputInterface<double> right_back_joint_suspension_torque_;
    OutputInterface<double> right_front_joint_suspension_torque_;

    OutputInterface<double> processed_encoder_angle_;

    double min_angle_;
    double max_angle_;
    double left_front_joint_offset_;
    double left_back_joint_offset_;
    double right_front_joint_offset_;
    double right_back_joint_offset_;
    JointFeedbackSource joint_feedback_source_ = JointFeedbackSource::kLegacyEncoderAngle;

    double current_target_angle_;
    double lf_current_target_angle_, lb_current_target_angle_, rb_current_target_angle_,
        rf_current_target_angle_;

    bool joint_target_active_ = false;
    std::array<double, kJointCount> requested_target_physical_angles_rad_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> current_target_physical_angles_rad_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> joint_target_angle_state_rad_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> joint_target_physical_angle_state_rad_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> joint_target_physical_velocity_state_rad_ = {
        0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> joint_target_physical_acceleration_state_rad_ = {
        0.0, 0.0, 0.0, 0.0};
    std::array<LegControlState, kJointCount> leg_control_states_{};
    std::array<LegCommand, kJointCount> leg_commands_{};
    double target_physical_velocity_limit_;
    double target_physical_acceleration_limit_;
    bool active_suspension_enable_;
    double active_suspension_mass_;
    double active_suspension_rod_length_;
    double active_suspension_Kz_;
    double active_suspension_Kp_;
    double active_suspension_pitch_ki_;
    double active_suspension_Dp_;
    double active_suspension_Kr_;
    double active_suspension_roll_ki_;
    double active_suspension_Dr_;
    double active_suspension_D_leg_;
    double active_suspension_com_height_;
    double active_suspension_wheel_base_half_x_;
    double active_suspension_wheel_base_half_y_;
    double active_suspension_gravity_comp_gain_;
    double active_suspension_control_acceleration_limit_;
    double active_suspension_preload_angle_;
    double active_suspension_entry_offset_;
    double active_suspension_ride_height_offset_;
    double active_suspension_hold_travel_;
    double active_suspension_activation_velocity_threshold_;
    double active_suspension_target_physical_velocity_limit_;
    double active_suspension_target_physical_acceleration_limit_;
    double active_suspension_torque_limit_;
    double active_suspension_pitch_angle_diff_limit_;
    double active_suspension_roll_angle_diff_limit_;
    double active_suspension_pid_integral_limit_;
    std::array<bool, kJointCount> joint_suspension_active_ = {false, false, false, false};
    AttitudePidAxis pitch_attitude_pid_;
    AttitudePidAxis roll_attitude_pid_;
    double chassis_imu_pitch_offset_ = 0.0;
    double chassis_imu_roll_offset_ = 0.0;
    double chassis_imu_calibration_wait_time_;
    double chassis_imu_calibration_sample_time_;
    double chassis_imu_calibration_hold_elapsed_ = 0.0;
    size_t chassis_imu_calibration_sample_count_ = 0;
    double chassis_imu_pitch_sum_ = 0.0;
    double chassis_imu_roll_sum_ = 0.0;
    bool chassis_imu_calibration_completed_for_window_ = false;
    Eigen::Vector2d control_acceleration_estimate_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d last_control_translational_velocity_ = Eigen::Vector2d::Zero();
    bool last_control_translational_velocity_valid_ = false;

    static constexpr double default_dt_ = 1e-3;
    static constexpr double joint_zero_physical_angle_rad_ = 1.090830782496456;

    static constexpr double pi_ = std::numbers::pi;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::DeformableChassis, rmcs_executor::Component)
