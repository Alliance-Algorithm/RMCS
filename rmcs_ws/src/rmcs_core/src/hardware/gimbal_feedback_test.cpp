#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <tuple>

#include <eigen3/Eigen/Dense>
#include <librmcs/board/c_board.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/msg/gimbal_can_sample.hpp>
#include <rmcs_msgs/switch.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/gimbal_dbus.hpp"
#include "hardware/joint2_encoder_velocity.hpp"
#include "hardware/joint2_lift_pid.hpp"
#include "hardware/joint_motor_angle.hpp"
#include "hardware/small_yaw_damping.hpp"
#include "hardware/small_yaw_identification.hpp"
#include "hardware/small_yaw_position_hold.hpp"
#include "hardware/small_yaw_robust_torque_slew.hpp"
#include "controller/gimbal/velocity_reference.hpp"
#include "controller/robust/robust_calculator.hpp"

namespace rmcs_core::hardware {

class GimbalFeedbackTest;

class GimbalJoint2LiftCommandTest final : public rmcs_executor::Component {
public:
    explicit GimbalJoint2LiftCommandTest(GimbalFeedbackTest& hardware);
    void update() override;

private:
    GimbalFeedbackTest& hardware_;
    Joint2LiftPid controller_;
    Joint2LiftSafetyGate safety_gate_;
    bool controller_enabled_{false};
    bool was_permitted_{false};
    int64_t previous_update_ns_{0}, previous_send_ns_{0};
    double last_sent_torque_{0.0};
    InputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    InputInterface<int64_t> remote_last_received_ns_;
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<bool> joint2_motor_online_, joint2_encoder_online_;
    InputInterface<double> joint2_encoder_angle_, joint2_encoder_velocity_;
    OutputInterface<bool> permitted_, active_, target_valid_;
    OutputInterface<double> target_angle_, position_error_, requested_torque_, applied_torque_;
    OutputInterface<std::string> block_reason_;
};

class GimbalSmallYawDampingCommandTest final : public rmcs_executor::Component {
public:
    explicit GimbalSmallYawDampingCommandTest(GimbalFeedbackTest& hardware);
    void update() override;

private:
    GimbalFeedbackTest& hardware_;
    SmallYawDamping damping_;
    SmallYawPositionHold position_hold_;
    SmallYawIdentification identification_;
    controller::robust::RobustVelocityCalculator robust_controller_;
    controller::gimbal::VelocityReference robust_reference_;
    SmallYawRobustTorqueSlew robust_torque_slew_;
    SmallYawMotionEstimator motion_estimator_;
    bool double_down_confirmed_{false};
    bool stick_centered_after_safety_{false};
    int64_t previous_update_ns_{0}, previous_send_ns_{0};
    double last_sent_torque_{0.0};
    InputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    InputInterface<int64_t> remote_last_received_ns_;
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<bool> small_yaw_online_;
    InputInterface<bool> coordinated_active_;
    InputInterface<double> coordinated_velocity_, coordinated_acceleration_;
    InputInterface<double> small_yaw_angle_;
    InputInterface<double> joint2_position_error_;
    InputInterface<double> identification_joint2_angle_;
    InputInterface<bool> joint2_permitted_;
    OutputInterface<bool> permitted_, active_;
    OutputInterface<double> estimated_velocity_, requested_torque_, applied_torque_;
    OutputInterface<bool> hold_reference_valid_;
    OutputInterface<double> hold_target_angle_, hold_position_error_;
    OutputInterface<std::string> block_reason_;
    OutputInterface<std::string> identification_state_;
    OutputInterface<double> identification_elapsed_, identification_relative_angle_;
    OutputInterface<double> identification_joint2_angle_span_, identification_joint2_stable_elapsed_;
    OutputInterface<bool> identification_joint2_window_ready_;
    OutputInterface<bool> identification_complete_;
    OutputInterface<bool> robust_permitted_, robust_active_;
    OutputInterface<double> robust_target_velocity_, robust_target_acceleration_;
    OutputInterface<double> robust_velocity_, robust_torque_, robust_torque_applied_;
    OutputInterface<double> robust_integral_torque_;
    OutputInterface<std::string> robust_block_reason_;
};

// The feedback path stays separate from the optional small-yaw damping command path.
class GimbalFeedbackTest final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::board::CBoard::Callback {
    friend class GimbalSmallYawDampingCommandTest;
    friend class GimbalJoint2LiftCommandTest;

    // The joint2 absolute encoder is connected to a separate CBoard on this robot.
    // Keep its callback separate so a frame from another board cannot refresh it.
    class Joint2EncoderReceiver final : public librmcs::board::CBoard::Callback {
    public:
        explicit Joint2EncoderReceiver(GimbalFeedbackTest& hardware) : hardware_(hardware) {}

    private:
        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (can == Spec::kCans.kCan1 && data.can_id == 0x200 && !data.is_fdcan
                && !data.is_extended_can_id && !data.is_remote_transmission
                && data.can_data.size() == 8)
                hardware_.store_joint2_encoder_status(data);
        }

        GimbalFeedbackTest& hardware_;
    };

public:
    GimbalFeedbackTest()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , update_rate_(parameter_or<double>("update_rate", 1000.0))
        , bmi088_(update_rate_, 0.2, 0.0)
        , encoder_receiver_(*this) {
        can_sample_publisher_ = create_publisher<rmcs_msgs::msg::GimbalCanSample>(
            "/gimbal/feedback/small_yaw_can_sample", rclcpp::QoS{20}.reliable());
        const auto serial               = get_parameter("board_serial_arm_board").as_string();
        const auto encoder_serial = parameter_or<std::string>("board_serial_encoder_board", "");
        separate_joint2_encoder_board_ = !encoder_serial.empty();
        const auto zero_point           = get_parameter("image_pitch_zero_point").as_int();
        const auto small_yaw_zero_point = get_parameter("joint1_zero_point").as_int();
        constexpr std::array<int, 5> joint_angle_zero_point_defaults{0, 29072, 33493, 60137, 12793};
        const auto timeout_ms           = parameter_or<double>("feedback_timeout_ms", 100.0);
        const auto remote_timeout_ms    = parameter_or<double>("remote_timeout_ms", 200.0);
        const auto small_yaw_current_bias_amp =
            parameter_or<double>("small_yaw_current_bias_amp", 0.0);
        const auto small_yaw_torque_scale_nm_per_amp =
            parameter_or<double>("small_yaw_torque_scale_nm_per_amp", 3.6);
        const auto small_yaw_torque_bias_nm =
            parameter_or<double>("small_yaw_torque_bias_nm", 0.0);
        damping_parameters_.enabled = parameter_or<bool>("small_yaw_damping_enabled", false);
        damping_parameters_.torque_nm = parameter_or<double>("small_yaw_damping_torque_nm", 2.5);
        damping_parameters_.start_velocity_rad_s =
            parameter_or<double>("small_yaw_damping_start_velocity_rad_s", 0.006);
        damping_parameters_.stop_velocity_rad_s =
            parameter_or<double>("small_yaw_damping_stop_velocity_rad_s", 0.002);
        damping_parameters_.torque_slew_nm_s =
            parameter_or<double>("small_yaw_damping_torque_slew_nm_s", 18.0);
        hold_parameters_.enabled = parameter_or<bool>("small_yaw_position_hold_enabled", false);
        hold_parameters_.position_gain_nm_rad =
            parameter_or<double>("small_yaw_position_hold_kp_nm_rad", 20.0);
        hold_parameters_.velocity_gain_nm_s_rad =
            parameter_or<double>("small_yaw_position_hold_kd_nm_s_rad", 2.5);
        hold_parameters_.torque_limit_nm = damping_parameters_.torque_nm;
        hold_parameters_.torque_slew_nm_s =
            parameter_or<double>("small_yaw_position_hold_torque_slew_nm_s", 75.0);
        hold_parameters_.position_deadband_rad =
            parameter_or<double>("small_yaw_position_hold_deadband_rad", 0.0002);
        hold_parameters_.sliding_enabled =
            parameter_or<bool>("small_yaw_position_hold_sliding_enabled", false);
        hold_parameters_.sliding_position_gain =
            parameter_or<double>("small_yaw_position_hold_sliding_position_gain", 4.0);
        hold_parameters_.sliding_velocity_limit_rad_s =
            parameter_or<double>("small_yaw_position_hold_sliding_velocity_limit_rad_s", 0.35);
        joint2_pid_parameters_.kp_nm_per_rad =
            parameter_or<double>("joint2_pid_kp_nm_per_rad", 20.0);
        joint2_pid_parameters_.ki_nm_per_rad_s =
            parameter_or<double>("joint2_pid_ki_nm_per_rad_s", 0.5);
        joint2_pid_parameters_.kd_nm_s_per_rad =
            parameter_or<double>("joint2_pid_kd_nm_s_per_rad", 3.0);
        joint2_pid_parameters_.torque_limit_nm =
            parameter_or<double>("joint2_pid_torque_limit_nm", 5.0);
        joint2_pid_parameters_.torque_slew_nm_s =
            parameter_or<double>("joint2_pid_torque_slew_nm_s", 8.0);
        joint2_pid_parameters_.torque_release_nm_s =
            parameter_or<double>("joint2_pid_torque_release_nm_s", 0.0);
        joint2_pid_parameters_.reference_speed_rad_s =
            parameter_or<double>("joint2_pid_reference_speed_rad_s", 0.12);
        joint2_pid_parameters_.integral_limit_rad_s =
            parameter_or<double>("joint2_pid_integral_limit_rad_s", 3.0);
        joint2_pid_parameters_.lift_angle_rad =
            parameter_or<double>("joint2_pid_lift_angle_deg", 30.0) * std::numbers::pi / 180.0;
        joint2_pid_parameters_.lower_limit_rad =
            parameter_or<double>("joint2_lower_limit_rad", -1.084);
        joint2_pid_parameters_.upper_limit_rad =
            parameter_or<double>("joint2_upper_limit_rad", 1.528);
        joint2_pid_parameters_.lift_direction =
            parameter_or<double>("joint2_lift_direction", 1.0);
        joint2_pid_enabled_ = parameter_or<bool>("joint2_pid_enabled", false);
        identification_parameters_.enabled =
            parameter_or<bool>("small_yaw_identification_enabled", false);
        identification_parameters_.command_sign =
            parameter_or<double>("small_yaw_identification_command_sign", 1.0);
        identification_parameters_.torque_limit_nm =
            parameter_or<double>("small_yaw_identification_torque_limit_nm", 6.0);
        identification_parameters_.pulse_duration_s =
            parameter_or<double>("small_yaw_identification_pulse_duration_s", 0.20);
        identification_parameters_.rest_duration_s =
            parameter_or<double>("small_yaw_identification_rest_duration_s", 0.7);
        identification_parameters_.settle_duration_s =
            parameter_or<double>("small_yaw_identification_settle_duration_s", 1.0);
        identification_parameters_.sine_torque_amplitude_nm =
            parameter_or<double>("small_yaw_identification_sine_torque_amplitude_nm", 6.0);
        identification_parameters_.sine_frequency_hz =
            parameter_or<double>("small_yaw_identification_sine_frequency_hz", 0.15);
        identification_parameters_.sine_frequency_end_hz =
            parameter_or<double>("small_yaw_identification_sine_frequency_end_hz", 0.60);
        identification_parameters_.sine_duration_s =
            parameter_or<double>("small_yaw_identification_sine_duration_s", 16.0);
        identification_parameters_.constant_torque_nm =
            parameter_or<double>("small_yaw_identification_constant_torque_nm", 1.0);
        identification_parameters_.constant_duration_s =
            parameter_or<double>("small_yaw_identification_constant_duration_s", 2.0);
        identification_parameters_.constant_rest_duration_s =
            parameter_or<double>("small_yaw_identification_constant_rest_duration_s", 0.5);
        identification_parameters_.linear_ramp_peak_nm =
            parameter_or<double>("small_yaw_identification_linear_ramp_peak_nm", 5.0);
        identification_parameters_.linear_ramp_duration_s =
            parameter_or<double>("small_yaw_identification_linear_ramp_duration_s", 6.0);
        identification_parameters_.linear_ramp_group_count =
            static_cast<int>(parameter_or<int64_t>("small_yaw_identification_linear_ramp_group_count", 1));
        identification_parameters_.breakaway_staircase_enabled =
            parameter_or<bool>("small_yaw_identification_breakaway_staircase_enabled", false);
        identification_parameters_.breakaway_pulse_duration_s =
            parameter_or<double>("small_yaw_identification_breakaway_pulse_duration_s", 0.5);
        identification_parameters_.breakaway_rest_duration_s =
            parameter_or<double>("small_yaw_identification_breakaway_rest_duration_s", 2.0);
        identification_parameters_.breakaway_group_count =
            static_cast<int>(parameter_or<int64_t>("small_yaw_identification_breakaway_group_count", 5));
        identification_parameters_.max_velocity_rad_s =
            parameter_or<double>("small_yaw_identification_max_velocity_rad_s", 1.0);
        identification_parameters_.joint2_position_tolerance_rad =
            parameter_or<double>("small_yaw_identification_joint2_position_tolerance_rad", 10.0 * std::numbers::pi / 180.0);
        identification_parameters_.joint2_stability_window_s =
            parameter_or<double>("small_yaw_identification_joint2_stability_window_s", 0.25);
        identification_parameters_.joint2_stable_duration_s =
            parameter_or<double>("small_yaw_identification_joint2_stable_duration_s", 1.0);
        small_yaw_robust_parameters_.enabled =
            parameter_or<bool>("small_yaw_robust_enabled", false);
        small_yaw_robust_parameters_.external_reference =
            parameter_or<bool>("small_yaw_robust_external_reference", false);
        small_yaw_robust_parameters_.max_velocity_rad_s =
            parameter_or<double>("small_yaw_robust_max_velocity_rad_s", 0.35);
        small_yaw_robust_parameters_.max_acceleration_rad_s2 =
            parameter_or<double>("small_yaw_robust_max_acceleration_rad_s2", 2.0);
        small_yaw_robust_parameters_.joystick_velocity_direction =
            parameter_or<double>("small_yaw_robust_joystick_velocity_direction", 1.0);
        small_yaw_robust_parameters_.torque_limit_nm =
            parameter_or<double>("small_yaw_robust_torque_limit_nm", 2.5);
        small_yaw_robust_parameters_.torque_slew_nm_s =
            parameter_or<double>("small_yaw_robust_torque_slew_nm_s", 18.0);
        controller::robust::RobustVelocityParameters robust_model;
        // Latest five-run small-yaw identification model (current-derived torque).
        robust_model.inertia = parameter_or<double>("small_yaw_robust_inertia", 0.311472);
        robust_model.viscous_friction =
            parameter_or<double>("small_yaw_robust_viscous_friction", 0.725324);
        robust_model.coulomb_friction = parameter_or<double>("small_yaw_robust_coulomb_friction", 2.63482);
        robust_model.static_friction_compensation =
            parameter_or<double>("small_yaw_robust_static_friction_compensation", 4.0);
        robust_model.torque_bias = parameter_or<double>("small_yaw_robust_torque_bias", -0.122053);
        robust_model.integral_gain =
            parameter_or<double>("small_yaw_robust_integral_gain_nm_per_rad", 0.0);
        robust_model.integral_torque_limit =
            parameter_or<double>("small_yaw_robust_integral_torque_limit_nm", 0.0);
        robust_model.tracking_gain = parameter_or<double>("small_yaw_robust_tracking_gain", 2.0);
        robust_model.robust_gain = parameter_or<double>("small_yaw_robust_gain", 1.0);
        robust_model.boundary_width = parameter_or<double>("small_yaw_robust_boundary_width", 0.1);
        robust_model.friction_velocity_scale =
            parameter_or<double>("small_yaw_robust_friction_velocity_scale", 0.05);
        robust_model.standstill_velocity_threshold =
            parameter_or<double>("small_yaw_robust_standstill_velocity_threshold", 0.02);
        robust_model.output_min = -small_yaw_robust_parameters_.torque_limit_nm;
        robust_model.output_max = small_yaw_robust_parameters_.torque_limit_nm;
        robust_model.auxiliary_law = controller::robust::AuxiliaryLaw::Smooth;
        small_yaw_robust_parameters_.model = robust_model;
        if (hold_parameters_.sliding_enabled) {
            hold_parameters_.sliding_model = robust_model;
            hold_parameters_.torque_limit_nm = small_yaw_robust_parameters_.torque_limit_nm;
            hold_parameters_.torque_slew_nm_s = small_yaw_robust_parameters_.torque_slew_nm_s;
        }
        // Position hold is the stiff submode of the legacy damping component,
        // so damping+hold together count as one owner of the CAN output.
        const int enabled_small_yaw_modes = static_cast<int>(
            damping_parameters_.enabled || hold_parameters_.enabled)
            + static_cast<int>(identification_parameters_.enabled)
            + static_cast<int>(small_yaw_robust_parameters_.enabled);
        if (enabled_small_yaw_modes > 1)
            throw std::invalid_argument("small yaw damping, hold, identification, and robust modes are mutually exclusive");
        if (small_yaw_robust_parameters_.external_reference && !small_yaw_robust_parameters_.enabled)
            throw std::invalid_argument("external small yaw reference requires the robust mode");
        if (identification_parameters_.enabled && !joint2_pid_enabled_)
            throw std::invalid_argument("small yaw identification requires joint2 support and excludes small yaw hold/damping");
        joint2_encoder_zero_point_ = parameter_or<int64_t>("joint2_encoder_zero_point", 39433);
        if (serial.empty())
            throw std::invalid_argument("board_serial_arm_board must select the arm board");
        if (separate_joint2_encoder_board_ && encoder_serial == serial)
            throw std::invalid_argument("encoder board must differ from the arm board");
        if (zero_point < 0 || zero_point > 65535)
            throw std::invalid_argument("image_pitch_zero_point must be in [0, 65535]");
        if (small_yaw_zero_point < 0 || small_yaw_zero_point > 65535)
            throw std::invalid_argument("joint1_zero_point must be in [0, 65535]");
        if (!std::isfinite(update_rate_) || update_rate_ <= 0.0)
            throw std::invalid_argument("update_rate must be finite and positive");
        if (!Joint2LiftPid{joint2_pid_parameters_}.valid_parameters())
            throw std::invalid_argument("joint2 PID parameters are invalid");
        if (!small_yaw_robust_parameters_.valid())
            throw std::invalid_argument("small yaw robust parameters are invalid");
        if (joint2_encoder_zero_point_ < 0 || joint2_encoder_zero_point_ > 65535)
            throw std::invalid_argument("joint2_encoder_zero_point must be in [0, 65535]");
        if (!std::isfinite(timeout_ms) || timeout_ms <= 0.0 || timeout_ms > 60'000.0)
            throw std::invalid_argument("feedback_timeout_ms must be in (0, 60000]");
        if (!std::isfinite(remote_timeout_ms) || remote_timeout_ms <= 0.0 || remote_timeout_ms > 60'000.0)
            throw std::invalid_argument("remote_timeout_ms must be in (0, 60000]");
        if (!std::isfinite(small_yaw_current_bias_amp)
            || !std::isfinite(small_yaw_torque_scale_nm_per_amp)
            || small_yaw_torque_scale_nm_per_amp <= 0.0
            || !std::isfinite(small_yaw_torque_bias_nm))
            throw std::invalid_argument("small-yaw torque calibration parameters are invalid");
        pitch_feedback_.zero_point     = static_cast<int>(zero_point);
        small_yaw_feedback_.zero_point = static_cast<int>(small_yaw_zero_point);
        pitch_feedback_.torque_constant_nm_a = 0.7;
        small_yaw_feedback_.torque_constant_nm_a = 3.6;
        small_yaw_feedback_.current_bias_amp = small_yaw_current_bias_amp;
        small_yaw_feedback_.torque_constant_nm_a = small_yaw_torque_scale_nm_per_amp;
        small_yaw_feedback_.torque_bias_nm = small_yaw_torque_bias_nm;
        constexpr std::array<uint32_t, 5> joint_angle_can_ids{0x142, 0x143, 0x144, 0x145, 0x141};
        for (size_t i = 0; i < joint_angle_feedback_.size(); ++i) {
            const int joint = static_cast<int>(i) + 2;
            const auto zero = parameter_or<int64_t>(
                "joint" + std::to_string(joint) + "_zero_point",
                joint_angle_zero_point_defaults[i]);
            if (zero < 0 || zero > 65535)
                throw std::invalid_argument(
                    "joint" + std::to_string(joint) + "_zero_point must be in [0, 65535]");
            auto& feedback = joint_angle_feedback_[i];
            feedback.can_id = joint_angle_can_ids[i];
            feedback.can2 = joint >= 4;
            feedback.zero_point = static_cast<int>(zero);
            feedback.direction = joint == 2 || joint == 6 ? -1.0 : 1.0;
            feedback.gear_ratio = joint == 2 ? 42.0 : 1.0;
            if (joint == 2) {
                // CAN1 / 0x142 is MF7015V210T with the same 42:1 gearbox
                // used by the arm-board configuration. Publish motor-reported
                // current and its output-shaft torque estimate for diagnosis.
                feedback.current_amp_per_count = 33.0 / 4096.0;
                feedback.motor_torque_constant_nm_per_amp = 0.12;
            }
            register_joint_angle_feedback(joint, feedback);
        }
        feedback_timeout_ns_           = static_cast<int64_t>(timeout_ms * 1'000'000.0);
        damping_remote_timeout_ns_ = static_cast<int64_t>(remote_timeout_ms * 1'000'000.0);

        register_motor_feedback(
            "/arm/image_pitch/motor", "/gimbal/feedback/motor_online", pitch_feedback_);
        register_motor_feedback(
            "/arm/joint_1/motor", "/gimbal/feedback/small_yaw_online", small_yaw_feedback_, true);
        register_output("/gimbal/feedback/imu_online", imu_online_, false);
        register_output("/arm/joint_2/encoder/angle", joint2_encoder_angle_, kNaN);
        register_output("/arm/joint_2/encoder/velocity", joint2_encoder_velocity_, kNaN);
        register_output("/arm/joint_2/encoder/raw_angle", joint2_encoder_raw_angle_, -1);
        register_output("/gimbal/feedback/joint_2_encoder_online", joint2_encoder_online_, false);
        register_output("/gimbal/feedback/can1_motor_id_scan", can1_motor_id_scan_,
                        std::string{"scanning"});
        register_output("yaw_imu_velocity", yaw_imu_velocity_, kNaN);
        register_output("yaw_imu_angle", yaw_imu_angle_, kNaN);
        register_output("pitch_imu_velocity", pitch_imu_velocity_, kNaN);
        register_output("pitch_imu_angle", pitch_imu_angle_, kNaN);
        register_output("roll_imu_velocity", roll_imu_velocity_, kNaN);
        register_output("roll_imu_angle", roll_imu_angle_, kNaN);
        register_output("/remote/switch/left", switch_left_, rmcs_msgs::Switch::UNKNOWN);
        register_output("/remote/switch/right", switch_right_, rmcs_msgs::Switch::UNKNOWN);
        register_output("/remote/joystick/right", joystick_right_, Eigen::Vector2d::Zero());
        register_output("/remote/joystick/right/x", joystick_right_x_, 0.0);
        register_output("/remote/joystick/right/y", joystick_right_y_, 0.0);
        register_output("/remote/joystick/left", joystick_left_, Eigen::Vector2d::Zero());
        register_output("/remote/joystick/left/x", joystick_left_x_, 0.0);
        register_output("/remote/joystick/left/y", joystick_left_y_, 0.0);
        register_output("/remote/last_received_monotonic_ns", remote_last_received_ns_, int64_t{0});
        damping_command_ = create_partner_component<GimbalSmallYawDampingCommandTest>(
            "gimbal_small_yaw_damping_command_test", *this);
        joint2_lift_command_ = create_partner_component<GimbalJoint2LiftCommandTest>(
            "gimbal_joint2_lift_command_test", *this);

        // Match the existing arm board IMU axes.
        bmi088_.set_coordinate_mapping(
            [](double x, double y, double z) { return std::make_tuple(-x, -y, z); });

        // CBoard can invoke callbacks during construction; initialize all receive state first.
        board_ = std::make_unique<librmcs::board::CBoard>(
            static_cast<librmcs::board::CBoard::Callback&>(*this), serial,
            librmcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true));
        if (separate_joint2_encoder_board_)
            encoder_board_ = std::make_unique<librmcs::board::CBoard>(
                encoder_receiver_, encoder_serial,
                librmcs::board::AdvancedOptions{}.set_dangerously_skip_version_checks(true));
    }

    ~GimbalFeedbackTest() override {
        if (board_ && (damping_parameters_.enabled || identification_parameters_.enabled
                       || small_yaw_robust_parameters_.enabled))
            send_small_yaw_torque(0.0);
        if (board_ && joint2_pid_enabled_)
            send_joint2_torque(0.0);
        // Stop callbacks before receive state is destroyed.
        encoder_board_.reset();
        board_.reset();
    }

    void update() override {
        const auto now = monotonic_now_ns();
        update_motor(pitch_feedback_, now);
        update_motor(small_yaw_feedback_, now);
        for (auto& feedback : joint_angle_feedback_)
            update_joint_angle_feedback(feedback, now);
        update_joint2_encoder(now);
        update_can1_motor_id_scan(now);
        update_imu(now);
        update_remote(now);
        publish_small_yaw_can_sample(now);

        if (now - last_query_ns_ >= kQueryIntervalNs) {
            // Pitch receives only read queries; joint1 damping is sent by the partner component.
            static constexpr std::array<std::byte, 8> query{std::byte{0x9C}};
            // Probe one additional standard LK ID per poll cycle. This adds only 100 read-only
            // frames/s and identifies a small-yaw motor whose configured CAN ID differs from 0x141.
            const uint32_t scan_can_id =
                kCanIdScanFirst + static_cast<uint32_t>(can1_motor_id_scan_index_);
            board_->start_transmit()
                .can_transmit(Spec::kCans.kCan1, {.can_id = kImagePitchCanId, .can_data = query})
                .can_transmit(Spec::kCans.kCan1, {.can_id = kSmallYawCanId, .can_data = query})
                .can_transmit(Spec::kCans.kCan1, {.can_id = 0x142, .can_data = query})
                .can_transmit(Spec::kCans.kCan1, {.can_id = 0x143, .can_data = query})
                .can_transmit(Spec::kCans.kCan2, {.can_id = 0x144, .can_data = query})
                .can_transmit(Spec::kCans.kCan2, {.can_id = 0x145, .can_data = query})
                .can_transmit(Spec::kCans.kCan2, {.can_id = 0x141, .can_data = query})
                .can_transmit(Spec::kCans.kCan1, {.can_id = scan_can_id, .can_data = query});
            can1_motor_id_scan_index_ =
                (can1_motor_id_scan_index_ + 1) % kCanIdScanCount;
            last_query_ns_ = now;
        }
    }

private:
    struct SmallYawCommandSnapshot {
        int64_t sent_ns{0};
        double torque_nm{0.0};
    };

    double send_small_yaw_torque(double torque, double torque_limit_nm = 6.0) {
        const auto packet = encode_small_yaw_damping_torque(torque, torque_limit_nm);
        board_->start_transmit().can_transmit(
            Spec::kCans.kCan1, {.can_id = kSmallYawCanId, .can_data = packet});
        constexpr double nm_per_count = 3.6 * 66.0 / 4096.0;
        const uint16_t encoded_current = std::to_integer<uint16_t>(packet[4])
            | (std::to_integer<uint16_t>(packet[5]) << 8U);
        const double quantized_torque = -static_cast<double>(std::bit_cast<int16_t>(encoded_current))
            * nm_per_count;
        small_yaw_command_snapshot_.store(
            SmallYawCommandSnapshot{.sent_ns = monotonic_now_ns(), .torque_nm = quantized_torque},
            std::memory_order_release);
        return quantized_torque;
    }

    void send_joint2_torque(double torque) {
        // Match LKMotorConfig{MF7015V210T}.reverse().set_gear_ratio(42.0).
        // This emits the motor's A1 current/torque command on CAN1 / 0x142.
        constexpr double nm_per_count = 42.0 * 0.12 * 33.0 / 4096.0;
        constexpr double hard_limit_nm = 36.0;
        const double bounded_torque = std::clamp(torque, -hard_limit_nm, hard_limit_nm);
        const auto current = static_cast<int16_t>(std::clamp(
            std::lround(-bounded_torque / nm_per_count),
            static_cast<long>(std::numeric_limits<int16_t>::min()),
            static_cast<long>(std::numeric_limits<int16_t>::max())));
        const auto current_bytes = std::bit_cast<std::array<uint8_t, 2>>(current);
        const std::array<std::byte, 8> packet{
            std::byte{0xA1}, std::byte{0}, std::byte{0}, std::byte{0},
            static_cast<std::byte>(current_bytes[0]), static_cast<std::byte>(current_bytes[1]),
            std::byte{0}, std::byte{0}};
        board_->start_transmit().can_transmit(
            Spec::kCans.kCan1, {.can_id = 0x142, .can_data = packet});
    }

    struct MotorFeedback {
        int zero_point{0};
        double torque_constant_nm_a{0.0};
        double current_bias_amp{0.0};
        double torque_bias_nm{0.0};
        std::atomic<uint64_t> packet{0};
        std::atomic<int64_t> received_ns{0};
        OutputInterface<int> raw_angle;
        OutputInterface<double> angle, velocity, current, torque, torque_estimate;
        OutputInterface<double> feedback_age_ms;
        OutputInterface<int64_t> last_received_ns;
        OutputInterface<bool> online;
    };

    struct JointAngleFeedback {
        uint32_t can_id{0};
        int zero_point{0};
        double direction{1.0};
        double gear_ratio{1.0};
        double current_amp_per_count{0.0};
        double motor_torque_constant_nm_per_amp{0.0};
        bool can2{false};
        std::atomic<uint64_t> packet{0};
        std::atomic<int64_t> received_ns{0};
        MultiTurnEncoderAngle multi_turn_angle;
        OutputInterface<double> angle, current, torque;
        OutputInterface<bool> online;
    };

    struct RemoteSnapshot {
        int64_t received_ns{0};
        uint8_t switches{0};
        double joystick_x{0.0};
        double joystick_y{0.0};
        double joystick_left_x{0.0};
        double joystick_left_y{0.0};
    };

    struct Joint2EncoderSnapshot {
        int64_t received_ns{0};
        uint32_t raw_angle{0};
    };

    struct SmallYawCanSnapshot {
        int64_t received_ns{0};
        uint16_t raw_angle{0};
        int16_t current_count{0};
        bool valid{false};
    };

    void register_motor_feedback(
        const std::string& prefix, const std::string& online_name, MotorFeedback& feedback,
        bool expose_torque_estimate = false) {
        register_output(prefix + "/raw_angle", feedback.raw_angle, -1);
        register_output(prefix + "/angle", feedback.angle, kNaN);
        register_output(prefix + "/velocity", feedback.velocity, kNaN);
        register_output(prefix + "/current", feedback.current, kNaN);
        register_output(prefix + "/torque", feedback.torque, kNaN);
        if (expose_torque_estimate)
            register_output(prefix + "/torque_estimate", feedback.torque_estimate, kNaN);
        register_output(prefix + "/feedback_age_ms", feedback.feedback_age_ms, kNaN);
        register_output(prefix + "/last_received_monotonic_ns", feedback.last_received_ns, int64_t{0});
        register_output(online_name, feedback.online, false);
    }

    void register_joint_angle_feedback(int joint, JointAngleFeedback& feedback) {
        const auto prefix = "/arm/joint_" + std::to_string(joint) + "/motor";
        register_output(prefix + "/angle", feedback.angle, kNaN);
        if (joint == 2) {
            register_output(prefix + "/current", feedback.current, kNaN);
            register_output(prefix + "/torque", feedback.torque, kNaN);
        }
        register_output("/gimbal/feedback/joint_" + std::to_string(joint) + "_online",
                        feedback.online, false);
    }

    template <typename T>
    T parameter_or(const std::string& name, const T& default_value) {
        if (has_parameter(name))
            return get_parameter(name).get_value<T>();
        return declare_parameter<T>(name, default_value);
    }

    static int64_t monotonic_now_ns() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    static bool fresh(int64_t received_at, int64_t now, int64_t timeout) {
        // A concurrently arriving callback may be newer than this update's sampled time.
        return received_at != 0 && (received_at >= now || now - received_at <= timeout);
    }

    void publish_small_yaw_can_sample(int64_t now) {
        const auto snapshot = small_yaw_can_snapshot_.load(std::memory_order_acquire);
        const auto received_at = snapshot.received_ns;
        if (received_at == 0 || received_at == last_published_small_yaw_sample_ns_)
            return;

        const bool online = snapshot.valid && fresh(received_at, now, feedback_timeout_ns_);
        rmcs_msgs::msg::GimbalCanSample message;
        message.can_receive_monotonic_ns = received_at;
        message.can_channel = 1;
        message.can_id = kSmallYawCanId;
        message.raw_angle = snapshot.raw_angle;
        message.current_amp = online
            ? -static_cast<double>(snapshot.current_count) * (66.0 / 4096.0)
                + small_yaw_feedback_.current_bias_amp
            : std::numeric_limits<double>::quiet_NaN();
        message.torque_nm = online
            ? message.current_amp * small_yaw_feedback_.torque_constant_nm_a
                + small_yaw_feedback_.torque_bias_nm
            : std::numeric_limits<double>::quiet_NaN();
        message.online = online;
        const auto command = small_yaw_command_snapshot_.load(std::memory_order_acquire);
        message.command_monotonic_ns = command.sent_ns;
        message.command_torque_nm = command.torque_nm;
        can_sample_publisher_->publish(message);
        last_published_small_yaw_sample_ns_ = received_at;
    }

    void update_motor(MotorFeedback& feedback, int64_t now) {
        const auto received_at = feedback.received_ns.load(std::memory_order_acquire);
        *feedback.last_received_ns = received_at;
        *feedback.feedback_age_ms = received_at > 0
            ? static_cast<double>(std::max(int64_t{0}, now - received_at)) * 1e-6 : kNaN;
        *feedback.online =
            fresh(received_at, now, feedback_timeout_ns_);
        if (!*feedback.online) {
            *feedback.raw_angle = -1;
            *feedback.angle = *feedback.velocity = *feedback.current = *feedback.torque = kNaN;
            if (feedback.torque_estimate.active())
                *feedback.torque_estimate = kNaN;
            return;
        }

        // Match the existing reversed LKMotor configurations for image pitch and joint 1.
        const auto bytes =
            std::bit_cast<std::array<uint8_t, 8>>(feedback.packet.load(std::memory_order_relaxed));
        const int encoder     = bytes[6] | (static_cast<int>(bytes[7]) << 8);
        const auto current_bits = static_cast<uint16_t>(bytes[2] | (uint16_t{bytes[3]} << 8));
        const auto current = std::bit_cast<int16_t>(current_bits);
        const auto speed_bits = static_cast<uint16_t>(bytes[4] | (uint16_t{bytes[5]} << 8));
        const auto speed      = std::bit_cast<int16_t>(speed_bits);
        int offset            = (encoder - feedback.zero_point + 65536) % 65536;
        if (offset > 32768)
            offset -= 65536;
        *feedback.raw_angle = encoder;
        *feedback.angle     = -static_cast<double>(offset) * (2.0 * std::numbers::pi / 65536.0);
        *feedback.velocity  = -static_cast<double>(speed) * (std::numbers::pi / 180.0);
        // Use each configured LK motor's output-shaft torque constant. This is
        // the motor-reported current, separate from the command sent over CAN.
        *feedback.current = -static_cast<double>(current) * (66.0 / 4096.0)
                          + feedback.current_bias_amp;
        const double torque_estimate = *feedback.current * feedback.torque_constant_nm_a
                                     + feedback.torque_bias_nm;
        *feedback.torque = torque_estimate;  // Backwards-compatible alias.
        if (feedback.torque_estimate.active())
            *feedback.torque_estimate = torque_estimate;
    }

    void update_joint2_encoder(int64_t now) {
        const auto snapshot = joint2_encoder_snapshot_.load(std::memory_order_acquire);
        const auto received_at = snapshot.received_ns;
        *joint2_encoder_online_ = fresh(received_at, now, feedback_timeout_ns_);
        if (!*joint2_encoder_online_) {
            *joint2_encoder_raw_angle_ = -1;
            *joint2_encoder_angle_ = *joint2_encoder_velocity_ = kNaN;
            joint2_encoder_velocity_estimator_.reset();
            return;
        }

        const int raw = static_cast<int>(snapshot.raw_angle);
        *joint2_encoder_raw_angle_ = raw;
        int offset = raw - static_cast<int>(joint2_encoder_zero_point_);
        if (offset > 32768)
            offset -= 65536;
        else if (offset < -32768)
            offset += 65536;
        *joint2_encoder_angle_ = static_cast<double>(offset) * (2.0 * std::numbers::pi / 65536.0);

        *joint2_encoder_velocity_ = joint2_encoder_velocity_estimator_.update(raw, received_at);
    }

    void update_joint_angle_feedback(JointAngleFeedback& feedback, int64_t now) {
        *feedback.online =
            fresh(feedback.received_ns.load(std::memory_order_acquire), now, feedback_timeout_ns_);
        if (!*feedback.online) {
            *feedback.angle = kNaN;
            if (feedback.current_amp_per_count > 0.0)
                *feedback.current = *feedback.torque = kNaN;
            return;
        }

        const auto bytes =
            std::bit_cast<std::array<uint8_t, 8>>(feedback.packet.load(std::memory_order_relaxed));
        if (feedback.current_amp_per_count > 0.0) {
            const auto current_bits = static_cast<uint16_t>(bytes[2] | (uint16_t{bytes[3]} << 8));
            const auto current_count = std::bit_cast<int16_t>(current_bits);
            // Match the reversed joint2 axis used by LKMotorConfig. Torque is
            // estimated from reported q-axis current, not measured by a sensor.
            *feedback.current = -static_cast<double>(current_count)
                              * feedback.current_amp_per_count;
            *feedback.torque = *feedback.current * feedback.motor_torque_constant_nm_per_amp
                             * feedback.gear_ratio;
        }
        const int encoder = bytes[6] | (static_cast<int>(bytes[7]) << 8);
        if (feedback.gear_ratio > 1.0) {
            const auto accumulated_counts = feedback.multi_turn_angle.update(encoder, feedback.zero_point);
            *feedback.angle = feedback.direction * static_cast<double>(accumulated_counts)
                            * (2.0 * std::numbers::pi / 65536.0) / feedback.gear_ratio;
            return;
        }

        int offset = (encoder - feedback.zero_point + 65536) % 65536;
        if (offset > 32768)
            offset -= 65536;
        *feedback.angle = feedback.direction * static_cast<double>(offset)
                        * (2.0 * std::numbers::pi / 65536.0) / feedback.gear_ratio;
    }

    void update_can1_motor_id_scan(int64_t now) {
        std::string summary;
        for (size_t i = 0; i < kCanIdScanCount; ++i) {
            if (!fresh(can1_motor_id_scan_received_ns_[i].load(std::memory_order_acquire), now,
                       kCanIdScanFreshnessNs))
                continue;
            if (!summary.empty())
                summary += ", ";
            summary += "0x";
            constexpr char hex[] = "0123456789ABCDEF";
            const auto can_id = kCanIdScanFirst + i;
            summary += hex[(can_id >> 4) & 0x0F];
            summary += hex[can_id & 0x0F];
        }
        *can1_motor_id_scan_ = summary.empty() ? "no LK replies" : summary;
    }

    void update_imu(int64_t now) {
        *imu_online_ =
            fresh(
                accelerometer_received_ns_.load(std::memory_order_acquire), now,
                feedback_timeout_ns_)
            && fresh(
                gyroscope_received_ns_.load(std::memory_order_acquire), now, feedback_timeout_ns_);
        if (!*imu_online_) {
            *roll_imu_velocity_ = *pitch_imu_velocity_ = *yaw_imu_velocity_ = kNaN;
            *roll_imu_angle_ = *pitch_imu_angle_ = *yaw_imu_angle_ = kNaN;
            return;
        }

        bmi088_.update_status();
        const double q0 = bmi088_.q0(), q1 = bmi088_.q1();
        const double q2 = bmi088_.q2(), q3 = bmi088_.q3();
        *roll_imu_velocity_  = bmi088_.gx();
        *pitch_imu_velocity_ = bmi088_.gy();
        *yaw_imu_velocity_   = bmi088_.gz();
        *roll_imu_angle_  = std::atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
        *pitch_imu_angle_ = std::asin(std::clamp(2.0 * (q0 * q2 - q3 * q1), -1.0, 1.0));
        *yaw_imu_angle_   = std::atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
    }

    void update_remote(int64_t now) {
        const auto snapshot    = remote_snapshot_.load(std::memory_order_acquire);
        const auto received_at = snapshot.received_ns;
        const bool online      = fresh(received_at, now, kRemoteTimeoutNs);
        *switch_right_         = static_cast<rmcs_msgs::Switch>(snapshot.switches & 0x03);
        *switch_left_          = static_cast<rmcs_msgs::Switch>((snapshot.switches >> 2) & 0x03);
        *joystick_right_ = {online ? snapshot.joystick_x : 0.0, online ? snapshot.joystick_y : 0.0};
        *joystick_right_x_        = online ? snapshot.joystick_x : 0.0;
        *joystick_right_y_        = online ? snapshot.joystick_y : 0.0;
        *joystick_left_ = {online ? snapshot.joystick_left_x : 0.0,
                          online ? snapshot.joystick_left_y : 0.0};
        *joystick_left_x_         = online ? snapshot.joystick_left_x : 0.0;
        *joystick_left_y_         = online ? snapshot.joystick_left_y : 0.0;
        *remote_last_received_ns_ = online ? received_at : 0;
    }

    void store_joint2_encoder_status(const View::Can& data) {
        const auto low = std::to_integer<uint16_t>(data.can_data[6]);
        const auto high = std::to_integer<uint16_t>(data.can_data[7]);
        // Keep angle and receive time from the same CAN frame for the velocity estimate.
        joint2_encoder_snapshot_.store(
            Joint2EncoderSnapshot{
                .received_ns = monotonic_now_ns(),
                .raw_angle = static_cast<uint32_t>(low | (high << 8))},
            std::memory_order_release);
    }

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if ((can != Spec::kCans.kCan1 && can != Spec::kCans.kCan2) || data.is_fdcan
            || data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() != 8)
            return;
        if (can == Spec::kCans.kCan1 && data.can_id == 0x200) {
            if (!separate_joint2_encoder_board_)
                store_joint2_encoder_status(data);
            return;
        }
        const auto command = std::to_integer<uint8_t>(data.can_data[0]);
        if (command != 0x9C && command != 0xA4 && command != 0xA1 && command != 0xAD)
            return;
        if (can == Spec::kCans.kCan1 && data.can_id >= kCanIdScanFirst
            && data.can_id < kCanIdScanFirst + kCanIdScanCount)
            can1_motor_id_scan_received_ns_[data.can_id - kCanIdScanFirst].store(
                monotonic_now_ns(), std::memory_order_release);
        uint64_t packet;
        std::memcpy(&packet, data.can_data.data(), sizeof(packet));
        const auto received_ns = monotonic_now_ns();
        if (can == Spec::kCans.kCan1
            && (data.can_id == kImagePitchCanId || data.can_id == kSmallYawCanId)) {
            auto& feedback = data.can_id == kImagePitchCanId ? pitch_feedback_ : small_yaw_feedback_;
            if (data.can_id == kSmallYawCanId) {
                const auto raw_angle = static_cast<uint16_t>(
                    std::to_integer<uint8_t>(data.can_data[6])
                    | (uint16_t{std::to_integer<uint8_t>(data.can_data[7])} << 8));
                const auto current_bits = static_cast<uint16_t>(
                    std::to_integer<uint8_t>(data.can_data[2])
                    | (uint16_t{std::to_integer<uint8_t>(data.can_data[3])} << 8));
                small_yaw_can_snapshot_.store(
                    SmallYawCanSnapshot{.received_ns = received_ns,
                                        .raw_angle = raw_angle,
                                        .current_count = std::bit_cast<int16_t>(current_bits),
                                        .valid = true},
                    std::memory_order_release);
            }
            feedback.packet.store(packet, std::memory_order_relaxed);
            feedback.received_ns.store(received_ns, std::memory_order_release);
            return;
        }
        for (auto& feedback : joint_angle_feedback_) {
            if (feedback.can2 == (can == Spec::kCans.kCan2) && feedback.can_id == data.can_id) {
                feedback.packet.store(packet, std::memory_order_relaxed);
                feedback.received_ns.store(received_ns, std::memory_order_release);
                return;
            }
        }
    }

    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart != Spec::kUarts.kDbus || data.uart_data.size() != 18)
            return;

        const auto frame = decode_gimbal_dbus(data.uart_data);
        publish_remote_snapshot(frame);
    }

    void publish_remote_snapshot(const GimbalDbusFrame& frame) {
        // Keep switch, joystick and receive time from the same DBUS frame. Separate atomics could
        // otherwise mix states across the double-down release edge.
        remote_snapshot_.store(
            RemoteSnapshot{
                .received_ns = monotonic_now_ns(),
                .switches    = frame.switches,
                .joystick_x  = frame.right_x,
                .joystick_y  = frame.right_y,
                .joystick_left_x = frame.left_x,
                .joystick_left_y = frame.left_y},
            std::memory_order_release);
    }

    void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
        bmi088_.store_accelerometer_status(data.x, data.y, data.z);
        accelerometer_received_ns_.store(monotonic_now_ns(), std::memory_order_release);
    }

    void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
        bmi088_.store_gyroscope_status(data.x, data.y, data.z);
        gyroscope_received_ns_.store(monotonic_now_ns(), std::memory_order_release);
    }

    static constexpr double kNaN               = std::numeric_limits<double>::quiet_NaN();
    static constexpr uint32_t kImagePitchCanId = 0x148;
    static constexpr uint32_t kSmallYawCanId = 0x141; // Joint 1, arm base; CAN2 / 0x141 is joint 6.
    static constexpr uint32_t kCanIdScanFirst = 0x140;
    static constexpr size_t kCanIdScanCount = 16;
    static constexpr int64_t kQueryIntervalNs = 10'000'000; // 100 Hz status polling.
    static constexpr int64_t kCanIdScanFreshnessNs = 1'000'000'000;
    static constexpr int64_t kRemoteTimeoutNs = 500'000'000;

    double update_rate_;
    int64_t feedback_timeout_ns_{100'000'000};
    int64_t damping_remote_timeout_ns_{200'000'000};
    int64_t last_query_ns_{0};
    size_t can1_motor_id_scan_index_{0};
    SmallYawDampingParameters damping_parameters_;
    SmallYawPositionHoldParameters hold_parameters_;
    SmallYawIdentificationParameters identification_parameters_;
    struct SmallYawRobustParameters {
        bool enabled{false};
        bool external_reference{false};
        double max_velocity_rad_s{0.35};
        double max_acceleration_rad_s2{0.5};
        double joystick_velocity_direction{1.0};
        double torque_limit_nm{2.5};
        double torque_slew_nm_s{18.0};
        controller::robust::RobustVelocityParameters model{};

        bool valid() const noexcept {
            return std::isfinite(max_velocity_rad_s) && max_velocity_rad_s > 0.0
                && std::isfinite(max_acceleration_rad_s2) && max_acceleration_rad_s2 > 0.0
                && std::isfinite(joystick_velocity_direction)
                && std::abs(joystick_velocity_direction) > 0.0
                && std::isfinite(torque_limit_nm) && torque_limit_nm > 0.0
                && torque_limit_nm <= 10.0
                && std::isfinite(torque_slew_nm_s) && torque_slew_nm_s > 0.0;
        }
    };
    SmallYawRobustParameters small_yaw_robust_parameters_;
    Joint2LiftPidParameters joint2_pid_parameters_;
    bool joint2_pid_enabled_{false};
    bool separate_joint2_encoder_board_{false};
    int64_t joint2_encoder_zero_point_{39433};
    std::atomic<Joint2EncoderSnapshot> joint2_encoder_snapshot_{};
    std::atomic<SmallYawCommandSnapshot> small_yaw_command_snapshot_{};
    std::atomic<SmallYawCanSnapshot> small_yaw_can_snapshot_{};
    Joint2EncoderVelocityEstimator joint2_encoder_velocity_estimator_;
    std::shared_ptr<GimbalSmallYawDampingCommandTest> damping_command_;
    std::shared_ptr<GimbalJoint2LiftCommandTest> joint2_lift_command_;
    device::Bmi088 bmi088_;
    MotorFeedback pitch_feedback_, small_yaw_feedback_;
    std::array<JointAngleFeedback, 5> joint_angle_feedback_;
    std::atomic<int64_t> accelerometer_received_ns_{0};
    std::atomic<int64_t> gyroscope_received_ns_{0};
    std::array<std::atomic<int64_t>, kCanIdScanCount> can1_motor_id_scan_received_ns_{};
    std::atomic<RemoteSnapshot> remote_snapshot_{};
    int64_t last_published_small_yaw_sample_ns_{0};
    rclcpp::Publisher<rmcs_msgs::msg::GimbalCanSample>::SharedPtr can_sample_publisher_;

    OutputInterface<bool> imu_online_;
    OutputInterface<double> joint2_encoder_angle_, joint2_encoder_velocity_;
    OutputInterface<bool> joint2_encoder_online_;
    OutputInterface<int> joint2_encoder_raw_angle_;
    OutputInterface<std::string> can1_motor_id_scan_;
    OutputInterface<double> yaw_imu_velocity_, yaw_imu_angle_;
    OutputInterface<double> pitch_imu_velocity_, pitch_imu_angle_;
    OutputInterface<double> roll_imu_velocity_, roll_imu_angle_;
    OutputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    OutputInterface<Eigen::Vector2d> joystick_right_;
    OutputInterface<double> joystick_right_x_, joystick_right_y_;
    OutputInterface<Eigen::Vector2d> joystick_left_;
    OutputInterface<double> joystick_left_x_, joystick_left_y_;
    OutputInterface<int64_t> remote_last_received_ns_;

    // Receivers and boards must follow all callback state in member destruction order.
    Joint2EncoderReceiver encoder_receiver_;
    std::unique_ptr<librmcs::board::CBoard> board_;
    std::unique_ptr<librmcs::board::CBoard> encoder_board_;
};

GimbalSmallYawDampingCommandTest::GimbalSmallYawDampingCommandTest(GimbalFeedbackTest& hardware)
    : hardware_(hardware), damping_(hardware.damping_parameters_)
    , position_hold_(hardware.hold_parameters_)
    , identification_(hardware.identification_parameters_)
    , robust_controller_(hardware.small_yaw_robust_parameters_.model) {
    register_input("/remote/switch/left", switch_left_);
    register_input("/remote/switch/right", switch_right_);
    register_input("/remote/last_received_monotonic_ns", remote_last_received_ns_);
    register_input("/remote/joystick/right", joystick_right_);
    register_input("/gimbal/feedback/small_yaw_online", small_yaw_online_);
    register_input("/arm/joint_1/motor/angle", small_yaw_angle_);
    if (hardware.small_yaw_robust_parameters_.external_reference) {
        register_input("/gimbal/test/coordinated_yaw_active", coordinated_active_);
        register_input("/gimbal/test/small_yaw_coordinated_target_velocity", coordinated_velocity_);
        register_input("/gimbal/test/small_yaw_coordinated_target_acceleration", coordinated_acceleration_);
    }
    if (hardware.identification_parameters_.enabled) {
        register_input("/gimbal/test/joint2_pid_position_error", joint2_position_error_);
        register_input("/arm/joint_2/encoder/angle", identification_joint2_angle_);
        register_input("/gimbal/test/joint2_pid_permitted", joint2_permitted_);
    }
    register_output("/gimbal/test/small_yaw_damping_permitted", permitted_, false);
    register_output("/gimbal/test/small_yaw_damping_active", active_, false);
    register_output("/gimbal/test/small_yaw_damping_velocity", estimated_velocity_, 0.0);
    register_output("/gimbal/test/small_yaw_damping_torque", requested_torque_, 0.0);
    register_output("/gimbal/test/small_yaw_damping_torque_applied", applied_torque_, 0.0);
    register_output("/gimbal/test/small_yaw_hold_reference_valid", hold_reference_valid_, false);
    register_output("/gimbal/test/small_yaw_hold_target_angle", hold_target_angle_,
                    std::numeric_limits<double>::quiet_NaN());
    register_output("/gimbal/test/small_yaw_hold_position_error", hold_position_error_, 0.0);
    register_output("/gimbal/test/small_yaw_block_reason", block_reason_, std::string{"disabled"});
    register_output("/gimbal/test/small_yaw_identification_state", identification_state_, std::string{"disabled"});
    register_output("/gimbal/test/small_yaw_identification_elapsed_s", identification_elapsed_, 0.0);
    register_output("/gimbal/test/small_yaw_identification_relative_angle", identification_relative_angle_, 0.0);
    register_output("/gimbal/test/small_yaw_identification_joint2_angle_span", identification_joint2_angle_span_, 0.0);
    register_output("/gimbal/test/small_yaw_identification_joint2_stable_elapsed_s", identification_joint2_stable_elapsed_, 0.0);
    register_output("/gimbal/test/small_yaw_identification_joint2_window_ready", identification_joint2_window_ready_, false);
    register_output("/gimbal/test/small_yaw_identification_complete", identification_complete_, false);
    register_output("/gimbal/test/small_yaw_robust_permitted", robust_permitted_, false);
    register_output("/gimbal/test/small_yaw_robust_active", robust_active_, false);
    register_output("/gimbal/test/small_yaw_robust_target_velocity", robust_target_velocity_, 0.0);
    register_output("/gimbal/test/small_yaw_robust_target_acceleration", robust_target_acceleration_, 0.0);
    register_output("/gimbal/test/small_yaw_robust_velocity", robust_velocity_, 0.0);
    register_output("/gimbal/test/small_yaw_robust_torque", robust_torque_, 0.0);
    register_output("/gimbal/test/small_yaw_robust_torque_applied", robust_torque_applied_, 0.0);
    register_output("/gimbal/test/small_yaw_robust_integral_torque", robust_integral_torque_, 0.0);
    register_output("/gimbal/test/small_yaw_robust_block_reason", robust_block_reason_,
                    std::string{"disabled"});
}

void GimbalSmallYawDampingCommandTest::update() {
    using rmcs_msgs::Switch;
    const auto now = GimbalFeedbackTest::monotonic_now_ns();
    const double dt = previous_update_ns_ == 0 ? 0.001 : (now - previous_update_ns_) * 1e-9;
    previous_update_ns_ = now;
    const double angle = *small_yaw_angle_;
    const bool feedback_valid = *small_yaw_online_ && std::isfinite(angle);
    const bool update_valid = std::isfinite(dt) && dt > 0.0 && dt <= 0.02;
    const auto received_at = *remote_last_received_ns_;
    const auto left = *switch_left_, right = *switch_right_;
    const auto valid_switch = [](Switch value) {
        return value == Switch::UP || value == Switch::MIDDLE || value == Switch::DOWN;
    };
    const bool remote_valid = received_at > 0 && received_at <= now
                           && now - received_at <= hardware_.damping_remote_timeout_ns_
                           && valid_switch(left) && valid_switch(right);
    const bool both_down = left == Switch::DOWN && right == Switch::DOWN;
    const double stick = (*joystick_right_)[0];
    if (hardware_.identification_parameters_.enabled) {
        // The identification fixture uses the opposite motor polarity from the
        // requested test sign. Keep the damping/hold paths unchanged and flip
        // only the open-loop identification excitation here.
        const double torque = hardware_.identification_parameters_.command_sign
            * identification_.update(SmallYawIdentificationInput{
            .angle = angle,
            .joint2_position_error = *joint2_position_error_,
            .joint2_angle = *identification_joint2_angle_,
            .remote_valid = remote_valid, .both_down = both_down,
            .feedback_valid = feedback_valid, .joint2_permitted = *joint2_permitted_,
            .dt = dt});
        *permitted_ = identification_.permitted();
        *estimated_velocity_ = identification_.estimated_velocity();
        *requested_torque_ = torque;
        *block_reason_ = identification_.faulted()
            ? identification_.fault_reason() : identification_.state();
        *identification_state_ = identification_.state();
        *identification_elapsed_ = identification_.elapsed_s();
        *identification_relative_angle_ = identification_.relative_angle();
        *identification_joint2_angle_span_ = identification_.joint2_angle_span();
        *identification_joint2_stable_elapsed_ = identification_.joint2_stable_elapsed_s();
        *identification_joint2_window_ready_ = identification_.joint2_window_ready();
        *identification_complete_ = identification_.complete();
        *hold_reference_valid_ = false;
        *hold_target_angle_ = std::numeric_limits<double>::quiet_NaN();
        *hold_position_error_ = 0.0;
        *robust_permitted_ = false;
        *robust_active_ = false;
        *robust_target_velocity_ = 0.0;
        *robust_target_acceleration_ = 0.0;
        *robust_velocity_ = 0.0;
        *robust_torque_ = 0.0;
        *robust_torque_applied_ = 0.0;
        robust_controller_.reset();
        *robust_integral_torque_ = 0.0;
        *robust_block_reason_ = "identification_active";
        // One sender owns this CAN ID in all modes. Zero edges are sent without
        // waiting for the periodic 2 ms hold, including fault and pulse end.
        if (previous_send_ns_ == 0 || now - previous_send_ns_ >= 2'000'000
            || (last_sent_torque_ != 0.0
                && (torque == 0.0 || !*permitted_ || last_sent_torque_ * torque < 0.0))) {
            hardware_.send_small_yaw_torque(
                torque, hardware_.identification_parameters_.torque_limit_nm);
            constexpr double nm_per_count = 3.6 * 66.0 / 4096.0;
            last_sent_torque_ = std::round(torque / nm_per_count) * nm_per_count;
            previous_send_ns_ = now;
        }
        *applied_torque_ = last_sent_torque_;
        *active_ = *permitted_ && last_sent_torque_ != 0.0;
        return;
    }
    if (hardware_.small_yaw_robust_parameters_.enabled) {
        // The robust path owns the same CAN ID exclusively. It uses the same
        // arming sequence as the existing brake: double-down, then centered stick.
        if (!remote_valid || !feedback_valid || !update_valid) {
            double_down_confirmed_ = false;
            stick_centered_after_safety_ = false;
        } else if (both_down) {
            double_down_confirmed_ = true;
            stick_centered_after_safety_ = false;
        } else if (double_down_confirmed_ && std::isfinite(stick) && std::abs(stick) <= 0.05) {
            stick_centered_after_safety_ = true;
        }
        *permitted_ = false;
        const bool stick_valid = std::isfinite(stick);
        const bool external = hardware_.small_yaw_robust_parameters_.external_reference;
        const bool reference_valid = !external || (*coordinated_active_
            && std::isfinite(*coordinated_velocity_) && std::isfinite(*coordinated_acceleration_)
            && std::abs(*coordinated_velocity_)
                <= hardware_.small_yaw_robust_parameters_.max_velocity_rad_s + 1e-9
            && std::abs(*coordinated_acceleration_)
                <= hardware_.small_yaw_robust_parameters_.max_acceleration_rad_s2 + 1e-9);
        if (!hardware_.small_yaw_robust_parameters_.enabled)
            *robust_block_reason_ = "disabled";
        else if (!remote_valid)
            *robust_block_reason_ = "remote_invalid";
        else if (!valid_switch(left) || !valid_switch(right))
            *robust_block_reason_ = "switch_invalid";
        else if (both_down)
            *robust_block_reason_ = "double_down_protection";
        else if (!feedback_valid)
            *robust_block_reason_ = "feedback_invalid";
        else if (!update_valid)
            *robust_block_reason_ = "update_gap";
        else if (!double_down_confirmed_)
            *robust_block_reason_ = "waiting_for_double_down";
        // Centering is an arming edge, not a motion interlock. Once the
        // operator has released the double-down switch with the stick centered,
        // moving X is the intended small-yaw command.
        else if (!stick_centered_after_safety_ || !stick_valid)
            *robust_block_reason_ = "waiting_for_stick_center";
        else if (!reference_valid)
            *robust_block_reason_ = "coordinated_reference_unavailable";
        else {
            *permitted_ = true;
            *robust_block_reason_ = "none";
        }

        const double joystick = stick_valid && std::abs(stick) > 0.05
            ? std::copysign((std::abs(stick) - 0.05) / 0.95, stick) : 0.0;
        const double target_velocity = joystick * hardware_.small_yaw_robust_parameters_.max_velocity_rad_s
            * hardware_.small_yaw_robust_parameters_.joystick_velocity_direction;
        // The coordinator already bounds both velocity and acceleration. A
        // second independent ramp here would delay base-motion compensation.
        robust_reference_.update(
            target_velocity, hardware_.small_yaw_robust_parameters_.max_acceleration_rad_s2, dt,
            *permitted_ && !external);
        const double reference_velocity = external
            ? (*permitted_ ? *coordinated_velocity_ : 0.0) : robust_reference_.velocity();
        const double reference_acceleration = external
            ? (*permitted_ ? *coordinated_acceleration_ : 0.0) : robust_reference_.acceleration();
        const double measured_velocity = *permitted_ ? motion_estimator_.update(angle, dt) : 0.0;
        if (!*permitted_)
            robust_controller_.reset();
        const double torque_demand = *permitted_
            ? robust_controller_.update(
                reference_velocity, measured_velocity, reference_acceleration, dt)
            : 0.0;
        *robust_integral_torque_ = robust_controller_.integral_torque();
        double torque = std::clamp(
            torque_demand, -hardware_.small_yaw_robust_parameters_.torque_limit_nm,
            hardware_.small_yaw_robust_parameters_.torque_limit_nm);
        const bool force_stop = !*permitted_ || !std::isfinite(torque);
        if (force_stop) {
            torque = 0.0;
            robust_reference_.update(0.0, hardware_.small_yaw_robust_parameters_.max_acceleration_rad_s2,
                                     std::max(dt, 1e-3), false);
            motion_estimator_.reset();
        }
        // Limit command buildup and clear a reversal before applying the new sign.
        const double step = hardware_.small_yaw_robust_parameters_.torque_slew_nm_s
            * std::min(std::max(dt, 0.0), 0.02);
        const double slew_torque = robust_torque_slew_.update(torque, step, force_stop);

        *robust_permitted_ = *permitted_;
        *robust_target_velocity_ = reference_velocity;
        *robust_target_acceleration_ = reference_acceleration;
        *robust_velocity_ = measured_velocity;
        *robust_torque_ = torque;
        if (hardware_.small_yaw_robust_parameters_.enabled
            && (previous_send_ns_ == 0 || now - previous_send_ns_ >= 2'000'000
                // Safety loss removes the last submitted command immediately,
                // even if the continuous slew state has already reached zero.
                || (force_stop && last_sent_torque_ != 0.0))) {
            last_sent_torque_ = hardware_.send_small_yaw_torque(
                slew_torque, hardware_.small_yaw_robust_parameters_.torque_limit_nm);
            previous_send_ns_ = now;
        }
        // Report the encoded command actually submitted to the board, including
        // between send cycles; the slew state may already be advancing again.
        *robust_torque_applied_ = hardware_.small_yaw_command_snapshot_.load(
            std::memory_order_acquire).torque_nm;
        *robust_active_ = *permitted_ && *robust_torque_applied_ != 0.0;
        // Keep the historical damping telemetry as the common small-yaw
        // command status so existing Foxglove layouts show the active owner.
        *permitted_ = *robust_permitted_;
        *active_ = *robust_active_;
        *estimated_velocity_ = *robust_velocity_;
        *requested_torque_ = *robust_torque_;
        *applied_torque_ = *robust_torque_applied_;
        *hold_reference_valid_ = false;
        *hold_target_angle_ = std::numeric_limits<double>::quiet_NaN();
        *hold_position_error_ = 0.0;
        return;
    }
    *robust_permitted_ = false;
    *robust_active_ = false;
    *robust_target_velocity_ = 0.0;
    *robust_target_acceleration_ = 0.0;
    *robust_velocity_ = 0.0;
    *robust_torque_ = 0.0;
    *robust_torque_applied_ = 0.0;
    robust_controller_.reset();
    *robust_integral_torque_ = 0.0;
    *robust_block_reason_ = "disabled";
    // Holding a stale angle after feedback loss is unsafe, and silently adopting
    // a different angle would hide that the original hold reference was lost.
    // Require a new double-down confirmation after a hold feedback/time fault.
    if (!remote_valid || (hardware_.hold_parameters_.enabled && (!feedback_valid || !update_valid))) {
        double_down_confirmed_ = false;
        stick_centered_after_safety_ = false;
    } else if (both_down) {
        double_down_confirmed_ = true;
        stick_centered_after_safety_ = false;
    } else if (double_down_confirmed_ && std::isfinite(stick) && std::abs(stick) <= 0.05) {
        stick_centered_after_safety_ = true;
    }
    // Remain available for the small-yaw coast-down after the big-yaw identification ends.
    *permitted_ = hardware_.damping_parameters_.enabled && remote_valid && !both_down
                && double_down_confirmed_ && stick_centered_after_safety_
                && std::isfinite(stick) && feedback_valid
                && (!hardware_.hold_parameters_.enabled || update_valid);
    *block_reason_ = !hardware_.damping_parameters_.enabled ? "disabled"
        : !remote_valid ? "remote_invalid"
        : both_down ? "double_down_protection"
        : !feedback_valid ? "feedback_invalid"
        : hardware_.hold_parameters_.enabled && !update_valid ? "update_gap"
        : !double_down_confirmed_ ? "waiting_for_double_down"
        : !stick_centered_after_safety_ || !std::isfinite(stick) ? "waiting_for_stick_center"
        : "none";
    // The motor's integer-degree/s speed repeatedly reports zero during slow drift.
    // Use encoder motion for braking without changing the original feedback telemetry.
    double velocity = 0.0;
    if (*permitted_)
        velocity = motion_estimator_.update(angle, dt);
    else
        motion_estimator_.reset();
    const double torque = hardware_.hold_parameters_.enabled
        ? position_hold_.update(angle, velocity, *permitted_, dt)
        : damping_.update(velocity, *permitted_, dt);
    *estimated_velocity_ = hardware_.hold_parameters_.enabled ? position_hold_.velocity() : velocity;
    *hold_reference_valid_ = position_hold_.reference_valid();
    *hold_target_angle_ = position_hold_.reference_valid()
        ? position_hold_.target_angle() : std::numeric_limits<double>::quiet_NaN();
    *hold_position_error_ = position_hold_.position_error();
    *requested_torque_ = torque;

    // Position feedback needs a shorter current-command hold than the old
    // on/off brake: a 10 ms current pulse is appreciable on this light joint.
    const int64_t command_interval_ns = hardware_.hold_parameters_.enabled ? 2'000'000 : 10'000'000;
    if (hardware_.damping_parameters_.enabled
        && (previous_send_ns_ == 0 || now - previous_send_ns_ >= command_interval_ns
            || (last_sent_torque_ != 0.0
                && (torque == 0.0 || !*permitted_ || last_sent_torque_ * torque < 0.0
                    || (!hardware_.hold_parameters_.enabled && last_sent_torque_ * velocity > 0.0))))) {
        const double torque_limit = hardware_.hold_parameters_.enabled
            ? hardware_.hold_parameters_.torque_limit_nm : hardware_.damping_parameters_.torque_nm;
        last_sent_torque_ = hardware_.send_small_yaw_torque(torque, torque_limit);
        previous_send_ns_ = now;
    }
    *applied_torque_ = last_sent_torque_;
    *active_ = *permitted_ && last_sent_torque_ != 0.0;
    if (hardware_.hold_parameters_.enabled && hardware_.hold_parameters_.sliding_enabled) {
        *robust_permitted_ = *permitted_;
        *robust_active_ = *active_;
        *robust_target_velocity_ = position_hold_.target_velocity();
        *robust_velocity_ = position_hold_.velocity();
        *robust_torque_ = torque;
        *robust_torque_applied_ = last_sent_torque_;
        *robust_block_reason_ = *block_reason_;
    }
}

GimbalJoint2LiftCommandTest::GimbalJoint2LiftCommandTest(GimbalFeedbackTest& hardware)
    : hardware_(hardware), controller_(hardware.joint2_pid_parameters_)
    , controller_enabled_(hardware.joint2_pid_enabled_) {
    register_input("/remote/switch/left", switch_left_);
    register_input("/remote/switch/right", switch_right_);
    register_input("/remote/last_received_monotonic_ns", remote_last_received_ns_);
    register_input("/remote/joystick/right", joystick_right_);
    register_input("/gimbal/feedback/joint_2_online", joint2_motor_online_);
    register_input("/gimbal/feedback/joint_2_encoder_online", joint2_encoder_online_);
    register_input("/arm/joint_2/encoder/angle", joint2_encoder_angle_);
    register_input("/arm/joint_2/encoder/velocity", joint2_encoder_velocity_);
    register_output("/gimbal/test/joint2_pid_permitted", permitted_, false);
    register_output("/gimbal/test/joint2_pid_active", active_, false);
    register_output("/gimbal/test/joint2_pid_target_valid", target_valid_, false);
    register_output("/gimbal/test/joint2_pid_target_angle", target_angle_,
                    std::numeric_limits<double>::quiet_NaN());
    register_output("/gimbal/test/joint2_pid_position_error", position_error_,
                    std::numeric_limits<double>::quiet_NaN());
    register_output("/gimbal/test/joint2_pid_torque", requested_torque_, 0.0);
    register_output("/gimbal/test/joint2_pid_torque_applied", applied_torque_, 0.0);
    register_output("/gimbal/test/joint2_pid_block_reason", block_reason_,
                    std::string{"disabled"});
}

void GimbalJoint2LiftCommandTest::update() {
    using rmcs_msgs::Switch;
    const auto now = GimbalFeedbackTest::monotonic_now_ns();
    const double dt = previous_update_ns_ == 0 ? 0.001
        : static_cast<double>(now - previous_update_ns_) * 1e-9;
    previous_update_ns_ = now;

    const auto left = *switch_left_;
    const auto right = *switch_right_;
    const auto valid_switch = [](Switch value) {
        return value == Switch::UP || value == Switch::MIDDLE || value == Switch::DOWN;
    };
    const auto received_at = *remote_last_received_ns_;
    const bool remote_valid = received_at > 0 && received_at <= now
        && now - received_at <= hardware_.damping_remote_timeout_ns_
        && valid_switch(left) && valid_switch(right);
    const bool switches_valid = valid_switch(left) && valid_switch(right);
    const bool both_down = left == Switch::DOWN && right == Switch::DOWN;
    const double stick = (*joystick_right_)[0];
    const bool feedback_valid = *joint2_motor_online_ && *joint2_encoder_online_
        && std::isfinite(*joint2_encoder_angle_) && std::isfinite(*joint2_encoder_velocity_);
    const bool update_valid = std::isfinite(dt) && dt > 0.0 && dt <= 0.02;

    // A stale or invalid position must never reuse a target selected from an
    // earlier feedback stream. A normal double-down/release keeps the target
    // latched, so toggling the safety switch cannot add another 30 degrees.
    if (!remote_valid || !feedback_valid || !update_valid)
        controller_.invalidate_target();

    *permitted_ = safety_gate_.update(
        controller_enabled_, remote_valid, switches_valid, both_down, feedback_valid,
        update_valid, std::isfinite(stick) && std::abs(stick) <= 0.05);

    const double angle = *joint2_encoder_angle_;
    if (*permitted_) {
        if (!controller_.target_valid())
            controller_.capture_target(angle);
        else if (!was_permitted_)
            controller_.resume(angle);
    }
    const double torque = controller_.update(
        angle, *joint2_encoder_velocity_, dt, *permitted_);
    was_permitted_ = *permitted_;

    *target_valid_ = controller_.target_valid();
    *target_angle_ = controller_.target_valid()
        ? controller_.target_angle() : std::numeric_limits<double>::quiet_NaN();
    *position_error_ = controller_.target_valid() && std::isfinite(angle)
        ? controller_.position_error(angle) : std::numeric_limits<double>::quiet_NaN();
    *requested_torque_ = torque;

    if (!controller_enabled_) {
        *block_reason_ = "disabled";
    } else if (!remote_valid) {
        *block_reason_ = "remote_invalid";
    } else if (!switches_valid) {
        *block_reason_ = "switch_invalid";
    } else if (both_down) {
        *block_reason_ = "double_down_protection";
    } else if (!feedback_valid) {
        *block_reason_ = "feedback_invalid";
    } else if (!update_valid) {
        *block_reason_ = "update_gap";
    } else if (!safety_gate_.double_down_confirmed()) {
        *block_reason_ = "waiting_for_double_down";
    } else if (!*permitted_) {
        *block_reason_ = "waiting_for_stick_center";
    } else {
        *block_reason_ = "none";
    }

    const int64_t send_interval_ns = *permitted_ ? 5'000'000 : 10'000'000;
    if (controller_enabled_
        && (previous_send_ns_ == 0 || now - previous_send_ns_ >= send_interval_ns
            || (last_sent_torque_ != 0.0 && (!*permitted_ || torque == 0.0
                                            || last_sent_torque_ * torque < 0.0)))) {
        hardware_.send_joint2_torque(torque);
        constexpr double nm_per_count = 42.0 * 0.12 * 33.0 / 4096.0;
        last_sent_torque_ = std::round(torque / nm_per_count) * nm_per_count;
        previous_send_ns_ = now;
    }
    *applied_torque_ = last_sent_torque_;
    *active_ = *permitted_ && last_sent_torque_ != 0.0;
}

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::GimbalFeedbackTest, rmcs_executor::Component)
