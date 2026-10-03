#pragma once

#include "control_interval.hpp"
#include "policy.hpp"
#include "recovery_controller.hpp"
#include "recovery_observer.hpp"
#include "recovery_sensor_guard.hpp"
#include "v6_recovery_controller.hpp"
#include "v6_recovery_observer.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <optional>
#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

namespace rmcs::rl {

// Policy order differs from the hardware bus order.
inline constexpr std::array<const char*, 6> kMotorNames{"left_hip_joint",  "left_knee_joint",
                                                        "right_hip_joint", "right_knee_joint",
                                                        "left_wheel",      "right_wheel"};
static_assert(kMotorNames.size() == PolicyAction{}.size());

enum class State : int { kInit = 0, kIdle = 1, kPrepare = 2, kRl = 3 };

bool accepts_motion_command(
    bool jump, double height, rmcs_msgs::ChassisMode mode,
    const rmcs_description::BaseLink::DirectionVector& command,
    const PolicyProfile& profile = kV6PolicyProfile);

class RlController final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    RlController();
    ~RlController() override = default;

    void before_updating() override;
    void update() override;

private:
    using Clock = std::chrono::steady_clock;
    using Vector6 = Eigen::Matrix<double, 6, 1>;

    void enter_(State next);
    void latch_fault_(std::optional<RecoveryFailure> reason = std::nullopt);
    bool advance_recovery_();
    bool advance_v6_recovery_();
    V6RecoveryFeedback v6_recovery_feedback_() const;
    std::optional<V6RecoveryFeedback> observe_v6_recovery_();
    RecoverySensorData recovery_sensor_data_() const;
    RecoveryPhase recovery_phase_() const;
    bool recovery_motion_hold_() const;
    bool evaluate_policy_(std::size_t tick, bool recovering);
    void clear_outputs_();
    void update_state_output_();
    bool read_model_state_();
    Eigen::Quaterniond world_base_orientation_() const;
    std::optional<RecoveryFeedback> observe_recovery_();
    bool update_prepare_();
    void update_command_reference_();
    bool assemble_observation_(bool shadow_recovery = false);
    std::expected<void, std::string>
        process_action_(const PolicyAction& raw, bool shadow_recovery = false);
    void compute_motor_torques_();
    void apply_soft_limits_(Eigen::Vector4d& leg_torque) const;

    std::array<InputInterface<double>, 6> angle_inputs_;
    std::array<InputInterface<double>, 6> velocity_inputs_;
    std::array<InputInterface<double>, 6> max_torque_inputs_;
    std::array<InputInterface<double>, 6> torque_feedback_inputs_;
    std::array<OutputInterface<double>, 6> torque_outputs_;
    std::array<InputInterface<int>, 4> fault_inputs_;
    std::array<InputInterface<std::uint64_t>, 6> motor_feedback_sequences_;
    std::array<InputInterface<std::uint64_t>, 6> motor_feedback_ns_;
    std::array<InputInterface<double>, 2> wheel_submitted_torque_;
    std::array<InputInterface<std::uint64_t>, 2> wheel_submitted_ns_;
    std::array<InputInterface<std::uint8_t>, 2> wheel_submitted_kind_;
    InputInterface<bool> feedback_fresh_;
    InputInterface<Eigen::Quaterniond> orientation_;
    InputInterface<Eigen::Vector3d> gyro_;
    InputInterface<rmcs_description::BaseLink::DirectionVector> velocity_command_;
    InputInterface<double> height_command_;
    InputInterface<int> state_command_;
    InputInterface<std::size_t> reset_count_;
    InputInterface<bool> jump_command_;
    InputInterface<double> jump_apex_command_;
    InputInterface<rmcs_msgs::ChassisMode> chassis_mode_;
    InputInterface<std::size_t> update_count_;
    InputInterface<double> update_rate_;
    InputInterface<Clock::time_point> timestamp_;
    InputInterface<bool> dm_control_ready_;
    InputInterface<Eigen::Vector3d> acceleration_;
    InputInterface<std::uint64_t> acceleration_ns_;
    InputInterface<std::uint64_t> imu_ns_, imu_sequence_, acceleration_sequence_;
    OutputInterface<int> state_output_;
    OutputInterface<bool> enable_request_;
    OutputInterface<int> recovery_phase_output_;
    OutputInterface<int> recovery_failure_output_;
    OutputInterface<int> recovery_native_phase_output_, recovery_native_failure_output_;
    OutputInterface<int> recovery_native_route_output_;
    OutputInterface<bool> recovery_native_motion_released_output_;
    OutputInterface<bool> recovery_support_output_;
    OutputInterface<bool> recovery_geometry_output_;
    OutputInterface<bool> recovery_motion_hold_output_;
    OutputInterface<bool> recovery_sensors_valid_output_, recovery_contact_output_;
    OutputInterface<int> recovery_sensor_issue_output_, recovery_sensor_mask_output_;
    OutputInterface<double> recovery_motor_age_output_, recovery_imu_age_output_;
    OutputInterface<double> recovery_acceleration_age_output_, recovery_blend_output_;
    OutputInterface<double> v6_takeover_blend_output_;
    OutputInterface<double> recovery_height_output_;
    OutputInterface<double> inference_time_us_;
    OutputInterface<double> pd_time_us_;
    std::array<OutputInterface<double>, ObservationLayout::kSize> observation_outputs_;
    std::array<OutputInterface<double>, PolicyAction{}.size()> action_outputs_;
    std::unique_ptr<OnnxPolicy> policy_;
    PolicyProfile policy_profile_ = kV6PolicyProfile;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;

    // q_model = J * q_motor + offset; tau_motor = J^T * tau_model.
    // P[0:4] are active model coordinates, not passive knee hinge coordinates.
    Eigen::Matrix4d leg_jacobian_ = Eigen::Matrix4d::Zero();
    Eigen::Vector4d leg_offset_ = Eigen::Vector4d::Zero();
    Eigen::Vector2d wheel_scale_ = Eigen::Vector2d::Ones();
    std::array<double, 6> nominal_ = DeployedPolicyContract::kNominalPosition;
    std::array<double, 4> hinge_coeff_{}; // [left_hip, left_knee, right_hip, right_knee]
    std::array<double, 2> hinge_bias_{};
    std::array<double, 2> hinge_min_{};
    std::array<double, 2> hinge_max_{};
    bool calibration_ready_ = false;
    bool soft_limits_ready_ = false;
    bool feedback_valid_ = false;
    bool policy_ready_ = false;
    bool imu_alignment_ready_ = false;
    bool auto_enter_rl_ = false;
    bool fault_latched_ = false;
    bool timing_ready_ = false;
    bool recovery_enabled_ = false;
    bool recovery_profile_ready_ = false;
    bool v6_native_profile_ready_ = false;
    bool recovery_started_ = false;
    bool strict_feedback_ = true;
    bool policy_targets_valid_ = false;
    bool motor_feedback_initialized_ = false;
    // IMU body axes -> frozen policy base_link. The source CAD's +90 degree
    // yaw was already applied when exporting the X-forward training asset.
    Eigen::Matrix3d imu_to_base_ = Eigen::Matrix3d::Identity();

    Vector6 q_ = Vector6::Zero();
    Vector6 dq_ = Vector6::Zero();
    std::array<std::uint64_t, 4> previous_motor_sequence_{};
    std::array<std::uint64_t, 4> previous_motor_sample_ns_{};
    std::array<double, 4> previous_motor_angle_{};
    std::array<double, 4> dm_feedback_position_max_{};
    Vector6 targets_ = Vector6::Zero();
    Vector6 policy_targets_ = Vector6::Zero();
    Vector6 v6_takeover_targets_ = Vector6::Zero();
    Eigen::Vector4d v6_prepare_target_ = Eigen::Vector4d::Zero();
    RecoveryController recovery_;
    std::optional<V6RecoveryController> v6_recovery_;
    std::optional<V6RecoveryObserver> v6_recovery_observer_;
    RecoveryPeakBudget recovery_peak_budget_;
    RecoveryCommand recovery_command_;
    std::optional<RecoveryObserver> recovery_observer_;
    RecoveryFeedback last_recovery_feedback_;
    V6RecoveryFeedback v6_recovery_feedback_data_;
    RecoverySensorGuard recovery_sensor_guard_;
    RecoverySensorStatus recovery_sensor_status_;
    RecoveryFailure recovery_failure_latched_ = RecoveryFailure::kNone;
    int v6_recovery_failure_latched_ = 0;
    PolicyObservation observation_{};
    PolicyAction previous_action_{};
    PolicyAction policy_action_{};
    std::size_t last_reset_count_ = 0;
    std::size_t last_policy_tick_ = 0;
    std::optional<std::size_t> last_pd_tick_;
    std::size_t policy_divisor_ = 20;
    std::size_t pd_divisor_ = 5;
    State state_ = State::kInit;
    Clock::time_point jump_start_{};
    Clock::time_point rl_start_{};
    Clock::time_point enable_wait_start_{};
    Clock::time_point recovery_update_time_{};
    Clock::time_point recovery_rl_start_{};
    ControlInterval recovery_interval_;
    ControlInterval recovery_actuation_interval_;
    double recovery_dt_ = DeployedPolicyContract::kControlPeriodSeconds;
    Clock::time_point height_start_{};
    std::optional<Clock::time_point> prepare_stable_since_;
    bool jump_was_requested_ = false;
    double vx_reference_ = 0.0;
    double yaw_reference_ = 0.0;
    double recovery_upright_seconds_ = 0.0;
    double height_reference_ = DeployedPolicyContract::kNominalHeight;
    double height_from_ = DeployedPolicyContract::kNominalHeight;
    double height_target_ = DeployedPolicyContract::kNominalHeight;
    double height_transition_seconds_ = 6.0;
    double wheel_radius_ = 0.06;
    double wheel_track_ = 0.4373;
    double inference_frequency_ = DeployedPolicyContract::kPolicyFrequencyHz;
    double prepare_kp_ = 80.0;
    double prepare_kd_ = 2.0;
    double prepare_max_velocity_ = 1.0;
    double prepare_reach_threshold_ = 0.02;
    double prepare_max_tilt_rad_ = 0.2;
    double prepare_max_angular_velocity_ = 0.35;
    double prepare_max_joint_velocity_ = 0.5;
    double prepare_stable_seconds_ = 0.25;
    double v6_capture_max_leg_error_rad_ = 0.15;
    double v6_capture_max_angular_velocity_ = 1.0;
    double v6_capture_max_leg_velocity_ = 2.0;
    double v6_capture_max_wheel_velocity_ = 5.0;
    double v6_takeover_blend_seconds_ = 0.0;
    double v6_takeover_blend_fraction_ = 0.0;
    double hinge_margin_ = 0.03;
    double recovery_dm_rated_output_rpm_ = 100.0;
    double recovery_dm_rated_torque_nm_ = 20.0;
    double recovery_dm_peak_torque_nm_ = 40.0;
};

} // namespace rmcs::rl
