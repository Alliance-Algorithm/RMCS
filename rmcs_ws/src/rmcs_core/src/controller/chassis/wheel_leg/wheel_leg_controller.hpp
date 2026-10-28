#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <numbers>
#include <optional>
#include <source_location>
#include <span>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "feedback_sensor_guard.hpp"
#include "leg_target_winding.hpp"
#include "policy.hpp"
#include "unified_observation.hpp"

namespace rmcs_core::controller::chassis {

// Policy order differs from the hardware bus order.
inline constexpr std::array<const char*, 6> kMotorNames{"left_hip_joint",  "left_knee_joint",
                                                        "right_hip_joint", "right_knee_joint",
                                                        "left_wheel",      "right_wheel"};
static_assert(kMotorNames.size() == PolicyAction{}.size());

// PREPARE only waits for drive readiness and the first policy action.
enum class State : int { kInit = 0, kIdle = 1, kPrepare = 2, kRl = 3 };

class WheelLegController final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegController();
    explicit WheelLegController(const rclcpp::NodeOptions& options);
    ~WheelLegController() override;

    void before_updating() override;
    void update() override;

private:
    using Clock = std::chrono::steady_clock;
    using Vector6 = Eigen::Matrix<double, 6, 1>;

    struct ControlSchedule {
        std::size_t policy_divisor = 20;
        std::size_t pd_divisor = 5;
        std::size_t last_policy_tick = 0;
        std::optional<std::size_t> last_pd_tick;
        bool ready = false;
    };

    void enter_(State next, bool force_reset = false);
    void latch_fault_(
        const char* reason = "control validation failed",
        std::source_location location = std::source_location::current());
    void log_fault_(const char* reason, std::source_location location) const;
    bool evaluate_policy_(std::size_t tick);
    void clear_outputs_();
    void update_state_output_();
    bool read_model_state_();
    Eigen::Quaterniond world_base_orientation_() const;
    void update_command_reference_();
    void reset_heading_hold_();
    bool assemble_observation_();
    bool publish_unified_observation_();
    void publish_model_feedback_();
    std::expected<void, std::string> process_action_(std::span<const double> raw);
    void compute_motor_torques_();
    bool apply_soft_limits_(Eigen::Vector4d& leg_torque) const;
    bool policy_action_fresh_() const;

    std::array<InputInterface<double>, 6> angle_inputs_;
    std::array<InputInterface<double>, 6> velocity_inputs_;
    std::array<InputInterface<double>, 6> max_torque_inputs_;
    std::array<OutputInterface<double>, 6> torque_outputs_;
    std::array<OutputInterface<double>, 4> angle_outputs_;
    InputInterface<bool> motor_position_pd_;
    InputInterface<double> motor_position_kp_, motor_position_kd_;
    OutputInterface<std::uint64_t> joint_command_ns_;
    OutputInterface<bool> joint_torque_override_;
    std::array<InputInterface<std::uint64_t>, 6> motor_feedback_sequences_;
    std::array<InputInterface<std::uint64_t>, 6> motor_feedback_ns_;
    InputInterface<bool> feedback_fresh_;
    InputInterface<Eigen::Quaterniond> orientation_;
    InputInterface<Eigen::Vector3d> gyro_;
    InputInterface<rmcs_description::BaseLink::DirectionVector> velocity_command_;
    InputInterface<double> height_command_;
    InputInterface<int> state_command_;
    InputInterface<std::size_t> reset_count_;
    InputInterface<rmcs_msgs::ChassisMode> chassis_mode_;
    InputInterface<std::size_t> update_count_;
    InputInterface<double> update_rate_;
    InputInterface<Clock::time_point> timestamp_;
    InputInterface<bool> dm_control_ready_;
    InputInterface<std::uint64_t> acceleration_ns_;
    InputInterface<std::uint64_t> imu_ns_, imu_sequence_, acceleration_sequence_;
    OutputInterface<int> state_output_;
    OutputInterface<bool> enable_request_, fault_output_;
    OutputInterface<bool> sensors_valid_output_;
    OutputInterface<int> sensor_issue_output_, sensor_mask_output_;
    OutputInterface<double> motor_age_output_, imu_age_output_, acceleration_age_output_;
    OutputInterface<double> pd_time_us_;
    std::array<OutputInterface<double>, 4> gas_compensation_torque_outputs_;
    std::array<OutputInterface<double>, 2> gas_beta_outputs_, gas_force_outputs_,
        gas_weight_outputs_, gas_extrapolated_outputs_;
    std::array<OutputInterface<double>, PolicyAction{}.size()> action_outputs_;
    std::array<OutputInterface<double>, 6> effective_action_outputs_;
    std::array<OutputInterface<double>, 6> model_angle_outputs_, model_velocity_outputs_;
    OutputInterface<Eigen::Vector3d> policy_command_output_, policy_gyro_output_;
    OutputInterface<Eigen::Vector3d> policy_gravity_output_;
    OutputInterface<double> policy_height_output_;
    OutputInterface<bool> heading_hold_active_output_;
    OutputInterface<double> heading_yaw_output_, heading_target_output_, heading_error_output_;
    OutputInterface<double> heading_raw_rate_output_, heading_limited_rate_output_;
    std::array<InputInterface<double>, 6> policy_target_inputs_, policy_scale_inputs_;
    std::array<InputInterface<double>, 6> policy_clip_min_inputs_, policy_clip_max_inputs_;
    InputInterface<double> policy_valid_input_, policy_healthy_input_, policy_rate_input_;
    InputInterface<std::size_t> policy_sequence_input_;
    InputInterface<std::uint64_t> policy_submitted_ns_input_;
    InputInterface<std::string> policy_model_spec_input_;
    OutputInterface<bool> policy_enable_output_;
    OutputInterface<std::vector<double>> policy_frame_output_;
    OutputInterface<std::size_t> policy_frame_sequence_output_;
    InputInterface<std::string> policy_observation_contract_input_;
    InputInterface<std::string> policy_frame_path_input_, policy_frame_sequence_path_input_;
    UnifiedObservation unified_observation_;
    std::size_t next_policy_frame_sequence_ = 0;
    std::array<double, 6> applied_policy_action_{};
    bool unified_full_rl_ = false;

    Vector6 action_scales_ = Vector6::Ones();
    std::array<double, 6> action_clip_min_{}, action_clip_max_{};
    std::size_t accepted_policy_sequence_ = 0;
    std::optional<Clock::time_point> policy_wait_started_;
    std::optional<Clock::time_point> accepted_policy_submitted_at_;
    double inference_timeout_ = 0.04;
    PolicyProfile policy_profile_ = kV6PolicyProfile;
    LegTargetWinding leg_target_winding_ = LegTargetWinding::kIndependentNearest;
    std::optional<double> heading_target_rad_;
    bool heading_hold_enabled_ = true;
    double heading_hold_gain_ = 5.0;
    double heading_hold_yaw_rate_max_ = 2.0 * std::numbers::pi;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;

    // q_model = J * q_motor + offset; tau_motor = J^T * tau_model.
    // P[0:4] are active model coordinates, not passive knee hinge coordinates.
    Eigen::Matrix4d leg_jacobian_ = Eigen::Matrix4d::Zero();
    Eigen::Vector4d leg_offset_ = Eigen::Vector4d::Zero();
    Eigen::Vector2d wheel_scale_ = Eigen::Vector2d::Ones();
    std::array<double, 6> nominal_ = DeployedPolicyContract::kNominalPosition;
    std::array<double, 4> hinge_coeff_{};
    std::array<double, 2> hinge_bias_{}, hinge_min_{}, hinge_max_{};
    bool calibration_ready_ = false;
    bool soft_limits_ready_ = false;
    bool feedback_valid_ = false;
    bool policy_ready_ = false;
    bool imu_alignment_ready_ = false;
    bool auto_enter_rl_ = false;
    bool fault_latched_ = false;
    bool strict_feedback_ = true;
    bool policy_targets_valid_ = false;
    bool use_motor_position_pd_ = false;
    bool gas_spring_compensation_enabled_ = false;
    double gas_spring_compensation_gain_ = 1.0;

    Vector6 q_ = Vector6::Zero();
    Vector6 dq_ = Vector6::Zero();
    bool leg_feedback_initialized_ = false;
    Vector6 policy_targets_ = Vector6::Zero();
    FeedbackSensorGuard sensor_guard_;
    FeedbackSensorStatus sensor_status_;
    std::array<double, 6> previous_action_{};
    std::size_t last_reset_count_ = 0;
    ControlSchedule schedule_;
    State state_ = State::kInit;
    Clock::time_point height_start_{};
    double vx_reference_ = 0.0;
    double yaw_reference_ = 0.0;
    double height_reference_ = DeployedPolicyContract::kNominalHeight;
    double height_from_ = DeployedPolicyContract::kNominalHeight;
    double height_target_ = DeployedPolicyContract::kNominalHeight;
    double height_transition_seconds_ = 6.0;
    bool height_command_is_reference_ = false;
    double height_reference_rate_max_ = 0.02;
    double inference_frequency_ = DeployedPolicyContract::kPolicyFrequencyHz;
    double pd_frequency_ = DeployedPolicyContract::kControlFrequencyHz;
    double hinge_margin_ = 0.03;
};

} // namespace rmcs_core::controller::chassis
