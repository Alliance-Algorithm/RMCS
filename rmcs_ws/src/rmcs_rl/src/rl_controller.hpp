#pragma once

#include "policy.hpp"

#include <array>
#include <chrono>
#include <cstddef>
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
    const rmcs_description::BaseLink::DirectionVector& command);

class RlController final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    RlController();
    void before_updating() override;
    void update() override;

private:
    using Clock = std::chrono::steady_clock;
    using Vector6 = Eigen::Matrix<double, 6, 1>;

    bool read_model_state_();
    bool assemble_observation_();
    std::expected<void, std::string> process_action_(const PolicyAction& raw);
    void compute_motor_torques_();
    void apply_soft_limits_(Eigen::Vector4d& leg_torque) const;
    void clear_outputs_();
    void enter_(State next);
    bool update_prepare_();
    void update_command_reference_();
    void update_state_output_();

    std::array<InputInterface<double>, 6> angle_inputs_;
    std::array<InputInterface<double>, 6> velocity_inputs_;
    std::array<InputInterface<double>, 6> max_torque_inputs_;
    std::array<InputInterface<double>, 6> torque_feedback_inputs_;
    std::array<OutputInterface<double>, 6> torque_outputs_;
    std::array<InputInterface<int>, 4> fault_inputs_;
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
    OutputInterface<int> state_output_;
    OutputInterface<double> inference_time_us_;
    OutputInterface<double> pd_time_us_;
    std::array<OutputInterface<double>, ObservationLayout::kSize> observation_outputs_;
    std::array<OutputInterface<double>, PolicyAction{}.size()> action_outputs_;
    std::unique_ptr<OnnxPolicy> policy_;

    // q_model = J * q_motor + offset; tau_motor = J^T * tau_model.
    // P[0:4] are active model coordinates, not passive knee hinge coordinates.
    Eigen::Matrix4d leg_jacobian_ = Eigen::Matrix4d::Zero();
    Eigen::Vector4d leg_offset_ = Eigen::Vector4d::Zero();
    Eigen::Vector2d wheel_scale_ = Eigen::Vector2d::Ones();
    std::array<double, 6> nominal_{0.42, -0.13742282595254576, -0.42, 0.13741557625658019, 0.0,
                                   0.0};
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
    Eigen::Matrix3d imu_to_base_ = Eigen::Matrix3d::Identity();

    Vector6 q_ = Vector6::Zero();
    Vector6 dq_ = Vector6::Zero();
    Vector6 targets_ = Vector6::Zero();
    PolicyObservation observation_{};
    PolicyAction previous_action_{};
    std::size_t last_reset_count_ = 0;
    std::size_t last_policy_tick_ = 0;
    std::size_t last_pd_tick_ = 0;
    std::size_t policy_divisor_ = 20;
    std::size_t pd_divisor_ = 5;
    State state_ = State::kInit;
    Clock::time_point jump_start_{};
    Clock::time_point rl_start_{};
    Clock::time_point height_start_{};
    std::optional<Clock::time_point> prepare_stable_since_;
    bool jump_was_requested_ = false;
    double vx_reference_ = 0.0;
    double yaw_reference_ = 0.0;
    double height_reference_ = 0.305;
    double height_from_ = 0.305;
    double height_target_ = 0.305;
    double height_transition_seconds_ = 6.0;
    double wheel_radius_ = 0.06;
    double wheel_track_ = 0.4373;
    double inference_frequency_ = 50.0;
    double prepare_kp_ = 80.0;
    double prepare_kd_ = 2.0;
    double prepare_max_velocity_ = 1.0;
    double prepare_reach_threshold_ = 0.02;
    double prepare_max_tilt_rad_ = 0.2;
    double prepare_max_angular_velocity_ = 0.35;
    double prepare_max_joint_velocity_ = 0.5;
    double prepare_stable_seconds_ = 0.25;
    double hinge_margin_ = 0.03;
};

} // namespace rmcs::rl
