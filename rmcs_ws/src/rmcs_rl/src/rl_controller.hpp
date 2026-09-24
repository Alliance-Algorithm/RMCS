#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <expected>
#include <memory>
#include <string>
#include <string_view>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

namespace rmcs::rl {

// Actor order P: left hip, left knee-drive, right hip, right knee-drive,
// left wheel, right wheel. This is NOT the CAN bus / control order C.
inline constexpr std::array<const char*, 6> kMotorNames{"left_hip_joint",  "left_knee_joint",
                                                        "right_hip_joint", "right_knee_joint",
                                                        "left_wheel",      "right_wheel"};

enum class State : int { kInit = 0, kIdle = 1, kPrepare = 2, kRl = 3 };
enum class PolicyProfile { kV5Full, kFlat12486 };

// SCUT35 actor contract: each following offset is derived from the preceding block.
// The fixed-size spans in observation.cpp make a layout change fail at compile time.
struct ObservationLayout {
    static constexpr std::size_t kCommand = 0;
    static constexpr std::size_t kHeight = kCommand + 3;
    static constexpr std::size_t kAngularVelocity = kHeight + 1;
    static constexpr std::size_t kProjectedGravity = kAngularVelocity + 3;
    static constexpr std::size_t kJointPosition = kProjectedGravity + 3;
    static constexpr std::size_t kJointVelocity = kJointPosition + 6;
    static constexpr std::size_t kPreviousAction = kJointVelocity + 6;
    static constexpr std::size_t kContext = kPreviousAction + 6;
    static constexpr std::size_t kSize = kContext + 7;

    static constexpr std::size_t kNormal = 0;
    static constexpr std::size_t kJumpRequest = 4;
    static constexpr std::size_t kJumpApex = 5;
    static constexpr std::size_t kJumpElapsed = 6;
};
static_assert(ObservationLayout::kSize == 35);

inline constexpr std::string_view kFlat12486Sha256 =
    "ae58b862be5547195d8c4b3e71aa9be37b147792ebc903c68f032f341d92be6d";

std::expected<PolicyProfile, std::string> parse_policy_profile(std::string_view name);
std::expected<void, std::string> validate_model_identity(
    std::string_view expected_sha, std::string_view actual_sha, PolicyProfile profile);

bool flat_candidate_accepts(
    bool jump, double height, rmcs_msgs::ChassisMode mode,
    const rmcs_description::BaseLink::DirectionVector& command);

class OnnxPolicy {
public:
    explicit OnnxPolicy(const std::string& model_path);
    ~OnnxPolicy();
    OnnxPolicy(const OnnxPolicy&) = delete;
    OnnxPolicy& operator=(const OnnxPolicy&) = delete;
    std::array<float, 6> run(const std::array<float, ObservationLayout::kSize>& observation);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

class RlController final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    RlController();
    void before_updating() override;
    void update() override;

private:
    using Clock = std::chrono::steady_clock;
    using Vector6 = Eigen::Matrix<double, 6, 1>;

    bool read_model_state_();
    bool assemble_observation_();
    void process_action_(const std::array<float, 6>& raw);
    void compute_motor_torques_();
    void apply_soft_limits_(Eigen::Vector4d& leg_torque) const;
    void clear_outputs_();
    void enter_(State next);
    bool update_prepare_();
    void update_command_reference_();
    void publish_state_();

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

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr observation_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr action_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_;
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
    PolicyProfile policy_profile_ = PolicyProfile::kV5Full;
    bool fault_latched_ = false;
    bool timing_ready_ = false;
    Eigen::Matrix3d imu_to_base_ = Eigen::Matrix3d::Identity();

    Vector6 q_ = Vector6::Zero();
    Vector6 dq_ = Vector6::Zero();
    Vector6 targets_ = Vector6::Zero();
    std::array<float, ObservationLayout::kSize> observation_{};
    std::array<float, 6> previous_action_{};
    std::size_t last_reset_count_ = 0;
    std::size_t last_policy_tick_ = 0;
    std::size_t last_pd_tick_ = 0;
    std::size_t policy_divisor_ = 20;
    std::size_t pd_divisor_ = 5;
    State state_ = State::kInit;
    int last_published_state_ = -1;
    Clock::time_point jump_start_{};
    Clock::time_point rl_start_{};
    Clock::time_point height_start_{};
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
    double hinge_margin_ = 0.03;
};

} // namespace rmcs::rl
