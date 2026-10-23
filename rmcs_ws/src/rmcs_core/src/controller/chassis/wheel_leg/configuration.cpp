#include "wheel_leg_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <ranges>
#include <stdexcept>
#include <utility>
#include <vector>

namespace rmcs_core::controller::chassis {

namespace {
template <std::size_t Size>
std::array<double, Size> parameter_array(rclcpp::Node& node, const char* name) {
    const auto v = node.get_parameter(name).as_double_array();
    if (v.size() != Size || !std::ranges::all_of(v, [](double x) { return std::isfinite(x); }))
        throw std::runtime_error(
            std::string{name} + " must have " + std::to_string(Size) + " finite values");
    std::array<double, Size> values;
    std::ranges::copy(v, values.begin());
    return values;
}

} // namespace

WheelLegController::WheelLegController()
    : WheelLegController{node::options()} {}

WheelLegController::WheelLegController(const rclcpp::NodeOptions& options)
    : Node{get_component_name(), options} {
    inference_timeout_ = get_parameter_or("inference_timeout", 0.06);
    if (!std::isfinite(inference_timeout_) || inference_timeout_ <= 0.0)
        throw std::runtime_error("Wheel-leg inference_timeout must be finite and positive");

    constexpr const char* base = "/wheel_leg/";
    for (std::size_t i = 0; i < kMotorNames.size(); ++i) {
        const std::string prefix = std::string{base} + kMotorNames[i];
        register_input(prefix + "/angle", angle_inputs_[i]);
        register_input(prefix + "/velocity", velocity_inputs_[i]);
        register_input(prefix + "/torque", torque_feedback_inputs_[i]);
        register_input(prefix + "/max_torque", max_torque_inputs_[i]);
        register_output(prefix + "/control_torque", torque_outputs_[i], 0.0);
        register_input(prefix + "/feedback_sequence", motor_feedback_sequences_[i], false);
        register_input(prefix + "/feedback_steady_ns", motor_feedback_ns_[i], false);
        if (i < 4) {
            register_input(prefix + "/fault_code", fault_inputs_[i]);
            register_output(
                prefix + "/control_angle", angle_outputs_[i],
                std::numeric_limits<double>::quiet_NaN());
        }
    }
    register_input("/wheel_leg/joint_position_pd", motor_position_pd_, false);
    register_input("/wheel_leg/joint_position_kp", motor_position_kp_, false);
    register_input("/wheel_leg/joint_position_kd", motor_position_kd_, false);
    register_output("/wheel_leg/joint_command_steady_ns", joint_command_ns_, std::uint64_t{0});
    register_input("/wheel_leg/feedback_fresh", feedback_fresh_);
    register_input("/wheel_leg/imu/quaternion", orientation_);
    register_input("/wheel_leg/imu/angular_velocity", gyro_);
    register_input("/chassis/control_velocity", velocity_command_);
    register_input("/chassis/control_height", height_command_);
    register_input("/chassis/control_state", state_command_);
    register_input("/chassis/reset_count", reset_count_);
    register_input("/chassis/control_mode", chassis_mode_);
    register_input("/chassis/jump_request", jump_command_);
    register_input("/chassis/jump_apex_delta", jump_apex_command_);
    register_input("/predefined/update_count", update_count_);
    register_input("/predefined/update_rate", update_rate_);
    register_input("/predefined/timestamp", timestamp_);
    register_input("/wheel_leg/dm_control_ready", dm_control_ready_, false);
    register_input("/wheel_leg/imu/acceleration", acceleration_, false);
    register_input("/wheel_leg/imu/acceleration_steady_ns", acceleration_ns_, false);
    register_input("/wheel_leg/imu/last_steady_ns", imu_ns_, false);
    register_input("/wheel_leg/imu/sequence", imu_sequence_, false);
    register_input("/wheel_leg/imu/acceleration_sequence", acceleration_sequence_, false);
    register_output("/wheel_leg/rl/state", state_output_, std::to_underlying(State::kInit));
    register_output("/wheel_leg/enable_request", enable_request_, false);
    register_output("/wheel_leg/rl/fault_latched", fault_output_, false);
    register_output("/wheel_leg/rl/feedback/sensors_valid", sensors_valid_output_, false);
    register_output("/wheel_leg/rl/feedback/sensor_issue", sensor_issue_output_, 0);
    register_output("/wheel_leg/rl/feedback/sensor_invalid_mask", sensor_mask_output_, 255);
    register_output("/wheel_leg/rl/feedback/motor_age_ms", motor_age_output_, -1.0);
    register_output("/wheel_leg/rl/feedback/imu_age_ms", imu_age_output_, -1.0);
    register_output("/wheel_leg/rl/feedback/acceleration_age_ms", acceleration_age_output_, -1.0);
    register_output("/wheel_leg/rl/performance/pd_us", pd_time_us_, 0.0);
    register_output("/wheel_leg/rl/heading/active", heading_hold_active_output_, false);
    register_output("/wheel_leg/rl/heading/yaw", heading_yaw_output_, 0.0);
    register_output("/wheel_leg/rl/heading/target", heading_target_output_, 0.0);
    register_output("/wheel_leg/rl/heading/error", heading_error_output_, 0.0);
    register_output("/wheel_leg/rl/heading/raw_yaw_rate", heading_raw_rate_output_, 0.0);
    register_output("/wheel_leg/rl/heading/limited_yaw_rate", heading_limited_rate_output_, 0.0);
    // Legacy profiles route physical inputs through YAML. Unified policies consume
    // the coherent, preprocessed frame produced by this controller.
    register_output(
        "/wheel_leg/rl/input/command", policy_command_output_, Eigen::Vector3d::Zero().eval());
    register_output("/wheel_leg/rl/input/height", policy_height_output_, 0.0);
    register_output(
        "/wheel_leg/rl/input/angular_velocity", policy_gyro_output_,
        Eigen::Vector3d::Zero().eval());
    register_output(
        "/wheel_leg/rl/input/projected_gravity", policy_gravity_output_,
        Eigen::Vector3d::Zero().eval());
    register_output("/wheel_leg/rl/input/enable", policy_enable_output_, false);
    register_output(
        "/wheel_leg/rl/input/frame", policy_frame_output_, std::vector<double>(35, 0.0));
    register_output(
        "/wheel_leg/rl/input/frame_sequence", policy_frame_sequence_output_, std::size_t{0});
    register_input(
        "/wheel_leg/rl/observation_contract_sha256", policy_observation_contract_input_, false);
    register_input("/wheel_leg/rl/observation_frame_interface", policy_frame_path_input_, false);
    register_input(
        "/wheel_leg/rl/observation_frame_sequence_interface", policy_frame_sequence_path_input_,
        false);
    register_input("/wheel_leg/rl/valid", policy_valid_input_);
    register_input("/wheel_leg/rl/healthy", policy_healthy_input_);
    register_input("/wheel_leg/rl/action_seq", policy_sequence_input_);
    register_input("/wheel_leg/rl/action_steady_ns", policy_submitted_ns_input_);
    register_input("/wheel_leg/rl/policy_rate", policy_rate_input_);
    register_input("/wheel_leg/rl/model_spec_sha256", policy_model_spec_input_);
    for (std::size_t i = 0; i < kMotorNames.size(); ++i) {
        const std::string model_path = std::string{"/wheel_leg/rl/model/"} + kMotorNames[i];
        register_output(model_path + "/angle", model_angle_outputs_[i], 0.0);
        register_output(model_path + "/velocity", model_velocity_outputs_[i], 0.0);
        const std::string target_path = std::string{"/wheel_leg/rl/target/"} + kMotorNames[i];
        register_input(target_path, policy_target_inputs_[i]);
        register_input(target_path + "/scale", policy_scale_inputs_[i]);
        register_input(target_path + "/clip_min", policy_clip_min_inputs_[i]);
        register_input(target_path + "/clip_max", policy_clip_max_inputs_[i]);
        register_output(
            std::string{"/wheel_leg/rl/action/"} + kMotorNames[i], action_outputs_[i], 0.0);
        register_output(
            std::string{"/wheel_leg/rl/effective_action/"} + kMotorNames[i],
            effective_action_outputs_[i], 0.0);
    }

    calibration_ready_ = get_parameter_or("calibration_ready", false);
    soft_limits_ready_ = get_parameter_or("soft_limits_ready", false);
    imu_alignment_ready_ = get_parameter_or("imu_alignment_ready", false);
    auto_enter_rl_ = get_parameter_or("auto_enter_rl", false);
    unified_full_rl_ = get_parameter_or("unified_full_rl", false);
    const auto profile_name = get_parameter_or<std::string>(
        "control_profile", std::string{DeployedPolicyContract::kName});
    if (profile_name == kV6PolicyProfile.name)
        policy_profile_ = kV6PolicyProfile;
    else if (profile_name == kV5PolicyProfile.name)
        policy_profile_ = kV5PolicyProfile;
    else
        throw std::runtime_error("Unknown wheel-leg control profile: " + profile_name);

    policy_profile_.forward_limit = get_parameter_or("vx_max", policy_profile_.forward_limit);
    policy_profile_.positive_yaw_limit =
        get_parameter_or("yaw_rate_max", policy_profile_.positive_yaw_limit);
    // V6 uses symmetric yaw commands; retain the legacy reverse-spin envelope.
    policy_profile_.negative_yaw_limit =
        policy_profile_.name == kV6PolicyProfile.name
            ? policy_profile_.positive_yaw_limit
            : std::min(policy_profile_.negative_yaw_limit, policy_profile_.positive_yaw_limit);
    leg_target_winding_ = parse_leg_target_winding(
        get_parameter_or<std::string>("leg_winding_version", "independent_nearest_v1"),
        policy_profile_.name);
    heading_hold_enabled_ = get_parameter_or("heading_hold_enabled", true);
    heading_hold_gain_ = get_parameter_or("heading_hold_gain", 5.0);
    heading_hold_yaw_rate_max_ =
        get_parameter_or("heading_hold_yaw_rate_max", 2.0 * std::numbers::pi);

    strict_feedback_ = policy_profile_.name == kV6PolicyProfile.name;
    FeedbackSensorGuardConfig sensor_config;
    sensor_config.motor_age_seconds = get_parameter_or("feedback_motor_age_s", 0.02);
    sensor_config.imu_age_seconds = get_parameter_or("feedback_imu_age_s", 0.02);
    sensor_config.acceleration_age_seconds = get_parameter_or("feedback_acceleration_age_s", 0.03);
    sensor_config.maximum_skew_seconds = get_parameter_or("feedback_sensor_skew_s", 0.01);
    sensor_guard_ = FeedbackSensorGuard{sensor_config};
    if (strict_feedback_) {
        dm_feedback_position_max_ = parameter_array<4>(*this, "dm_feedback_position_max");
        if (!std::ranges::all_of(
                dm_feedback_position_max_, [](double value) { return value > 1.0; }))
            throw std::runtime_error("DM MIT feedback position bounds require valid values");
    }
    const double maximum_interval = get_parameter_or("control_interval_max_s", 0.02);
    if (!std::isfinite(maximum_interval) || maximum_interval <= 0.0 || maximum_interval > 0.05)
        throw std::runtime_error("Control interval must be in (0, 50 ms]");
    control_interval_.set_maximum_interval(
        std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>{maximum_interval}));
    hinge_margin_ = get_parameter_or("hinge_margin", 0.03);
    height_transition_seconds_ = get_parameter_or("height_transition_seconds", 6.0);
    height_command_is_reference_ = get_parameter_or("height_command_is_reference", false);
    height_reference_rate_max_ = get_parameter_or("height_reference_rate_max", 0.02);
    wheel_radius_ = get_parameter_or("wheel_radius", 0.06);
    wheel_track_ = get_parameter_or("wheel_track", 0.4373);
    inference_frequency_ = DeployedPolicyContract::kPolicyFrequencyHz;
    pd_frequency_ = get_parameter_or("pd_frequency", DeployedPolicyContract::kControlFrequencyHz);
    const std::array parameters{
        policy_profile_.forward_limit,
        policy_profile_.negative_yaw_limit,
        policy_profile_.positive_yaw_limit,
        pd_frequency_,
        hinge_margin_,
        height_transition_seconds_,
        height_reference_rate_max_,
        wheel_radius_,
        wheel_track_,
        heading_hold_gain_,
        heading_hold_yaw_rate_max_,
    };
    if (!std::ranges::all_of(parameters, [](double value) { return std::isfinite(value); })
        || policy_profile_.forward_limit <= 0 || policy_profile_.negative_yaw_limit <= 0
        || policy_profile_.positive_yaw_limit <= 0
        || pd_frequency_ < DeployedPolicyContract::kControlFrequencyHz || pd_frequency_ > 1000.0
        || std::fmod(pd_frequency_, DeployedPolicyContract::kControlFrequencyHz) != 0.0
        || hinge_margin_ < 0 || height_transition_seconds_ <= 0 || height_reference_rate_max_ <= 0
        || wheel_radius_ <= 0 || wheel_track_ <= 0 || heading_hold_gain_ < 0
        || heading_hold_yaw_rate_max_ <= 0)
        throw std::runtime_error("Invalid motion limits, PD frequency or robot geometry");

    const auto matrix = parameter_array<16>(*this, "leg_motor_to_model");
    const auto offsets = parameter_array<4>(*this, "leg_model_offsets");
    const auto wheel_scales = parameter_array<2>(*this, "wheel_model_scale");
    const auto hinge = parameter_array<4>(*this, "hinge_coefficients");
    const auto hinge_bias = parameter_array<2>(*this, "hinge_bias");
    const auto hinge_min = parameter_array<2>(*this, "hinge_min");
    const auto hinge_max = parameter_array<2>(*this, "hinge_max");
    const auto nominal = parameter_array<6>(*this, "nominal_model_pos");
    leg_jacobian_ = Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>{matrix.data()};
    leg_offset_ = Eigen::Map<const Eigen::Vector4d>{offsets.data()};
    std::ranges::copy(hinge, hinge_coeff_.begin());
    for (int i = 0; i < 2; ++i) {
        wheel_scale_[i] = wheel_scales[i];
        hinge_bias_[i] = hinge_bias[i];
        hinge_min_[i] = hinge_min[i];
        hinge_max_[i] = hinge_max[i];
    }
    std::ranges::copy(nominal, nominal_.begin());
    for (std::size_t i = 0; i < nominal_.size(); ++i)
        if (std::abs(nominal_[i] - policy_profile_.nominal[i]) > 1e-9)
            throw std::runtime_error(
                "Nominal position does not match the wheel-leg control profile");

    if (calibration_ready_
        && (std::abs(leg_jacobian_.determinant()) < 1e-6 || std::abs(wheel_scale_[0]) < 1e-6
            || std::abs(wheel_scale_[1]) < 1e-6))
        throw std::runtime_error("Calibrated motor-to-model Jacobian must be invertible");
    if (soft_limits_ready_) {
        for (int side = 0; side < 2; ++side) {
            const double b = hinge_coeff_[side * 2 + 1];
            if (std::abs(b) < 1e-6 || hinge_min_[side] >= hinge_max_[side]
                || hinge_max_[side] - hinge_min_[side] < 2 * hinge_margin_)
                throw std::runtime_error("Uncalibrated/invalid hinge soft limits");
            const double at_nominal = hinge_coeff_[side * 2] * nominal_[side * 2]
                                    + b * nominal_[side * 2 + 1] + hinge_bias_[side];
            if (at_nominal < hinge_min_[side] + hinge_margin_
                || at_nominal > hinge_max_[side] - hinge_margin_)
                throw std::runtime_error("Nominal leg posture is outside calibrated hinge limits");
        }
    }
    // Configuration is copied into fixed control-loop state at construction.
    // Reject edits that would otherwise change ROS metadata without changing it.
    parameter_guard_ = add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter>&) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason =
            "RL parameters are startup-only; update the profile and restart the component";
        return result;
    });
    clear_outputs_();
}

} // namespace rmcs_core::controller::chassis
