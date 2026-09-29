#include "rl_controller.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <numbers>
#include <ranges>
#include <stdexcept>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace rmcs::rl {

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

std::vector<double> finite_parameter_vector(rclcpp::Node& node, const std::string& name) {
    const auto values = node.get_parameter(name).as_double_array();
    if (values.empty() || !std::ranges::all_of(values, [](double v) { return std::isfinite(v); }))
        throw std::runtime_error("Missing/nonfinite recovery calibration: " + name);
    return values;
}

std::vector<Eigen::Vector3d>
    calibrated_points(rclcpp::Node& node, const std::string& name, std::size_t count) {
    const auto values = finite_parameter_vector(node, name);
    if (values.size() != 3 * count)
        throw std::runtime_error(name + " must contain exactly three values per point");
    std::vector<Eigen::Vector3d> result;
    result.reserve(count);
    for (std::size_t i = 0; i < count; ++i)
        result.emplace_back(values[3 * i], values[3 * i + 1], values[3 * i + 2]);
    return result;
}

RecoveryMechanism calibrated_mechanism(rclcpp::Node& node) {
    RecoveryMechanism mechanism;
    mechanism.wheel_radius_m = node.get_parameter("wheel_radius").as_double();
    mechanism.spring_stroke_m = node.get_parameter("recovery_spring_stroke_m").as_double();
    const auto coefficients = parameter_array<4>(node, "recovery_spring_force_n");
    std::ranges::copy(coefficients, mechanism.spring_force_n.begin());
    for (int side = 0; side < 2; ++side) {
        auto& table = mechanism.sides[side];
        const std::string prefix = side == 0 ? "recovery_left_" : "recovery_right_";
        table.delta_rad = finite_parameter_vector(node, prefix + "delta_rad");
        table.inner_knee_deg = finite_parameter_vector(node, prefix + "inner_knee_deg");
        table.slider_m = finite_parameter_vector(node, prefix + "slider_m");
        table.wheel_at_hip_zero_m =
            calibrated_points(node, prefix + "wheel_at_hip_zero_m", table.delta_rad.size());
        const auto hip_origin = parameter_array<3>(node, (prefix + "hip_origin_m").c_str());
        const auto hip_axis = parameter_array<3>(node, (prefix + "hip_axis").c_str());
        table.hip_origin_m = Eigen::Vector3d{hip_origin[0], hip_origin[1], hip_origin[2]};
        table.hip_axis = Eigen::Vector3d{hip_axis[0], hip_axis[1], hip_axis[2]};
        table.spring_compression_at_zero_m =
            node.get_parameter(prefix + "spring_compression_at_zero_m").as_double();
    }
    const auto shell = finite_parameter_vector(node, "recovery_shell_points_body_m");
    if (shell.size() < 12 || shell.size() % 3)
        throw std::runtime_error("Calibrated shell geometry needs at least four xyz points");
    mechanism.shell_points_body_m =
        calibrated_points(node, "recovery_shell_points_body_m", shell.size() / 3);
    return mechanism;
}
} // namespace

RlController::RlController()
    : Node{get_component_name(), node::options()} {
    constexpr const char* base = "/wheel_leg/";
    for (std::size_t i = 0; i < kMotorNames.size(); ++i) {
        const std::string prefix = std::string{base} + kMotorNames[i];
        register_input(prefix + "/angle", angle_inputs_[i]);
        register_input(prefix + "/velocity", velocity_inputs_[i]);
        register_input(prefix + "/torque", torque_feedback_inputs_[i]);
        register_input(prefix + "/max_torque", max_torque_inputs_[i]);
        register_output(prefix + "/control_torque", torque_outputs_[i], 0.0);
        if (i < 4) {
            register_input(prefix + "/fault_code", fault_inputs_[i]);
            register_input(prefix + "/feedback_sequence", leg_feedback_sequences_[i], false);
            register_input(prefix + "/feedback_steady_ns", leg_feedback_ns_[i], false);
        }
    }
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
    register_output("/wheel_leg/rl/state", state_output_, std::to_underlying(State::kInit));
    register_output("/wheel_leg/enable_request", enable_request_, false);
    register_output("/wheel_leg/rl/recovery/phase", recovery_phase_output_, 0);
    register_output("/wheel_leg/rl/recovery/failure", recovery_failure_output_, 0);
    register_output("/wheel_leg/rl/recovery/support_confirmed", recovery_support_output_, false);
    register_output("/wheel_leg/rl/recovery/geometry_valid", recovery_geometry_output_, false);
    register_output("/wheel_leg/rl/recovery/motion_hold", recovery_motion_hold_output_, false);
    register_output("/wheel_leg/rl/performance/inference_us", inference_time_us_, 0.0);
    register_output("/wheel_leg/rl/performance/pd_us", pd_time_us_, 0.0);
    for (std::size_t i = 0; i < observation_outputs_.size(); ++i)
        register_output(
            std::string{"/wheel_leg/rl/observation/"} + std::string{kObservationNames[i]},
            observation_outputs_[i], 0.0);
    for (std::size_t i = 0; i < action_outputs_.size(); ++i)
        register_output(
            std::string{"/wheel_leg/rl/action/"} + kMotorNames[i], action_outputs_[i], 0.0);

    calibration_ready_ = get_parameter_or("calibration_ready", false);
    soft_limits_ready_ = get_parameter_or("soft_limits_ready", false);
    imu_alignment_ready_ = get_parameter_or("imu_alignment_ready", false);
    auto_enter_rl_ = get_parameter_or("auto_enter_rl", false);
    recovery_enabled_ = get_parameter_or("recovery_enabled", false);
    recovery_profile_ready_ = get_parameter_or("recovery_profile_ready", false);
    recovery_dm_rated_output_rpm_ = get_parameter_or("recovery_dm_rated_output_rpm", 100.0);
    recovery_dm_rated_torque_nm_ = get_parameter_or("recovery_dm_rated_torque_nm", 20.0);
    recovery_dm_peak_torque_nm_ = get_parameter_or("recovery_dm_peak_torque_nm", 40.0);
    prepare_kp_ = get_parameter_or("prepare_kp", 80.0);
    prepare_kd_ = get_parameter_or("prepare_kd", 2.0);
    prepare_max_velocity_ = get_parameter_or("prepare_max_velocity", 1.0);
    prepare_reach_threshold_ = get_parameter_or("prepare_reach_threshold", 0.02);
    prepare_max_tilt_rad_ = get_parameter_or("prepare_max_tilt_rad", 0.2);
    prepare_max_angular_velocity_ = get_parameter_or("prepare_max_angular_velocity", 0.35);
    prepare_max_joint_velocity_ = get_parameter_or("prepare_max_joint_velocity", 0.5);
    prepare_stable_seconds_ = get_parameter_or("prepare_stable_seconds", 0.25);
    hinge_margin_ = get_parameter_or("hinge_margin", 0.03);
    height_transition_seconds_ = get_parameter_or("height_transition_seconds", 6.0);
    wheel_radius_ = get_parameter_or("wheel_radius", 0.06);
    wheel_track_ = get_parameter_or("wheel_track", 0.4373);
    inference_frequency_ =
        get_parameter_or("rl_inference_frequency", DeployedPolicyContract::kPolicyFrequencyHz);
    const std::array parameters{
        inference_frequency_,
        prepare_kp_,
        prepare_kd_,
        prepare_max_velocity_,
        prepare_reach_threshold_,
        prepare_max_tilt_rad_,
        prepare_max_angular_velocity_,
        prepare_max_joint_velocity_,
        prepare_stable_seconds_,
        hinge_margin_,
        height_transition_seconds_,
        wheel_radius_,
        wheel_track_,
        recovery_dm_rated_output_rpm_,
        recovery_dm_rated_torque_nm_,
        recovery_dm_peak_torque_nm_,
    };
    if (!std::ranges::all_of(parameters, [](double value) { return std::isfinite(value); })
        || inference_frequency_ != DeployedPolicyContract::kPolicyFrequencyHz || prepare_kp_ <= 0
        || prepare_kd_ < 0 || prepare_max_velocity_ <= 0 || prepare_reach_threshold_ <= 0
        || prepare_max_tilt_rad_ <= 0 || prepare_max_tilt_rad_ >= std::numbers::pi / 2
        || prepare_max_angular_velocity_ <= 0 || prepare_max_joint_velocity_ <= 0
        || prepare_stable_seconds_ <= 0 || hinge_margin_ < 0 || height_transition_seconds_ <= 0
        || wheel_radius_ <= 0 || wheel_track_ <= 0 || recovery_dm_rated_output_rpm_ <= 0
        || recovery_dm_rated_torque_nm_ <= 0
        || recovery_dm_rated_torque_nm_ > recovery_dm_peak_torque_nm_
        || recovery_dm_peak_torque_nm_ > 40.0)
        throw std::runtime_error("Invalid policy frequency, PREPARE thresholds, or robot geometry");

    const auto matrix = parameter_array<16>(*this, "leg_motor_to_model");
    const auto offsets = parameter_array<4>(*this, "leg_model_offsets");
    const auto wheel_scales = parameter_array<2>(*this, "wheel_model_scale");
    const auto hinge = parameter_array<4>(*this, "hinge_coefficients");
    const auto hinge_bias = parameter_array<2>(*this, "hinge_bias");
    const auto hinge_min = parameter_array<2>(*this, "hinge_min");
    const auto hinge_max = parameter_array<2>(*this, "hinge_max");
    const auto nominal = parameter_array<6>(*this, "nominal_model_pos");
    const auto imu_alignment = parameter_array<9>(*this, "imu_to_base");
    for (int row = 0; row < 4; ++row) {
        leg_offset_[row] = offsets[row];
        for (int col = 0; col < 4; ++col)
            leg_jacobian_(row, col) = matrix[row * 4 + col];
        hinge_coeff_[row] = hinge[row];
    }
    for (int i = 0; i < 2; ++i) {
        wheel_scale_[i] = wheel_scales[i];
        hinge_bias_[i] = hinge_bias[i];
        hinge_min_[i] = hinge_min[i];
        hinge_max_[i] = hinge_max[i];
    }
    std::ranges::copy(nominal, nominal_.begin());
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
            imu_to_base_(row, col) = imu_alignment[row * 3 + col];

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
    if (imu_alignment_ready_
        && (std::abs(imu_to_base_.determinant() - 1.0) > 1e-3
            || (imu_to_base_.transpose() * imu_to_base_ - Eigen::Matrix3d::Identity()).norm()
                   > 1e-3))
        throw std::runtime_error("imu_to_base must be a calibrated rotation matrix");

    if (recovery_enabled_ && recovery_profile_ready_) {
        recovery_peak_budget_.configure(
            recovery_dm_rated_torque_nm_,
            get_parameter("recovery_above_rated_budget_s").as_double());
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                const double value = leg_jacobian_(row, col);
                if (row == col ? std::abs(std::abs(value) - 1.0) > 0.01 : std::abs(value) > 0.01)
                    throw std::runtime_error(
                        "Recovery requires verified independent 1:1 chain coordinates");
            }
        }
        if ((wheel_scale_.cwiseAbs().array() - 1.0).abs().maxCoeff() > 0.01)
            throw std::runtime_error(
                "Wheel output coordinates already include DjiMotor's 15.8 reduction ratio");
        const auto position_limits = parameter_array<4>(*this, "recovery_dm_feedback_position_max");
        for (int i = 0; i < 4; ++i) {
            if (position_limits[i] <= 1.0)
                throw std::runtime_error("DM MIT feedback position bounds require measured values");
            recovery_dm_feedback_position_max_[i] = position_limits[i];
        }
        RecoveryConfig profile;
        profile.orbit_speed = get_parameter("recovery_orbit_speed").as_double();
        profile.side_speed = get_parameter("recovery_side_speed").as_double();
        profile.rollover_speed = get_parameter("recovery_rollover_speed").as_double();
        profile.capture_speed = get_parameter("recovery_capture_speed").as_double();
        const auto reference = [this](const char* key) {
            const auto values = parameter_array<4>(*this, key);
            return Eigen::Vector4d{values[0], values[1], values[2], values[3]};
        };
        profile.fold = reference("recovery_fold_p4");
        profile.thrust = reference("recovery_thrust_p4");
        profile.side_extended = reference("recovery_side_extended_p4");
        for (int i = 0; i < 4; ++i)
            profile.plant[i] = nominal_[i];
        profile.stand = reference("recovery_stand_p4");
        profile.upright = reference("recovery_upright_p4");
        profile.support_extended = reference("recovery_support_extended_p4");
        profile.upright_support_extended = reference("recovery_upright_support_extended_p4");
        profile.capture_extended = reference("recovery_capture_extended_p4");
        recovery_ = RecoveryController{profile};
        recovery_observer_.emplace(calibrated_mechanism(*this));
    }

    const std::string model_path = get_parameter_or<std::string>("rl_model_path", "");
    if (!model_path.empty()) {
        auto path = std::filesystem::path{model_path};
        if (!path.is_absolute())
            path = std::filesystem::path{ament_index_cpp::get_package_share_directory("rmcs_rl")}
                 / path;
        policy_ = std::make_unique<OnnxPolicy>(path.string());
        policy_ready_ = true;
    }
    clear_outputs_();
}

} // namespace rmcs::rl
