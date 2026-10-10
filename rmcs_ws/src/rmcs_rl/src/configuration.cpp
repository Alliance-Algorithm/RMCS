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
        table.knee_axis_at_hip_zero =
            calibrated_points(node, prefix + "knee_axis_at_hip_zero", table.delta_rad.size());
        table.wheel_axis_at_hip_zero =
            calibrated_points(node, prefix + "wheel_axis_at_hip_zero", table.delta_rad.size());
        table.slider_slope_m_per_rad =
            finite_parameter_vector(node, prefix + "slider_slope_m_per_rad");
        table.passive_knee_sign = node.get_parameter(prefix + "passive_knee_sign").as_double();
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
    // Existing deployment files keep working while runtime parameters use mechanism names.
    const auto recovery_parameter = [this]<typename T>(const std::string& name, const T& fallback) {
        if (has_parameter(name))
            return get_parameter_or<T>(name, fallback);
        return get_parameter_or<T>("v6_" + name, fallback);
    };
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
        if (i >= 4) {
            register_input(
                prefix + "/last_submitted_torque", wheel_submitted_torque_[i - 4], false);
            register_input(prefix + "/last_submitted_kind", wheel_submitted_kind_[i - 4], false);
            register_input(prefix + "/last_submitted_steady_ns", wheel_submitted_ns_[i - 4], false);
        }
        if (i < 4) {
            register_input(prefix + "/fault_code", fault_inputs_[i]);
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
    register_input("/wheel_leg/imu/last_steady_ns", imu_ns_, false);
    register_input("/wheel_leg/imu/sequence", imu_sequence_, false);
    register_input("/wheel_leg/imu/acceleration_sequence", acceleration_sequence_, false);
    register_output("/wheel_leg/rl/state", state_output_, std::to_underlying(State::kInit));
    register_output("/wheel_leg/enable_request", enable_request_, false);
    register_output("/wheel_leg/rl/recovery/phase", recovery_phase_output_, 0);
    register_output("/wheel_leg/rl/recovery/native_phase", recovery_native_phase_output_, -1);
    register_output("/wheel_leg/rl/recovery/native_failure", recovery_native_failure_output_, 0);
    register_output("/wheel_leg/rl/recovery/native_route", recovery_native_route_output_, -1);
    register_output(
        "/wheel_leg/rl/recovery/native_motion_released", recovery_native_motion_released_output_,
        false);
    register_output("/wheel_leg/rl/recovery/failure", recovery_failure_output_, 0);
    register_output("/wheel_leg/rl/recovery/support_confirmed", recovery_support_output_, false);
    register_output("/wheel_leg/rl/recovery/geometry_valid", recovery_geometry_output_, false);
    register_output("/wheel_leg/rl/recovery/motion_hold", recovery_motion_hold_output_, false);
    register_output("/wheel_leg/rl/recovery/sensors_valid", recovery_sensors_valid_output_, false);
    register_output("/wheel_leg/rl/recovery/sensor_issue", recovery_sensor_issue_output_, 0);
    register_output(
        "/wheel_leg/rl/recovery/sensor_invalid_mask", recovery_sensor_mask_output_, 255);
    register_output("/wheel_leg/rl/recovery/motor_age_ms", recovery_motor_age_output_, -1.0);
    register_output("/wheel_leg/rl/recovery/imu_age_ms", recovery_imu_age_output_, -1.0);
    register_output(
        "/wheel_leg/rl/recovery/acceleration_age_ms", recovery_acceleration_age_output_, -1.0);
    register_output("/wheel_leg/rl/recovery/contact_candidate", recovery_contact_output_, false);
    register_output("/wheel_leg/rl/recovery/height_if_grounded", recovery_height_output_, 0.0);
    register_output("/wheel_leg/rl/recovery/blend", recovery_blend_output_, 0.0);
    register_output("/wheel_leg/rl/v6_takeover/blend_fraction", takeover_blend_output_, 0.0);
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
    const auto profile_name = get_parameter_or<std::string>(
        "control_profile", std::string{DeployedPolicyContract::kName});
    if (profile_name == kV6PolicyProfile.name)
        policy_profile_ = kV6PolicyProfile;
    else if (profile_name == kV5PolicyProfile.name)
        policy_profile_ = kV5PolicyProfile;
    else
        throw std::runtime_error("Unknown wheel-leg control profile: " + profile_name);

    strict_feedback_ = policy_profile_.name == kV6PolicyProfile.name || recovery_enabled_;
    RecoverySensorGuardConfig sensor_config;
    sensor_config.motor_age_seconds = get_parameter_or("recovery_motor_age_s", 0.02);
    sensor_config.imu_age_seconds = get_parameter_or("recovery_imu_age_s", 0.02);
    sensor_config.acceleration_age_seconds = get_parameter_or("recovery_acceleration_age_s", 0.03);
    sensor_config.maximum_skew_seconds = get_parameter_or("recovery_sensor_skew_s", 0.01);
    recovery_sensor_guard_ = RecoverySensorGuard{sensor_config};
    if (strict_feedback_) {
        dm_feedback_position_max_ = parameter_array<4>(
            *this, recovery_enabled_ && policy_profile_.name == kV5PolicyProfile.name
                       ? "recovery_dm_feedback_position_max"
                       : "dm_feedback_position_max");
        if (!std::ranges::all_of(
                dm_feedback_position_max_, [](double value) { return value > 1.0; }))
            throw std::runtime_error("DM MIT feedback position bounds require valid values");
    }
    recovery_dm_rated_output_rpm_ = get_parameter_or("recovery_dm_rated_output_rpm", 100.0);
    recovery_dm_rated_torque_nm_ = get_parameter_or("recovery_dm_rated_torque_nm", 20.0);
    recovery_dm_peak_torque_nm_ = get_parameter_or("recovery_dm_peak_torque_nm", 40.0);
    prepare_kp_ = get_parameter_or(
        "prepare_kp", policy_profile_.name == kV6PolicyProfile.name ? 160.0 : 80.0);
    prepare_kd_ =
        get_parameter_or("prepare_kd", policy_profile_.name == kV6PolicyProfile.name ? 2.5 : 2.0);
    prepare_max_velocity_ = get_parameter_or("prepare_max_velocity", 1.0);
    prepare_reach_threshold_ = get_parameter_or("prepare_reach_threshold", 0.02);
    prepare_max_tilt_rad_ = get_parameter_or("prepare_max_tilt_rad", 0.2);
    prepare_max_angular_velocity_ = get_parameter_or("prepare_max_angular_velocity", 0.35);
    prepare_max_joint_velocity_ = get_parameter_or("prepare_max_joint_velocity", 0.5);
    prepare_stable_seconds_ = get_parameter_or("prepare_stable_seconds", 0.25);
    capture_max_leg_error_rad_ = recovery_parameter("capture_max_leg_error_rad", 0.15);
    capture_max_angular_velocity_ = recovery_parameter("capture_max_angular_velocity", 1.0);
    capture_max_leg_velocity_ = recovery_parameter("capture_max_leg_velocity", 2.0);
    capture_max_wheel_velocity_ = recovery_parameter("capture_max_wheel_velocity", 5.0);
    takeover_blend_seconds_ = recovery_parameter("takeover_blend_seconds", 0.0);
    hinge_margin_ = get_parameter_or("hinge_margin", 0.03);
    height_transition_seconds_ = get_parameter_or("height_transition_seconds", 6.0);
    height_command_is_reference_ = get_parameter_or("height_command_is_reference", false);
    height_reference_rate_max_ = get_parameter_or("height_reference_rate_max", 0.02);
    wheel_radius_ = get_parameter_or("wheel_radius", 0.06);
    wheel_track_ = get_parameter_or("wheel_track", 0.4373);
    inference_frequency_ =
        get_parameter_or("rl_inference_frequency", DeployedPolicyContract::kPolicyFrequencyHz);
    pd_frequency_ = get_parameter_or("pd_frequency", DeployedPolicyContract::kControlFrequencyHz);
    recovery_frequency_ =
        get_parameter_or("recovery_frequency", DeployedPolicyContract::kControlFrequencyHz);
    const std::array parameters{
        inference_frequency_,
        pd_frequency_,
        recovery_frequency_,
        prepare_kp_,
        prepare_kd_,
        prepare_max_velocity_,
        prepare_reach_threshold_,
        prepare_max_tilt_rad_,
        prepare_max_angular_velocity_,
        prepare_max_joint_velocity_,
        prepare_stable_seconds_,
        capture_max_leg_error_rad_,
        capture_max_angular_velocity_,
        capture_max_leg_velocity_,
        capture_max_wheel_velocity_,
        takeover_blend_seconds_,
        hinge_margin_,
        height_transition_seconds_,
        height_reference_rate_max_,
        wheel_radius_,
        wheel_track_,
        recovery_dm_rated_output_rpm_,
        recovery_dm_rated_torque_nm_,
        recovery_dm_peak_torque_nm_,
    };
    if (!std::ranges::all_of(parameters, [](double value) { return std::isfinite(value); })
        || inference_frequency_ != DeployedPolicyContract::kPolicyFrequencyHz
        || pd_frequency_ < DeployedPolicyContract::kControlFrequencyHz || pd_frequency_ > 1000.0
        || std::fmod(pd_frequency_, DeployedPolicyContract::kControlFrequencyHz) != 0.0
        || (recovery_frequency_ != 200.0 && recovery_frequency_ != 1000.0)
        || std::fmod(pd_frequency_, recovery_frequency_) != 0.0 || prepare_kp_ <= 0
        || prepare_kd_ < 0 || prepare_max_velocity_ <= 0 || prepare_reach_threshold_ <= 0
        || prepare_max_tilt_rad_ <= 0 || prepare_max_tilt_rad_ >= std::numbers::pi / 2
        || prepare_max_angular_velocity_ <= 0 || prepare_max_joint_velocity_ <= 0
        || prepare_stable_seconds_ <= 0 || hinge_margin_ < 0 || height_transition_seconds_ <= 0
        || height_reference_rate_max_ <= 0 || wheel_radius_ <= 0 || wheel_track_ <= 0
        || recovery_dm_rated_output_rpm_ <= 0 || recovery_dm_rated_torque_nm_ <= 0
        || recovery_dm_rated_torque_nm_ > recovery_dm_peak_torque_nm_
        || recovery_dm_peak_torque_nm_ > 40.0)
        throw std::runtime_error(
            "Invalid policy/PD/recovery frequency, PREPARE thresholds, or robot geometry");
    // These are bounded upright takeover envelopes, not recovery parameters.
    // Startup overrides may tighten them, but cannot turn flat capture into a
    // folded-pose or high-speed recovery entry.
    if (capture_max_leg_error_rad_ <= 0.0 || capture_max_leg_error_rad_ > 0.25
        || capture_max_angular_velocity_ <= 0.0 || capture_max_angular_velocity_ > 2.0
        || capture_max_leg_velocity_ <= 0.0 || capture_max_leg_velocity_ > 5.0
        || capture_max_wheel_velocity_ <= 0.0 || capture_max_wheel_velocity_ > 10.0
        || (policy_profile_.name == kV6PolicyProfile.name && prepare_max_tilt_rad_ > 0.35))
        throw std::runtime_error("V6 capture thresholds exceed the bounded upright entry domain");
    if (takeover_blend_seconds_ < 0.0 || takeover_blend_seconds_ > 0.3)
        throw std::runtime_error("V6 torque takeover blend must be between 0 and 0.3 seconds");

    const auto matrix = parameter_array<16>(*this, "leg_motor_to_model");
    const auto offsets = parameter_array<4>(*this, "leg_model_offsets");
    const auto wheel_scales = parameter_array<2>(*this, "wheel_model_scale");
    const auto hinge = parameter_array<4>(*this, "hinge_coefficients");
    const auto hinge_bias = parameter_array<2>(*this, "hinge_bias");
    const auto hinge_min = parameter_array<2>(*this, "hinge_min");
    const auto hinge_max = parameter_array<2>(*this, "hinge_max");
    const auto nominal = parameter_array<6>(*this, "nominal_model_pos");
    const auto imu_alignment = parameter_array<9>(*this, "imu_to_base");
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
    imu_to_base_ =
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>{imu_alignment.data()};

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
    base_to_imu_orientation_ = Eigen::Quaterniond{imu_to_base_.transpose()};

    if (policy_profile_.name == kV6PolicyProfile.name) {
        const auto resolve = [](const std::string& name) {
            auto path = std::filesystem::path{name};
            if (!path.is_absolute())
                path =
                    std::filesystem::path{ament_index_cpp::get_package_share_directory("rmcs_rl")}
                    / path;
            return path;
        };
        const auto native = RecoveryProfile::load(
            resolve(recovery_parameter(
                "recovery_profile_path",
                std::string{"models/wheel_leg/deployment/v6_recovery_profiles_v1.json"})),
            resolve(recovery_parameter(
                "height_lookup_path",
                std::string{"models/wheel_leg/deployment/v6_height_lookup_v1.json"})));
        auto recovery_config = native.controller;
        recovery_config.dt = 1.0 / recovery_frequency_;
        recovery_config.blend_seconds =
            recovery_parameter("recovery_blend_seconds", recovery_config.blend_seconds);
        recovery_config.dynamic_takeover = recovery_parameter("recovery_dynamic_takeover", false);
        recovery_config.release_motion_on_takeover =
            recovery_parameter("recovery_release_motion_on_takeover", false);
        recovery_config.dynamic_capture_height_min = recovery_parameter(
            "recovery_capture_height_min", recovery_config.dynamic_capture_height_min);
        recovery_capture_max_tilt_rad_ =
            recovery_parameter("recovery_capture_max_tilt_rad", prepare_max_tilt_rad_);
        recovery_capture_max_angular_velocity_ = recovery_parameter(
            "recovery_capture_max_angular_velocity", capture_max_angular_velocity_);
        if (!std::isfinite(recovery_capture_max_tilt_rad_) || recovery_capture_max_tilt_rad_ <= 0.0
            || recovery_capture_max_tilt_rad_ > 40.0 * std::numbers::pi / 180.0)
            throw std::runtime_error("V6 recovery capture tilt must be within 40 degrees");
        if (!std::isfinite(recovery_capture_max_angular_velocity_)
            || recovery_capture_max_angular_velocity_ < capture_max_angular_velocity_
            || recovery_capture_max_angular_velocity_ > 2.0)
            throw std::runtime_error(
                "V6 upright recovery capture angular velocity must be within 2 rad/s");
        recovery_config.prepare_speed_rad_s = recovery_parameter(
            "recovery_fold_plant_speed_rad_s", recovery_config.prepare_speed_rad_s);
        if (!std::isfinite(recovery_config.prepare_speed_rad_s)
            || recovery_config.prepare_speed_rad_s <= 0.0
            || recovery_config.prepare_speed_rad_s > recovery_config.push_speed_rad_s)
            throw std::runtime_error("V6 FOLD/PLANT speed must not exceed the native push speed");
        joint_reference_recovery_.emplace(recovery_config);
        RCLCPP_INFO(
            get_logger(),
            "Self-righting takeover: %s, bounded upright capture=%d, "
            "FOLD/PLANT %.2f rad/s, capture %.1f deg above %.2f m, "
            "upright gyro %.2f rad/s, motion %s",
            recovery_config.blend_seconds == 0.0 ? "direct" : "200 ms blend",
            recovery_config.dynamic_takeover, recovery_config.prepare_speed_rad_s,
            recovery_capture_max_tilt_rad_ * 180.0 / std::numbers::pi,
            recovery_config.dynamic_capture_height_min, recovery_capture_max_angular_velocity_,
            recovery_config.release_motion_on_takeover ? "released at takeover"
                                                       : "held until stable");
        support_observer_.emplace(native.geometry, recovery_config.dt);
        prepare_target_ = native.controller.nominal;
        joint_reference_profile_ready_ = true;
        if (prepare_kp_ != policy_profile_.leg_kp || prepare_kd_ != policy_profile_.leg_kd)
            throw std::runtime_error("V6 PREPARE gains must match native training: 160 / 2.5");
    }
    if (recovery_enabled_ && recovery_profile_ready_
        && policy_profile_.name == kV5PolicyProfile.name) {
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
                "Wheel output coordinates already include DjiMotor's configured reduction ratio");
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
        const auto mechanism = calibrated_mechanism(*this);
        for (int side = 0; side < 2; ++side) {
            profile.root_axis_y.segment<2>(2 * side).setConstant(
                mechanism.sides[side].hip_axis.y());
            profile.wheel_axis_y[side] = mechanism.sides[side].wheel_axis_at_hip_zero.front().y();
        }
        recovery_ = RecoveryController{profile};
        RecoveryObserverConfig observer_config;
        observer_config.probe_torque_nm = get_parameter_or("recovery_probe_torque_nm", 0.18);
        observer_config.minimum_submitted_torque_nm =
            get_parameter_or("recovery_probe_min_submitted_nm", 0.14);
        observer_config.minimum_feedback_torque_nm =
            get_parameter_or("recovery_probe_min_feedback_nm", 0.025);
        observer_config.feedback_torque_ratio =
            get_parameter_or("recovery_probe_feedback_ratio", 0.25);
        observer_config.pulse_seconds = get_parameter_or("recovery_probe_pulse_s", 0.015);
        observer_config.response_seconds = get_parameter_or("recovery_probe_response_s", 0.005);
        observer_config.quiet_seconds = get_parameter_or("recovery_probe_quiet_s", 0.05);
        observer_config.support_seconds = get_parameter_or("recovery_support_dwell_s", 0.1);
        observer_config.evidence_ttl_seconds =
            get_parameter_or("recovery_probe_evidence_ttl_s", 0.6);
        observer_config.maximum_sample_age_seconds = sensor_config.motor_age_seconds;
        observer_config.maximum_imu_sample_age_seconds = sensor_config.imu_age_seconds;
        observer_config.maximum_submission_age_seconds =
            get_parameter_or("recovery_probe_submission_age_s", 0.02);
        observer_config.maximum_feedback_interval_seconds =
            get_parameter_or("recovery_probe_feedback_interval_s", 0.02);
        observer_config.maximum_wheel_acceleration_rad_s2 =
            get_parameter_or("recovery_probe_max_wheel_accel", 100.0);
        observer_config.maximum_gyro_acceleration_rad_s2 =
            get_parameter_or("recovery_probe_max_gyro_accel", 90.0);
        recovery_observer_.emplace(mechanism, observer_config);
    }

    const std::string model_path = get_parameter_or<std::string>(
        "model_path", get_parameter_or<std::string>("rl_model_path", ""));
    if (!model_path.empty()) {
        auto path = std::filesystem::path{model_path};
        if (!path.is_absolute())
            path = std::filesystem::path{ament_index_cpp::get_package_share_directory("rmcs_rl")}
                 / path;
        policy_ = std::make_unique<OnnxPolicy>(path.string());
        RCLCPP_INFO(
            get_logger(),
            "ONNX model %s, %s control, %.0fHz policy / %.0fHz feedback PD / "
            "%.0fHz recovery",
            path.string().c_str(), std::string{policy_profile_.name}.c_str(), inference_frequency_,
            pd_frequency_, recovery_frequency_);
        policy_ready_ = true;
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

} // namespace rmcs::rl
