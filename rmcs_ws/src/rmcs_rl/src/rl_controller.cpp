#include "rl_controller.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <openssl/evp.h>
#include <pluginlib/class_list_macros.hpp>

namespace rmcs::rl {

namespace {
std::vector<double> parameter_vector(rclcpp::Node& node, const char* name, std::size_t count) {
    const auto v = node.get_parameter(name).as_double_array();
    if (v.size() != count || !std::ranges::all_of(v, [](double x) { return std::isfinite(x); }))
        throw std::runtime_error(
            std::string{name} + " must have " + std::to_string(count) + " finite values");
    return v;
}

bool almost_integer(double value) {
    return std::isfinite(value) && value >= 1.0 && std::abs(value - std::round(value)) < 1e-6;
}

std::string sha256_file(const std::filesystem::path& path) {
    std::ifstream stream{path, std::ios::binary};
    if (!stream)
        throw std::runtime_error("Cannot open ONNX model: " + path.string());
    std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> ctx(EVP_MD_CTX_new(), EVP_MD_CTX_free);
    if (!ctx || EVP_DigestInit_ex(ctx.get(), EVP_sha256(), nullptr) != 1)
        throw std::runtime_error("Failed to initialize model SHA-256");
    std::array<char, 8192> data{};
    while (stream) {
        stream.read(data.data(), data.size());
        const auto size = stream.gcount();
        if (size > 0
            && EVP_DigestUpdate(ctx.get(), data.data(), static_cast<std::size_t>(size)) != 1)
            throw std::runtime_error("Failed to hash ONNX model");
    }
    if (!stream.eof())
        throw std::runtime_error("Failed to read ONNX model for SHA-256");
    std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
    unsigned int length = 0;
    if (EVP_DigestFinal_ex(ctx.get(), digest.data(), &length) != 1 || length != 32)
        throw std::runtime_error("Failed to finalize ONNX model SHA-256");
    std::ostringstream text;
    for (unsigned i = 0; i < length; ++i)
        text << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(digest[i]);
    return text.str();
}
} // namespace

std::expected<PolicyProfile, std::string> parse_policy_profile(std::string_view name) {
    if (name == "v5_full")
        return PolicyProfile::kV5Full;
    if (name == "flat_12486")
        return PolicyProfile::kFlat12486;
    return std::unexpected{std::string{"Unknown RL policy_profile: "} + std::string{name}};
}

std::expected<void, std::string> validate_model_identity(
    std::string_view expected_sha, std::string_view actual_sha, PolicyProfile profile) {
    const auto is_lower_hex = [](char c) {
        return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    };
    if (expected_sha.size() != 64 || !std::ranges::all_of(expected_sha, is_lower_hex))
        return std::unexpected{std::string{"rl_model_sha256 must be 64 lowercase hex digits"}};
    if (expected_sha != actual_sha)
        return std::unexpected{
            std::string{"ONNX SHA-256 does not match the selected model bundle"}};
    const bool flat_model = expected_sha == kFlat12486Sha256;
    const bool flat_profile = profile == PolicyProfile::kFlat12486;
    if (flat_model != flat_profile)
        return std::unexpected{std::string{"policy_profile does not match rl_model_sha256"}};
    return {};
}

bool flat_candidate_accepts(
    bool jump, double height, rmcs_msgs::ChassisMode mode,
    const rmcs_description::BaseLink::DirectionVector& command) {
    if (!std::isfinite(height) || !command.vector.allFinite() || jump
        || std::abs(height - 0.305) > 1e-3 || command.vector.x() < -3.0 || command.vector.x() > 3.0
        || command.vector.z() < -1.05 || command.vector.z() > 4.0 * std::numbers::pi + 1e-3)
        return false;
    if (rmcs_msgs::is_spining(mode)
        && (std::abs(command.vector.x()) > 1e-3 || std::abs(command.vector.y()) > 1e-3))
        return false;
    return std::abs(command.vector.y()) <= 1e-3;
}

RlController::RlController()
    : Node(
          get_component_name(),
          rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
    constexpr const char* base = "/wheel_leg/";
    for (std::size_t i = 0; i < kMotorNames.size(); ++i) {
        const std::string prefix = std::string{base} + kMotorNames[i];
        register_input(prefix + "/angle", angle_inputs_[i]);
        register_input(prefix + "/velocity", velocity_inputs_[i]);
        register_input(prefix + "/torque", torque_feedback_inputs_[i]);
        register_input(prefix + "/max_torque", max_torque_inputs_[i]);
        register_output(prefix + "/control_torque", torque_outputs_[i], 0.0);
        if (i < 4)
            register_input(prefix + "/fault_code", fault_inputs_[i]);
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
    register_output("/wheel_leg/rl/state", state_output_, std::to_underlying(State::kInit));

    calibration_ready_ = get_parameter_or("calibration_ready", false);
    soft_limits_ready_ = get_parameter_or("soft_limits_ready", false);
    imu_alignment_ready_ = get_parameter_or("imu_alignment_ready", false);
    auto_enter_rl_ = get_parameter_or("auto_enter_rl", false);
    const auto profile =
        parse_policy_profile(get_parameter_or<std::string>("policy_profile", "v5_full"));
    if (!profile)
        throw std::runtime_error(profile.error());
    policy_profile_ = *profile;
    prepare_kp_ = get_parameter_or("prepare_kp", 80.0);
    prepare_kd_ = get_parameter_or("prepare_kd", 2.0);
    prepare_max_velocity_ = get_parameter_or("prepare_max_velocity", 1.0);
    prepare_reach_threshold_ = get_parameter_or("prepare_reach_threshold", 0.02);
    hinge_margin_ = get_parameter_or("hinge_margin", 0.03);
    height_transition_seconds_ = get_parameter_or("height_transition_seconds", 6.0);
    wheel_radius_ = get_parameter_or("wheel_radius", 0.06);
    wheel_track_ = get_parameter_or("wheel_track", 0.4373);
    inference_frequency_ = get_parameter_or("rl_inference_frequency", 50.0);
    if (inference_frequency_ != 50.0 || prepare_kp_ <= 0 || prepare_kd_ < 0
        || prepare_max_velocity_ <= 0 || prepare_reach_threshold_ <= 0 || hinge_margin_ < 0
        || height_transition_seconds_ <= 0 || wheel_radius_ <= 0 || wheel_track_ <= 0)
        throw std::runtime_error("Invalid policy frequency, PREPARE gains, or robot geometry");

    const auto matrix = parameter_vector(*this, "leg_motor_to_model", 16);
    const auto offsets = parameter_vector(*this, "leg_model_offsets", 4);
    const auto wheel_scales = parameter_vector(*this, "wheel_model_scale", 2);
    const auto hinge = parameter_vector(*this, "hinge_coefficients", 4);
    const auto hinge_bias = parameter_vector(*this, "hinge_bias", 2);
    const auto hinge_min = parameter_vector(*this, "hinge_min", 2);
    const auto hinge_max = parameter_vector(*this, "hinge_max", 2);
    const auto nominal = parameter_vector(*this, "nominal_model_pos", 6);
    const auto imu_alignment = parameter_vector(*this, "imu_to_base", 9);
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

    state_pub_ = create_publisher<std_msgs::msg::Int32>(
        get_parameter_or<std::string>("rl_state_topic", "/chassis/rl/state"), 5);
    if (get_parameter_or("rl_publish_network_io", false)) {
        observation_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
            get_parameter_or<std::string>("observation_topic", "/wheel_leg/rl/observation"), 5);
        action_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
            get_parameter_or<std::string>("action_topic", "/wheel_leg/rl/action"), 5);
    }
    const std::string model_path = get_parameter_or<std::string>("rl_model_path", "");
    if (!model_path.empty()) {
        auto path = std::filesystem::path{model_path};
        if (!path.is_absolute())
            path = std::filesystem::path{ament_index_cpp::get_package_share_directory("rmcs_rl")}
                 / path;
        const auto expected_sha = get_parameter_or<std::string>("rl_model_sha256", "");
        const auto identity =
            validate_model_identity(expected_sha, sha256_file(path), policy_profile_);
        if (!identity)
            throw std::runtime_error(identity.error());
        policy_ = std::make_unique<OnnxPolicy>(path.string());
        policy_ready_ = true;
    }
    clear_outputs_();
}

void RlController::before_updating() {
    // Executor::start sets /predefined/update_rate AFTER before_updating().
    // Calculate divisors on the first update instead.
    last_reset_count_ = *reset_count_;
    if (!calibration_ready_ || !soft_limits_ready_ || !imu_alignment_ready_ || !policy_ready_)
        RCLCPP_WARN(
            get_logger(),
            "RL disarmed: calibrated motor mapping, hinge limits and ONNX model are required");
}

void RlController::clear_outputs_() {
    for (auto& output : torque_outputs_)
        *output = 0.0;
}

void RlController::enter_(State next) {
    if (state_ == next)
        return;
    state_ = next;
    if (next == State::kIdle || next == State::kInit) {
        clear_outputs_();
        previous_action_.fill(0);
        jump_was_requested_ = false;
        vx_reference_ = yaw_reference_ = 0.0;
        height_reference_ = height_from_ = height_target_ = 0.305;
        height_start_ = *timestamp_;
    } else if (next == State::kPrepare) {
        targets_ = q_;
        previous_action_.fill(0);
        clear_outputs_();
    } else if (next == State::kRl) {
        last_policy_tick_ = std::numeric_limits<std::size_t>::max();
        rl_start_ = *timestamp_;
        jump_was_requested_ = false;
    }
}

void RlController::publish_state_() {
    const int state = std::to_underlying(state_);
    *state_output_ = state;
    if (last_published_state_ != state) {
        std_msgs::msg::Int32 msg;
        msg.data = state;
        state_pub_->publish(msg);
        last_published_state_ = state;
    }
}

bool RlController::update_prepare_() {
    const double dt = 1.0 / *update_rate_;
    bool reached = true;
    for (int i = 0; i < 4; ++i) {
        const double delta = std::remainder(nominal_[i] - targets_[i], 2 * std::numbers::pi);
        targets_[i] += std::clamp(delta, -prepare_max_velocity_ * dt, prepare_max_velocity_ * dt);
        reached &= std::abs(std::remainder(nominal_[i] - q_[i], 2 * std::numbers::pi))
                 < prepare_reach_threshold_;
    }
    targets_[4] = targets_[5] = 0.0;
    return reached;
}

void RlController::update() {
    if (!timing_ready_) {
        const double rate = *update_rate_;
        if (!almost_integer(rate / inference_frequency_) || !almost_integer(rate / 200.0))
            throw std::runtime_error(
                "RMCS update_rate must be an integer multiple of 200Hz and 50Hz");
        policy_divisor_ = static_cast<std::size_t>(std::llround(rate / inference_frequency_));
        pd_divisor_ = static_cast<std::size_t>(std::llround(rate / 200.0));
        timing_ready_ = true;
    }
    const int requested = *state_command_;
    const bool reset = last_reset_count_ != *reset_count_;
    last_reset_count_ = *reset_count_;
    if (reset) {
        fault_latched_ = false;
        enter_(State::kIdle);
    }
    if (requested == 0) {
        enter_(State::kInit);
        clear_outputs_();
        publish_state_();
        return;
    }
    // Auto entry is only for simulation / startup; a manual reset must never
    // re-arm itself merely because auto_enter_rl was configured.
    const bool automatic = requested == 1 && auto_enter_rl_ && *reset_count_ == 0;
    if (requested == 1 && !automatic) {
        enter_(State::kIdle);
        clear_outputs_();
        publish_state_();
        return;
    }
    const bool unsupported_flat_command =
        policy_profile_ == PolicyProfile::kFlat12486
        && !flat_candidate_accepts(
            *jump_command_, *height_command_, *chassis_mode_, *velocity_command_);
    if (unsupported_flat_command)
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Requested motion exceeds the active policy capability profile");
    if ((requested != 2 && requested != 3 && !automatic) || unsupported_flat_command
        || fault_latched_ || !policy_ready_ || !calibration_ready_ || !soft_limits_ready_
        || !imu_alignment_ready_ || !read_model_state_()) {
        if (requested >= 2 && calibration_ready_ && soft_limits_ready_ && policy_ready_
            && imu_alignment_ready_)
            fault_latched_ = true;
        enter_(State::kIdle);
        clear_outputs_();
        publish_state_();
        return;
    }

    if (requested == 2 || (state_ != State::kPrepare && state_ != State::kRl))
        enter_(State::kPrepare);
    if (state_ == State::kPrepare) {
        const bool ready = update_prepare_();
        if (ready && (requested == 3 || automatic))
            enter_(State::kRl);
    }
    if (state_ == State::kRl) {
        const std::size_t tick = *update_count_;
        if (last_policy_tick_ == std::numeric_limits<std::size_t>::max()
            || tick - last_policy_tick_ >= policy_divisor_) {
            if (!assemble_observation_()) {
                fault_latched_ = true;
                enter_(State::kIdle);
                clear_outputs_();
                publish_state_();
                return;
            }
            try {
                process_action_(policy_->run(observation_));
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "ONNX inference failed: %s", e.what());
                fault_latched_ = true;
                enter_(State::kIdle);
                clear_outputs_();
                publish_state_();
                return;
            }
            last_policy_tick_ = tick;
            if (observation_pub_) {
                std_msgs::msg::Float32MultiArray obs_msg, action_msg;
                obs_msg.data.assign(observation_.begin(), observation_.end());
                action_msg.data.assign(previous_action_.begin(), previous_action_.end());
                observation_pub_->publish(obs_msg);
                action_pub_->publish(action_msg);
            }
        }
    }
    const std::size_t tick = *update_count_;
    if (tick - last_pd_tick_ >= pd_divisor_ || last_pd_tick_ == 0) {
        compute_motor_torques_();
        last_pd_tick_ = tick;
    }
    // Other ticks hold the last PD effort; send it again on the CAN bus at 1kHz.
    // clear_outputs_ above is only for states that cannot execute the PD.
    publish_state_();
}

} // namespace rmcs::rl

PLUGINLIB_EXPORT_CLASS(rmcs::rl::RlController, rmcs_executor::Component)
