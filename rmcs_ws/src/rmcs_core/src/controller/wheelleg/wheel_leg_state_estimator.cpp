#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "controller/wheelleg/leg_transmission.hpp"

namespace rmcs_core::controller::wheelleg {

using LegLinkage = rmcs_core::wheelleg::LegLinkage;

// Real-hardware state estimator for the traditional V5.0 stack.
//
// In simulation the MuJoCo bridge (wheelleg_simulation.cpp) is the single
// producer of /wheelleg/state/* using model truth (xpos/xanchor/mj_objectVelocity).
// On the real robot there is no such source, so this component rebuilds the
// reduced-model state from the hardware feedback:
//
//   * attitude (pitch/roll/yaw + rates): IMU quaternion / gravity / gyro,
//     rotated into base_link by `imu_to_base`.
//   * leg shape (knee beta, shank, leg length/angle): the four DM joint encoder
//     angles, through the LegLinkage relative-crank LUT and the reduced
//     thigh/shank geometry.
//   * CoM offset d: the analytic model of 执行路线1_数学解析.md §3.
//
// The real joint zero/sign mapping is the hardware/V6 one
// (hardware_hip_sign/zero, hardware_crank_sign), NOT the sim hip_sign/zero.
//
// Skeleton status: IO + parameters + the mechanical FK are wired. Attitude
// sign conventions and the CoM/velocity channels are marked TODO and must be
// verified on the bench against the real robot before use.
class WheelLegStateEstimator
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    using node = rmcs_utility::NodeMixin;

    WheelLegStateEstimator()
        : Node{get_component_name(), node::options()} {
        load_config_();

        constexpr std::array kJointTopics{
            "left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint"};
        for (std::size_t i = 0; i < kJointTopics.size(); ++i) {
            const std::string prefix = std::string{"/wheel_leg/"} + kJointTopics[i];
            register_input(prefix + "/angle", joint_angle_[i]);
            register_input(prefix + "/velocity", joint_velocity_[i]);
        }
        register_input("/wheel_leg/left_wheel/velocity", wheel_velocity_[0]);
        register_input("/wheel_leg/right_wheel/velocity", wheel_velocity_[1]);

        register_input("/wheel_leg/imu/quaternion", imu_quaternion_);
        register_input("/wheel_leg/imu/sensor_gravity/x", gravity_[0]);
        register_input("/wheel_leg/imu/sensor_gravity/y", gravity_[1]);
        register_input("/wheel_leg/imu/sensor_gravity/z", gravity_[2]);
        register_input("/wheel_leg/imu/sensor_gyro/x", gyro_[0], false);
        register_input("/wheel_leg/imu/sensor_gyro/y", gyro_[1], false);
        register_input("/wheel_leg/imu/sensor_gyro/z", gyro_[2], false);
        register_input("/wheel_leg/feedback_fresh", feedback_fresh_, false);

        register_output("/wheelleg/state/pitch", state_pitch_, nan_);
        register_output("/wheelleg/state/pitch_velocity", state_pitch_velocity_, nan_);
        register_output("/wheelleg/state/roll", state_roll_, nan_);
        register_output("/wheelleg/state/roll_rate", state_roll_rate_, nan_);
        register_output("/wheelleg/state/yaw", state_yaw_, nan_);
        register_output("/wheelleg/state/yaw_rate", state_yaw_rate_, nan_);
        register_output("/wheelleg/state/knee_angle", state_knee_angle_, nan_);
        register_output("/wheelleg/state/knee_velocity", state_knee_velocity_, nan_);
        register_output("/wheelleg/state/shank_angle", state_shank_angle_, nan_);
        register_output("/wheelleg/state/shank_velocity", state_shank_velocity_, nan_);
        register_output("/wheelleg/state/leg_length", state_leg_length_, nan_);
        register_output("/wheelleg/state/leg_angle", state_leg_angle_, nan_);
        register_output("/wheelleg/state/leg_length_l", state_leg_length_l_, nan_);
        register_output("/wheelleg/state/leg_length_r", state_leg_length_r_, nan_);
        register_output("/wheelleg/state/leg_angle_l", state_leg_angle_l_, nan_);
        register_output("/wheelleg/state/leg_angle_r", state_leg_angle_r_, nan_);
        // Reduced hip (raw L_joint1) and base height for the self-righting
        // recovery controller (parity with wheelleg_simulation.cpp).
        register_output("/wheelleg/state/hip_angle", state_hip_angle_, nan_);
        register_output("/wheelleg/state/base_height", state_base_height_, nan_);
        // Wheel-ground normal force per wheel (contact gate). No force sensor on
        // the real robot yet: report 0, so the trajectory controller treats the
        // gate as "wheels loaded" (its documented default).
        register_output("/wheelleg/state/contact_normal_l", state_contact_l_, 0.0);
        register_output("/wheelleg/state/contact_normal_r", state_contact_r_, 0.0);
        register_output("/wheelleg/state/com_offset", state_com_offset_, 0.0);
        register_output("/wheelleg/state/wheel_velocity", state_wheel_velocity_, nan_);
        register_output("/wheelleg/state/velocity", state_velocity_, 0.0);
        register_output("/wheelleg/state/distance", state_distance_, 0.0);
    }

    void before_updating() override {
        has_previous_ = false;
        unwrapped_yaw_ = 0.0;
        distance_ = 0.0;
    }

    void update() override {
        if (!all_required_ready()) {
            publish_nan_();
            return;
        }

        // ------------------------------------------------------------------
        // Attitude from the IMU (base_link is X-forward / Y-left / Z-up).
        // TODO(verify on bench): the exact axis order and sign of these
        // extractions must be checked against the real robot before use.
        // ------------------------------------------------------------------
        const Eigen::Matrix3d r_base_imu = imu_to_base_;
        const Eigen::Vector3d gravity_base =
            r_base_imu * Eigen::Vector3d{*gravity_[0], *gravity_[1], *gravity_[2]};
        const double gx = gravity_base.x(), gy = gravity_base.y(), gz = gravity_base.z();

        const double pitch = std::atan2(gx, -gz);
        const double roll = std::atan2(gy, -gz);
        const Eigen::Quaterniond q{
            imu_quaternion_->w(), imu_quaternion_->x(), imu_quaternion_->y(),
            imu_quaternion_->z()};
        const Eigen::Vector3d gyro_base = r_base_imu * Eigen::Vector3d{
            gyro_[0].ready() ? *gyro_[0] : 0.0, gyro_[1].ready() ? *gyro_[1] : 0.0,
            gyro_[2].ready() ? *gyro_[2] : 0.0};
        const double yaw_wrapped = std::atan2(
            2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
        if (!has_previous_) {
            unwrapped_yaw_ = yaw_wrapped;
        } else {
            double dyaw = yaw_wrapped - previous_yaw_wrapped_;
            while (dyaw > std::numbers::pi)
                dyaw -= 2.0 * std::numbers::pi;
            while (dyaw < -std::numbers::pi)
                dyaw += 2.0 * std::numbers::pi;
            unwrapped_yaw_ += dyaw;
        }
        previous_yaw_wrapped_ = yaw_wrapped;

        // ------------------------------------------------------------------
        // Leg shape from the joint encoders.
        // ------------------------------------------------------------------
        const double beta_l = reduced_knee_(0, *joint_angle_[0], *joint_angle_[1]);
        const double beta_r = reduced_knee_(1, *joint_angle_[2], *joint_angle_[3]);
        const double beta = 0.5 * (beta_l + beta_r);
        const double leg_length_l = leg_length_(beta_l);
        const double leg_length_r = leg_length_(beta_r);
        const double leg_length = 0.5 * (leg_length_l + leg_length_r);

        // Reduced hip alpha = q_hip / hip_sign + hip_zero (inverse of
        // q_hip = hip_sign * (alpha - hip_zero), see wheelleg_standup.yaml).
        const double alpha_l = *joint_angle_[0] / hardware_hip_sign_[0] + hardware_hip_zero_[0];
        const double alpha_r = *joint_angle_[2] / hardware_hip_sign_[1] + hardware_hip_zero_[1];
        const double alpha = 0.5 * (alpha_l + alpha_r);
        // Reduced leg orientation (the second DOF of the refactored 2-DOF solve):
        // alpha = pi - beta - phi  =>  phi = pi - beta - alpha.
        const double shank_l = std::numbers::pi - beta_l - alpha_l;
        const double shank_r = std::numbers::pi - beta_r - alpha_r;
        const double shank = 0.5 * (shank_l + shank_r);

        double knee_rate = 0.0;
        double shank_rate = 0.0;
        if (has_previous_) {
            knee_rate = (beta - previous_beta_) / dt_;
            shank_rate = (shank - previous_shank_) / dt_;
        }
        previous_beta_ = beta;
        previous_shank_ = shank;
        const double pitch_rate = gyro_base.y();
        const double yaw_rate = gyro_base.z();
        const double roll_rate = gyro_base.x();

        // Output sign conventions (match the traditional controllers' sim values).
        *state_pitch_ = state_pitch_sign_ * pitch;
        *state_pitch_velocity_ = state_pitch_sign_ * pitch_rate;
        *state_roll_ = state_roll_sign_ * roll;
        *state_roll_rate_ = state_roll_sign_ * roll_rate;
        *state_yaw_ = state_yaw_sign_ * unwrapped_yaw_;
        *state_yaw_rate_ = state_yaw_sign_ * yaw_rate;
        *state_knee_angle_ = state_knee_sign_ * beta;
        *state_knee_velocity_ = state_knee_sign_ * knee_rate;
        *state_shank_angle_ = state_shank_sign_ * shank;
        *state_shank_velocity_ = state_shank_sign_ * shank_rate;
        *state_leg_length_ = leg_length;
        *state_leg_angle_ = alpha;
        *state_leg_length_l_ = leg_length_l;
        *state_leg_length_r_ = leg_length_r;
        *state_leg_angle_l_ = alpha_l;
        *state_leg_angle_r_ = alpha_r;
        // Raw L_joint1 (sim reports the reduced hip == raw L_joint1 here).
        *state_hip_angle_ = state_hip_sign_ * *joint_angle_[0];
        // TODO: base origin height needs wheel contact / leg-angle projection.
        // Placeholder: leg length + wheel radius (vertical leg assumption).
        *state_base_height_ = leg_length + wheel_radius_;
        const double omega = 0.5 * (*wheel_velocity_[0] + *wheel_velocity_[1]);
        *state_wheel_velocity_ = state_wheel_sign_ * omega;

        // TODO: CoM offset d = (C_veh - A)_forward from 执行路线1_数学解析.md §3,
        // and wheel-odometry velocity/distance. Zero placeholders for now.
        *state_com_offset_ = 0.0;
        *state_velocity_ = 0.0;
        distance_ += 0.0;
        *state_distance_ = distance_;

        has_previous_ = true;
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    bool all_required_ready() const {
        for (const auto& a : joint_angle_)
            if (!a.ready())
                return false;
        for (const auto& v : joint_velocity_)
            if (!v.ready())
                return false;
        if (!imu_ready_())
            return false;
        return true;
    }

    bool imu_ready_() const {
        return imu_quaternion_.ready() && gravity_[0].ready() && gravity_[1].ready()
            && gravity_[2].ready() && wheel_velocity_[0].ready() && wheel_velocity_[1].ready();
    }

    void publish_nan_() {
        *state_pitch_ = nan_;
        *state_pitch_velocity_ = nan_;
        *state_roll_ = nan_;
        *state_roll_rate_ = nan_;
        *state_yaw_ = nan_;
        *state_yaw_rate_ = nan_;
        *state_knee_angle_ = nan_;
        *state_knee_velocity_ = nan_;
        *state_shank_angle_ = nan_;
        *state_shank_velocity_ = nan_;
        *state_leg_length_ = nan_;
        *state_leg_angle_ = nan_;
        *state_hip_angle_ = nan_;
        *state_base_height_ = nan_;
        *state_wheel_velocity_ = nan_;
    }

    // Relative crank c = crank_sign * (q_crank - q_hip) -> interior knee beta.
    [[nodiscard]] double reduced_knee_(std::size_t side, double q_hip, double q_crank) const {
        const double c = hardware_crank_sign_.at(side) * (q_crank - q_hip);
        const double beta = linkage_.knee_from_crank(c);
        return std::clamp(beta, knee_limit_min_, knee_limit_max_);
    }

    [[nodiscard]] double leg_length_(double beta) const {
        return std::sqrt(
            thigh_length_ * thigh_length_ + shank_length_ * shank_length_
            - 2.0 * thigh_length_ * shank_length_ * std::cos(beta));
    }

    void load_config_() {
        thigh_length_ = param_or("thigh_length", 0.21);
        shank_length_ = param_or("shank_length", 0.25005);
        wheel_radius_ = param_or("wheel_radius", 0.06);
        knee_limit_min_ = param_or("knee_limit_min", 0.6981317007977318);
        knee_limit_max_ = param_or("knee_limit_max", 1.8325957145940461);
        dt_ = param_or("dt", 0.001);
        hardware_hip_sign_ = get_parameter("hardware_hip_sign").as_double_array();
        hardware_hip_zero_ = get_parameter("hardware_hip_zero").as_double_array();
        hardware_crank_sign_ = get_parameter("hardware_crank_sign").as_double_array();
        if (hardware_hip_sign_.size() != 2 || hardware_hip_zero_.size() != 2
            || hardware_crank_sign_.size() != 2)
            throw std::invalid_argument(
                "hardware_hip_sign/zero/crank_sign require two values");
        const auto imu = get_parameter("imu_to_base").as_double_array();
        if (imu.size() != 9)
            throw std::invalid_argument("imu_to_base requires nine values");
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j)
                imu_to_base_(i, j) = imu[3 * i + j];
        state_pitch_sign_ = param_or("state_pitch_sign", 1.0);
        state_roll_sign_ = param_or("state_roll_sign", 1.0);
        state_yaw_sign_ = param_or("state_yaw_sign", 1.0);
        state_knee_sign_ = param_or("state_knee_sign", -1.0);
        state_shank_sign_ = param_or("state_shank_sign", 1.0);
        state_hip_sign_ = param_or("state_hip_sign", 1.0);
        state_wheel_sign_ = param_or("state_wheel_sign", 1.0);
    }

    [[nodiscard]] double param_or(const std::string& name, double fallback) {
        return get_parameter_or(name, fallback);
    }

    LegLinkage linkage_;
    double thigh_length_ = 0.21, shank_length_ = 0.25005, wheel_radius_ = 0.06;
    double knee_limit_min_ = 0.6981317007977318, knee_limit_max_ = 1.8325957145940461;
    double dt_ = 0.001;
    std::vector<double> hardware_hip_sign_{1.0, 1.0};
    std::vector<double> hardware_hip_zero_{0.0, 0.0};
    std::vector<double> hardware_crank_sign_{1.0, 1.0};
    Eigen::Matrix3d imu_to_base_ = Eigen::Matrix3d::Identity();
    double state_pitch_sign_ = 1.0, state_roll_sign_ = 1.0, state_yaw_sign_ = 1.0;
    double state_knee_sign_ = -1.0, state_shank_sign_ = 1.0, state_hip_sign_ = 1.0;
    double state_wheel_sign_ = 1.0;

    std::array<InputInterface<double>, 4> joint_angle_;
    std::array<InputInterface<double>, 4> joint_velocity_;
    std::array<InputInterface<double>, 2> wheel_velocity_;
    InputInterface<Eigen::Quaterniond> imu_quaternion_;
    std::array<InputInterface<double>, 3> gravity_;
    std::array<InputInterface<double>, 3> gyro_;
    InputInterface<bool> feedback_fresh_;

    OutputInterface<double> state_pitch_{}, state_pitch_velocity_{}, state_roll_{}, state_roll_rate_{};
    OutputInterface<double> state_yaw_{}, state_yaw_rate_{};
    OutputInterface<double> state_knee_angle_{}, state_knee_velocity_{};
    OutputInterface<double> state_shank_angle_{}, state_shank_velocity_{};
    OutputInterface<double> state_leg_length_{}, state_leg_angle_{};
    OutputInterface<double> state_leg_length_l_{}, state_leg_length_r_{};
    OutputInterface<double> state_leg_angle_l_{}, state_leg_angle_r_{};
    OutputInterface<double> state_hip_angle_{}, state_base_height_{};
    OutputInterface<double> state_contact_l_{}, state_contact_r_{};
    OutputInterface<double> state_com_offset_{}, state_wheel_velocity_{};
    OutputInterface<double> state_velocity_{}, state_distance_{};

    bool has_previous_ = false;
    double previous_yaw_wrapped_ = 0.0;
    double previous_beta_ = 0.0, previous_shank_ = 0.0;
    double unwrapped_yaw_ = 0.0;
    double distance_ = 0.0;
};

} // namespace rmcs_core::controller::wheelleg

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::wheelleg::WheelLegStateEstimator, rmcs_executor::Component)
