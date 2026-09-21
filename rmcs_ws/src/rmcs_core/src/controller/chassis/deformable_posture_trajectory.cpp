#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::chassis {

class DeformablePostureTrajectory
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DeformablePostureTrajectory()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        output_angle_suffix_ = get_parameter_or<std::string>(
            "output_angle_suffix", "/traditional_target_physical_angle");
        output_velocity_suffix_ = get_parameter_or<std::string>(
            "output_velocity_suffix", "/traditional_target_physical_velocity");
        output_acceleration_suffix_ = get_parameter_or<std::string>(
            "output_acceleration_suffix", "/traditional_target_physical_acceleration");
        output_error_suffix_ = get_parameter_or<std::string>(
            "output_error_suffix", "/traditional_control_angle_error");

        joint_target_vel_limit_ = std::max(
            deg_to_rad_(std::abs(get_parameter_or("target_physical_velocity_limit", 180.0))),
            1e-6);
        joint_target_acc_limit_ = std::max(
            deg_to_rad_(std::abs(get_parameter_or("target_physical_acceleration_limit", 720.0))),
            1e-6);

        register_input("/predefined/update_rate", update_rate_, false);
        register_input("/chassis/deformable/reset_count", reset_count_, false);
        register_input("/chassis/deformable/min_angle_deg", min_angle_deg_);
        register_input("/chassis/deformable/max_angle_deg", max_angle_deg_);

        for (size_t i = 0; i < kJointCount; ++i) {
            const auto joint_base = std::string{"/chassis/"} + kJointName[i] + "_joint";
            register_input(
                std::string{"/chassis/deformable/"} + kJointName[i] + "_joint/posture_target_angle",
                joint_posture_target_angle_rad_[i]);
            register_input(joint_base + "/physical_angle", joint_physical_angle_[i], false);
            register_output(joint_base + output_angle_suffix_, joint_target_angle_[i], nan_);
            register_output(joint_base + output_velocity_suffix_, joint_target_velocity_[i], nan_);
            register_output(
                joint_base + output_acceleration_suffix_, joint_target_acceleration_[i], nan_);
            register_output(joint_base + output_error_suffix_, joint_angle_error_[i], nan_);
        }
    }

    void before_updating() override {
        if (!update_rate_.ready())
            update_rate_.make_and_bind_directly(1000.0);
        if (!reset_count_.ready())
            reset_count_.make_and_bind_directly(static_cast<size_t>(0));
        for (size_t i = 0; i < kJointCount; ++i)
            if (!joint_physical_angle_[i].ready())
                throw std::runtime_error(
                    "missing deformable chassis feedback interfaces: expected "
                    "/chassis/*_joint/physical_angle");
        reset_all_controls_();
        last_reset_count_ = *reset_count_;
    }

    void update() override {
        if (*reset_count_ != last_reset_count_) {
            reset_all_controls_();
            last_reset_count_ = *reset_count_;
            return;
        }

        std::array<double, kJointCount> physical_angles{};
        physical_angles.fill(nan_);
        for (size_t i = 0; i < kJointCount; ++i)
            if (joint_physical_angle_[i].ready() && std::isfinite(*joint_physical_angle_[i]))
                physical_angles[i] = *joint_physical_angle_[i];

        if (!init_joint_targets_from_feedback_(physical_angles)) {
            reset_all_controls_();
            return;
        }

        std::array<double, kJointCount> posture_targets{};
        for (size_t i = 0; i < kJointCount; ++i)
            posture_targets[i] = *joint_posture_target_angle_rad_[i];

        run_joint_trajectory_(posture_targets, update_dt_());
        publish_joint_targets_(physical_angles);
    }

private:
    static constexpr size_t kJointCount = 4;
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr const char* kJointName[] = {
        "left_front",
        "left_back",
        "right_back",
        "right_front",
    };

    static double deg_to_rad_(double deg) { return deg * std::numbers::pi / 180.0; }

    double update_dt_() const {
        if (update_rate_.ready() && std::isfinite(*update_rate_) && *update_rate_ > 1e-6)
            return 1.0 / *update_rate_;
        return 1e-3;
    }

    void reset_all_controls_() {
        joint_target_active_.fill(false);
        joint_target_angle_state_rad_.fill(nan_);
        joint_target_velocity_state_rad_.fill(0.0);
        joint_target_acceleration_state_rad_.fill(0.0);
        for (size_t i = 0; i < kJointCount; ++i) {
            *joint_target_angle_[i] = nan_;
            *joint_target_velocity_[i] = nan_;
            *joint_target_acceleration_[i] = nan_;
            *joint_angle_error_[i] = nan_;
        }
    }

    bool init_joint_targets_from_feedback_(const std::array<double, kJointCount>& physical_angles) {
        bool any_active = false;
        for (size_t i = 0; i < kJointCount; ++i) {
            if (std::isfinite(physical_angles[i]) && !joint_target_active_[i]) {
                joint_target_angle_state_rad_[i] = physical_angles[i];
                joint_target_velocity_state_rad_[i] = 0.0;
                joint_target_acceleration_state_rad_[i] = 0.0;
                joint_target_active_[i] = true;
            }
            any_active = any_active || joint_target_active_[i];
        }
        return any_active;
    }

    void run_joint_trajectory_(
        const std::array<double, kJointCount>& target_angles_rad, double dt) {
        for (size_t i = 0; i < kJointCount; ++i) {
            if (!joint_target_active_[i])
                continue;

            double& angle_state = joint_target_angle_state_rad_[i];
            double& velocity_state = joint_target_velocity_state_rad_[i];
            double& acceleration_state = joint_target_acceleration_state_rad_[i];
            const double target = target_angles_rad[i];
            if (!std::isfinite(target) || !std::isfinite(angle_state))
                continue;

            const double position_error = target - angle_state;
            const double stopping_distance =
                velocity_state * velocity_state / (2.0 * joint_target_acc_limit_);

            double desired_velocity = 0.0;
            if (std::abs(position_error) > 1e-6 && std::abs(position_error) > stopping_distance)
                desired_velocity = std::copysign(joint_target_vel_limit_, position_error);

            const double velocity_error = desired_velocity - velocity_state;
            acceleration_state = std::clamp(
                velocity_error / dt, -joint_target_acc_limit_, joint_target_acc_limit_);
            velocity_state += acceleration_state * dt;
            velocity_state =
                std::clamp(velocity_state, -joint_target_vel_limit_, joint_target_vel_limit_);
            angle_state += velocity_state * dt;

            const double next_error = target - angle_state;
            if ((position_error > 0.0 && next_error < 0.0)
                || (position_error < 0.0 && next_error > 0.0)
                || (std::abs(next_error) < 1e-5 && std::abs(velocity_state) < 1e-3)) {
                angle_state = target;
                velocity_state = 0.0;
                acceleration_state = 0.0;
            }
        }
    }

    void publish_joint_targets_(const std::array<double, kJointCount>& feedback_angles) {
        const double min_angle_rad = deg_to_rad_(*min_angle_deg_ - 5.0);
        const double max_angle_rad = deg_to_rad_(*max_angle_deg_);

        bool any_active = false;
        for (size_t i = 0; i < kJointCount; ++i)
            any_active = any_active || joint_target_active_[i];
        if (!any_active) {
            reset_all_controls_();
            return;
        }

        for (size_t i = 0; i < kJointCount; ++i) {
            if (!joint_target_active_[i]) {
                *joint_target_angle_[i] = nan_;
                *joint_target_velocity_[i] = nan_;
                *joint_target_acceleration_[i] = nan_;
                *joint_angle_error_[i] = nan_;
                continue;
            }

            *joint_target_angle_[i] =
                std::clamp(joint_target_angle_state_rad_[i], min_angle_rad, max_angle_rad);
            *joint_target_velocity_[i] = joint_target_velocity_state_rad_[i];
            *joint_target_acceleration_[i] = joint_target_acceleration_state_rad_[i];
            *joint_angle_error_[i] = std::isfinite(feedback_angles[i])
                                       ? feedback_angles[i] - *joint_target_angle_[i]
                                       : nan_;
        }
    }

    InputInterface<double> update_rate_;
    InputInterface<size_t> reset_count_;
    InputInterface<double> min_angle_deg_;
    InputInterface<double> max_angle_deg_;
    std::array<InputInterface<double>, kJointCount> joint_posture_target_angle_rad_;
    std::array<InputInterface<double>, kJointCount> joint_physical_angle_;
    std::array<OutputInterface<double>, kJointCount> joint_target_angle_;
    std::array<OutputInterface<double>, kJointCount> joint_target_velocity_;
    std::array<OutputInterface<double>, kJointCount> joint_target_acceleration_;
    std::array<OutputInterface<double>, kJointCount> joint_angle_error_;

    std::string output_angle_suffix_;
    std::string output_velocity_suffix_;
    std::string output_acceleration_suffix_;
    std::string output_error_suffix_;

    std::array<bool, kJointCount> joint_target_active_ = {false, false, false, false};
    std::array<double, kJointCount> joint_target_angle_state_rad_ = {nan_, nan_, nan_, nan_};
    std::array<double, kJointCount> joint_target_velocity_state_rad_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> joint_target_acceleration_state_rad_ = {0.0, 0.0, 0.0, 0.0};

    double joint_target_vel_limit_ = 0.0;
    double joint_target_acc_limit_ = 0.0;
    size_t last_reset_count_ = 0;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::DeformablePostureTrajectory, rmcs_executor::Component)
