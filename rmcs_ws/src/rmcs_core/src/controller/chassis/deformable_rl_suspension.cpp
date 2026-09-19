#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::chassis {

class DeformableRlSuspension
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DeformableRlSuspension()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        rl_base_ = get_parameter_or<std::string>("rl_base", "/chassis/rl");
        joint_base_path_ = get_parameter_or<std::string>("joint_base_path", "/chassis");
        joint_suffix_ = get_parameter_or<std::string>("joint_suffix", "_joint");
        angle_suffix_ = get_parameter_or<std::string>("angle_suffix", "/physical_angle");

        leg_action_scale_ = get_parameter_or("leg_action_scale", 0.15);
        q_target_min_rad_ = get_parameter_or("q_target_min_rad", 0.0);
        q_target_max_rad_ = get_parameter_or("q_target_max_rad", 1.0563);
        if (q_target_min_rad_ > q_target_max_rad_)
            std::swap(q_target_min_rad_, q_target_max_rad_);

        physical_min_rad_ =
            get_parameter_or("physical_min_rad", -std::numeric_limits<double>::infinity());
        physical_max_rad_ =
            get_parameter_or("physical_max_rad", std::numeric_limits<double>::infinity());
        if (physical_min_rad_ > physical_max_rad_)
            std::swap(physical_min_rad_, physical_max_rad_);

        target_physical_velocity_limit_ = std::max(
            std::abs(get_parameter_or("target_physical_velocity_limit", 180.0)) * std::numbers::pi
                / 180.0,
            1e-6);
        target_physical_acceleration_limit_ = std::max(
            std::abs(get_parameter_or("target_physical_acceleration_limit", 720.0))
                * std::numbers::pi / 180.0,
            1e-6);

        hold_on_invalid_ = get_parameter_or("hold_on_invalid", true);

        register_input("/predefined/update_rate", update_rate_);
        register_input(rl_base_ + "/valid", valid_, false);
        register_input(rl_base_ + "/healthy", healthy_, false);
        register_input("/chassis/active_suspension/active", active_suspension_);
        register_input("/chassis/deformable/reset_count", reset_count_);
        register_input("/chassis/deformable/rl_q_cmd", q_cmd_);
        register_input("/chassis/rl/calibration/high_physical_angle_rad", high_physical_angle_);
        register_input("/chassis/rl/calibration/low_physical_angle_rad", low_physical_angle_);
        register_input("/chassis/rl/calibration/q_max_rad", q_max_rad_);
        register_input("/chassis/deformable/min_angle_deg", min_angle_deg_);
        register_input("/chassis/deformable/max_angle_deg", max_angle_deg_);

        for (std::size_t leg = 0; leg < kLegCount; ++leg)
            register_input(
                rl_base_ + "/action/joint_leg_" + std::to_string(leg + 1), action_[leg], false);

        for (std::size_t corner = 0; corner < kCornerCount; ++corner) {
            register_input(joint_path_(corner, angle_suffix_), physical_angle_[corner]);
            register_input(posture_path_(corner), posture_target_angle_[corner]);
            register_output(
                joint_path_(corner, "/target_physical_angle"), target_angle_[corner], nan_);
            register_output(
                joint_path_(corner, "/target_physical_velocity"), target_velocity_[corner], nan_);
            register_output(
                joint_path_(corner, "/target_physical_acceleration"), target_acceleration_[corner],
                nan_);
            register_output(
                joint_path_(corner, "/control_angle_error"), angle_error_[corner], nan_);
        }
    }

    void before_updating() override {
        if (!update_rate_.ready())
            update_rate_.make_and_bind_directly(1000.0);
        if (!valid_.ready())
            valid_.make_and_bind_directly(0.0);
        if (!healthy_.ready())
            healthy_.make_and_bind_directly(0.0);

        for (std::size_t leg = 0; leg < kLegCount; ++leg)
            if (!action_[leg].ready())
                action_[leg].make_and_bind_directly(nan_);
        for (std::size_t corner = 0; corner < kCornerCount; ++corner) {
            if (!physical_angle_[corner].ready())
                throw std::runtime_error("missing deformable joint physical angle interface");
        }

        last_reset_count_ = *reset_count_;
    }

    void update() override {
        if (*reset_count_ != last_reset_count_) {
            last_reset_count_ = *reset_count_;
            reset_trajectory_state_();
        }

        const bool suspension_active = *active_suspension_;
        if (!suspension_state_initialized_ || suspension_active != last_suspension_active_) {
            reset_trajectory_state_();
            suspension_state_initialized_ = true;
            last_suspension_active_ = suspension_active;
            RCLCPP_INFO(
                get_logger(), "RL suspension authority changed: active=%s",
                suspension_active ? "true" : "false");
        }

        const bool authoritative = suspension_active && *valid_ > 0.5 && *healthy_ > 0.5;

        const double min_angle_deg = *min_angle_deg_;
        const double max_angle_deg = *max_angle_deg_;
        const bool posture_limits_ok = std::isfinite(min_angle_deg) && std::isfinite(max_angle_deg)
                                    && max_angle_deg > min_angle_deg;
        const double posture_min_rad =
            (min_angle_deg - kMinAngleMarginDeg) * std::numbers::pi / 180.0;
        const double posture_max_rad = max_angle_deg * std::numbers::pi / 180.0;
        const double dt = update_dt_();

        if (!suspension_active) {
            std::array<double, kCornerCount> physical_angles{};
            std::array<double, kCornerCount> posture_targets{};
            for (std::size_t corner = 0; corner < kCornerCount; ++corner) {
                physical_angles[corner] = *physical_angle_[corner];
                posture_targets[corner] = *posture_target_angle_[corner];
            }

            if (!posture_limits_ok || !init_trajectory_state_from_feedback_(physical_angles)) {
                publish_nan_targets_();
                return;
            }

            run_joint_trajectory_(posture_targets, dt);
            publish_trajectory_targets_(physical_angles, posture_min_rad, posture_max_rad);
            return;
        }

        const double q_cmd = *q_cmd_;
        const double high_physical = *high_physical_angle_;
        const double low_physical = *low_physical_angle_;
        const double q_max = *q_max_rad_;
        const double span = high_physical - low_physical;
        const bool calibration_ok = std::isfinite(high_physical) && std::isfinite(low_physical)
                                 && std::isfinite(q_max) && q_max > 0.0 && span > 1e-9;

        for (std::size_t leg = 0; leg < kLegCount; ++leg) {
            const auto corner = kCornerForLeg[leg];
            double& target = *target_angle_[corner];
            const double physical = *physical_angle_[corner];
            const double raw_action = *action_[leg];

            if (!authoritative || !calibration_ok || !std::isfinite(q_cmd)
                || !std::isfinite(physical) || !std::isfinite(raw_action)) {
                target = hold_on_invalid_ ? physical : nan_;
                *target_velocity_[corner] = nan_;
                *target_acceleration_[corner] = nan_;
                *angle_error_[corner] =
                    std::isfinite(physical) && std::isfinite(target) ? physical - target : nan_;
                continue;
            }

            const double q_target = std::clamp(
                q_cmd + leg_action_scale_ * raw_action, q_target_min_rad_, q_target_max_rad_);
            const double p_target = std::clamp(
                high_physical - q_target * span / q_max, physical_min_rad_, physical_max_rad_);
            target = p_target;
            *target_velocity_[corner] = nan_;
            *target_acceleration_[corner] = nan_;
            *angle_error_[corner] = physical - target;
        }
    }

private:
    static constexpr std::size_t kLegCount = 4;
    static constexpr std::size_t kCornerCount = 4;
    static constexpr std::size_t kLeftFront = 0;
    static constexpr std::size_t kLeftBack = 1;
    static constexpr std::size_t kRightBack = 2;
    static constexpr std::size_t kRightFront = 3;

    // Real corner order used by the hardware / joint controllers.
    static constexpr const char* kJointName[kCornerCount] = {
        "left_front",
        "left_back",
        "right_back",
        "right_front",
    };

    // RL leg order is joint_leg_1..4 = RF, LF, LB, RB.
    static constexpr std::size_t kCornerForLeg[kLegCount] = {
        kRightFront,
        kLeftFront,
        kLeftBack,
        kRightBack,
    };

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kMinAngleMarginDeg = 5.0;

    std::string joint_path_(std::size_t corner, const std::string& suffix) const {
        return joint_base_path_ + "/" + kJointName[corner] + joint_suffix_ + suffix;
    }

    std::string posture_path_(std::size_t corner) const {
        return std::string{"/chassis/deformable/"} + kJointName[corner]
             + "_joint/posture_target_angle";
    }

    double update_dt_() const {
        if (update_rate_.ready() && std::isfinite(*update_rate_) && *update_rate_ > 1e-6)
            return 1.0 / *update_rate_;
        return 1e-3;
    }

    void reset_trajectory_state_() {
        joint_target_active_.fill(false);
        joint_target_angle_state_rad_.fill(nan_);
        joint_target_velocity_state_rad_.fill(0.0);
        joint_target_acceleration_state_rad_.fill(0.0);
    }

    bool init_trajectory_state_from_feedback_(
        const std::array<double, kCornerCount>& physical_angles) {
        bool any_active = false;
        for (std::size_t corner = 0; corner < kCornerCount; ++corner) {
            if (std::isfinite(physical_angles[corner]) && !joint_target_active_[corner]) {
                joint_target_angle_state_rad_[corner] = physical_angles[corner];
                joint_target_velocity_state_rad_[corner] = 0.0;
                joint_target_acceleration_state_rad_[corner] = 0.0;
                joint_target_active_[corner] = true;
            }
            any_active = any_active || joint_target_active_[corner];
        }
        return any_active;
    }

    void run_joint_trajectory_(const std::array<double, kCornerCount>& targets, double dt) {
        for (std::size_t corner = 0; corner < kCornerCount; ++corner) {
            if (!joint_target_active_[corner])
                continue;

            double& angle = joint_target_angle_state_rad_[corner];
            double& velocity = joint_target_velocity_state_rad_[corner];
            double& acceleration = joint_target_acceleration_state_rad_[corner];
            const double target = targets[corner];
            if (!std::isfinite(target) || !std::isfinite(angle))
                continue;

            const double position_error = target - angle;
            const double stopping_distance =
                velocity * velocity / (2.0 * target_physical_acceleration_limit_);
            double desired_velocity = 0.0;
            if (std::abs(position_error) > 1e-6 && std::abs(position_error) > stopping_distance)
                desired_velocity = std::copysign(target_physical_velocity_limit_, position_error);

            const double velocity_error = desired_velocity - velocity;
            acceleration = std::clamp(
                velocity_error / dt, -target_physical_acceleration_limit_,
                target_physical_acceleration_limit_);
            velocity += acceleration * dt;
            velocity = std::clamp(
                velocity, -target_physical_velocity_limit_, target_physical_velocity_limit_);
            angle += velocity * dt;

            const double next_error = target - angle;
            if ((position_error > 0.0 && next_error < 0.0)
                || (position_error < 0.0 && next_error > 0.0)
                || (std::abs(next_error) < 1e-5 && std::abs(velocity) < 1e-3)) {
                angle = target;
                velocity = 0.0;
                acceleration = 0.0;
            }
        }
    }

    void publish_nan_targets_() {
        for (std::size_t corner = 0; corner < kCornerCount; ++corner) {
            *target_angle_[corner] = nan_;
            *target_velocity_[corner] = nan_;
            *target_acceleration_[corner] = nan_;
            *angle_error_[corner] = nan_;
        }
    }

    void publish_trajectory_targets_(
        const std::array<double, kCornerCount>& physical_angles, double min_angle_rad,
        double max_angle_rad) {
        for (std::size_t corner = 0; corner < kCornerCount; ++corner) {
            if (!joint_target_active_[corner]) {
                *target_angle_[corner] = nan_;
                *target_velocity_[corner] = nan_;
                *target_acceleration_[corner] = nan_;
                *angle_error_[corner] = nan_;
                continue;
            }

            *target_angle_[corner] =
                std::clamp(joint_target_angle_state_rad_[corner], min_angle_rad, max_angle_rad);
            *target_velocity_[corner] = joint_target_velocity_state_rad_[corner];
            *target_acceleration_[corner] = joint_target_acceleration_state_rad_[corner];
            *angle_error_[corner] = std::isfinite(physical_angles[corner])
                                      ? physical_angles[corner] - *target_angle_[corner]
                                      : nan_;
        }
    }

    InputInterface<double> update_rate_;
    InputInterface<double> valid_;
    InputInterface<double> healthy_;
    InputInterface<bool> active_suspension_;
    InputInterface<std::size_t> reset_count_;
    InputInterface<double> q_cmd_;
    InputInterface<double> high_physical_angle_;
    InputInterface<double> low_physical_angle_;
    InputInterface<double> q_max_rad_;
    InputInterface<double> min_angle_deg_;
    InputInterface<double> max_angle_deg_;

    std::array<InputInterface<double>, kLegCount> action_;
    std::array<InputInterface<double>, kCornerCount> physical_angle_;
    std::array<InputInterface<double>, kCornerCount> posture_target_angle_;
    std::array<OutputInterface<double>, kCornerCount> target_angle_;
    std::array<OutputInterface<double>, kCornerCount> target_velocity_;
    std::array<OutputInterface<double>, kCornerCount> target_acceleration_;
    std::array<OutputInterface<double>, kCornerCount> angle_error_;

    std::array<bool, kCornerCount> joint_target_active_ = {false, false, false, false};
    std::array<double, kCornerCount> joint_target_angle_state_rad_ = {nan_, nan_, nan_, nan_};
    std::array<double, kCornerCount> joint_target_velocity_state_rad_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kCornerCount> joint_target_acceleration_state_rad_ = {0.0, 0.0, 0.0, 0.0};

    std::string rl_base_;
    std::string joint_base_path_;
    std::string joint_suffix_;
    std::string angle_suffix_;

    double leg_action_scale_ = 0.15;
    double q_target_min_rad_ = 0.0;
    double q_target_max_rad_ = 1.0563;
    double physical_min_rad_ = -std::numeric_limits<double>::infinity();
    double physical_max_rad_ = std::numeric_limits<double>::infinity();
    double target_physical_velocity_limit_ = 180.0 * std::numbers::pi / 180.0;
    double target_physical_acceleration_limit_ = 720.0 * std::numbers::pi / 180.0;
    bool hold_on_invalid_ = true;

    std::size_t last_reset_count_ = 0;
    bool suspension_state_initialized_ = false;
    bool last_suspension_active_ = false;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::DeformableRlSuspension, rmcs_executor::Component)
